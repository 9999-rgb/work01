// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
//
// xczs_ros_global_args_guard
// --------------------------
// Gazebo world plugin that guards the gzserver process against global rcl
// argument pollution from gazebo_ros2_control.
//
// gazebo_ros2_control (humble libgazebo_ros2_control.so) rewrites the
// gzserver process-global rcl arguments on every robot spawn:
//
//   {"--ros-args", "--params-file", <toolset yaml>,
//    "--param", "robot_description:=<urdf>",
//    "--remap", "__ns:=/xczs"}
//
// rcl_node_init applies that ``__ns`` default remap to *every* node the
// gzserver process creates afterwards.  Sensor plugins of the NEXT robot spawn
// load before the ros2_control plugin, so a sensor that should register under
// ``/xczs/lidar/scan`` is silently truncated to ``/xczs/scan``, which breaks
// Nav2, the Web camera/LiDAR feeds and the toolset switch readiness check
// after the first toolset switch.  (Verified live in the running world: the
// second robot's lidar publishes on ``/xczs/scan`` instead of
// ``/xczs/lidar/scan``.)
//
// This plugin lives entirely inside gzserver and exposes a Trigger service
// that resets the process-global rcl arguments to a clean ``{"--ros-args"}``.
// The toolset supervisor calls it immediately before every robot spawn so
// each new robot's sensor plugins load with a clean default namespace.  The
// ros2_control plugin re-injects its own (namespace-truncating) arguments when
// its controller manager node is created, but that happens after the sensors
// have already bound their namespaces, so it no longer corrupts them.

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <rcl/allocator.h>
#include <rcl/arguments.h>
#include <rcl/context.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace gazebo
{
class XczsRosGlobalArgsGuard : public WorldPlugin
{
public:
  XczsRosGlobalArgsGuard() = default;

  ~XczsRosGlobalArgsGuard() override
  {
    stop_.store(true);
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void Load(physics::WorldPtr /*_world*/, sdf::ElementPtr /*_sdf*/) override
  {
    if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
      // gazebo_ros_init normally initializes rclcpp before the world loads;
      // this guard keeps the world usable if that plugin was not injected.
      rclcpp::init(0, nullptr);
    }

    // Create the node with explicit namespace and WITHOUT inheriting the
    // process-global arguments, so the node itself is immune to the very
    // pollution it is responsible for removing (the gazebo_ros2_control
    // ``__ns:=/xczs`` default-namespace remap).
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    node_ = std::make_shared<rclcpp::Node>("global_args_guard", "/xczs", options);

    // ``~/reset`` (not a bare relative ``reset``): rcl resolves a relative
    // service name against the node's *namespace only*, so ``reset`` would
    // land on ``/xczs/reset``.  ``~/`` expands to the node's fully-qualified
    // name and gives exactly the address the supervisor waits for,
    // ``/xczs/global_args_guard/reset``.  (Verified against the installed
    // rcl/rclcpp with both clean and polluted global arguments.)
    service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = false;
        // The default context is the one initialized by gazebo_ros_init and
        // shared by every node created in the gzserver process.  humble's rcl
        // does not export rcl_get_default_context, so reach it through the
        // guard node's own rcl node handle.
        rcl_node_t * rcl_node = node_->get_node_base_interface()->get_rcl_node_handle();
        rcl_context_t * ctx = (rcl_node != nullptr) ? rcl_node->context : nullptr;
        if (ctx == nullptr) {
          response->message = "gzserver has no default rcl context";
          RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
          return;
        }
        rcl_allocator_t alloc = rcl_get_default_allocator();
        rcl_arguments_t clean = rcl_get_zero_initialized_arguments();
        // A lone "--ros-args" marker parses to an argument set with no remap
        // rules and no parameter files: exactly the clean default namespace we
        // want every subsequent node in gzserver to use.
        const char * argv[] = {"--ros-args"};
        rcl_ret_t ret = rcl_parse_arguments(1, argv, alloc, &clean);
        if (ret != RCL_RET_OK) {
          response->message =
            std::string("rcl_parse_arguments failed: ") + rcl_get_error_string().str;
          rcl_reset_error();
          RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
          return;
        }
        // Swap the process-global arguments to the clean set.  rcl_arguments_t
        // is a single-implementation-pointer struct, so this assignment is one
        // pointer-sized store: a concurrent rcl_node_init (if one were in
        // flight on another gzserver thread) sees either the old or the new
        // pointer, never a torn value, so the swap itself cannot race.
        ctx->global_arguments = clean;
        // The old (polluting) arguments are deliberately NOT finalized here.
        // rcl exposes no lock around context->global_arguments, so a
        // rcl_node_init that read the pre-swap pointer on another thread could
        // still be traversing it while rcl_arguments_fini frees it -- a
        // use-after-free that would take down gzserver.  The supervisor
        // serializes reset-then-spawn today, but nothing stops a future
        // unsynchronized caller from racing it.  The swapped-out set is never
        // read again once the swap is done, so leaving it to the process
        // lifetime is safe; the leak is one small argument set per toolset
        // switch (gazebo_ros2_control already leaks its parse of these on
        // every spawn and never finalizes them).
        response->success = true;
        response->message = "gzserver global rcl arguments reset to --ros-args";
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_.load()) {
        executor_->spin_once(std::chrono::milliseconds(200));
      }
      executor_->remove_node(node_);
    });
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_{false};
};

GZ_REGISTER_WORLD_PLUGIN(XczsRosGlobalArgsGuard)
}  // namespace gazebo
