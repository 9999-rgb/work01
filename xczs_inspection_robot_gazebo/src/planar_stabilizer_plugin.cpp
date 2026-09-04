#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace xczs_inspection_robot_control
{

class PlanarStabilizerPlugin final : public gazebo::ModelPlugin
{
public:
  ~PlanarStabilizerPlugin() override
  {
    const auto callback_lifetime = callback_lifetime_;
    if (callback_lifetime) {
      std::lock_guard<std::mutex> lock(callback_lifetime->mutex);
      callback_lifetime->shutting_down = true;
      callback_lifetime->owner = nullptr;
    }

    // Stop both world-update sources before any Gazebo object is released.
    // A callback that already holds a lease is allowed to finish below.
    joint_snapshot_connection_.reset();
    update_connection_.reset();
    cmd_vel_subscription_.reset();
    grasp_active_subscription_.reset();
    joint_hold_subscription_.reset();

    if (callback_lifetime) {
      std::unique_lock<std::mutex> lock(callback_lifetime->mutex);
      callback_lifetime->condition.wait(
        lock, [&callback_lifetime]() {
          return callback_lifetime->active_callbacks == 0U;
        });
    }
  }

  void Load(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_) {
      gzerr << "Planar stabilizer could not create a ROS 2 node.\n";
      return;
    }
    const auto grasp_active_topic = sdf->Get<std::string>(
      "grasp_active_topic", "/xczs/cabinet/grasp_active").first;
    if (grasp_active_topic.empty() || grasp_active_topic.front() != '/') {
      gzerr << "Planar stabilizer grasp_active_topic must be absolute.\n";
      return;
    }
    const auto joint_hold_topic = sdf->Get<std::string>(
      "joint_hold_topic", "/xczs/joint_hold_enabled").first;
    if (joint_hold_topic.empty() || joint_hold_topic.front() != '/') {
      gzerr << "Planar stabilizer joint_hold_topic must be absolute.\n";
      return;
    }
    const auto cmd_vel_topic = sdf->Get<std::string>(
      "cmd_vel_topic", "/xczs/cmd_vel").first;
    if (cmd_vel_topic.empty() || cmd_vel_topic.front() != '/') {
      gzerr << "Planar stabilizer cmd_vel_topic must be absolute.\n";
      return;
    }
    const auto grasp_active = grasp_active_;
    grasp_active_subscription_ =
      ros_node_->create_subscription<std_msgs::msg::Bool>(
      grasp_active_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [grasp_active](const std_msgs::msg::Bool::SharedPtr message) {
        grasp_active->store(message->data);
      });
    const auto joint_hold_enabled = joint_hold_enabled_;
    joint_hold_subscription_ =
      ros_node_->create_subscription<std_msgs::msg::Bool>(
      joint_hold_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [joint_hold_enabled](const std_msgs::msg::Bool::SharedPtr message) {
        joint_hold_enabled->store(message->data);
      });
    const auto base_command_moving = base_command_moving_;
    cmd_vel_subscription_ =
      ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      10,
      [base_command_moving](
        const geometry_msgs::msg::Twist::SharedPtr message)
      {
        constexpr double command_epsilon = 1.0e-6;
        const bool command_is_finite =
        std::isfinite(message->linear.x) &&
        std::isfinite(message->linear.y) &&
        std::isfinite(message->angular.z);
        // Fail open for malformed input: never pin the chassis merely because
        // an invalid motion command cannot be interpreted as a velocity.
        const bool command_is_moving = !command_is_finite || (
          std::abs(message->linear.x) > command_epsilon ||
          std::abs(message->linear.y) > command_epsilon ||
          std::abs(message->angular.z) > command_epsilon);
        base_command_moving->store(command_is_moving);
      });
    settling_duration_ =
      sdf->Get<double>("settling_duration", 0.5).first;
    load_held_joints(sdf);
    keep_slider_links_awake();
    held_joint_positions_.resize(held_joints_.size(), 0.0);
    schedule_height_lock();

    callback_lifetime_ = std::make_shared<UpdateCallbackLifetime>();
    callback_lifetime_->owner = this;
    const auto callback_lifetime = callback_lifetime_;
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      [callback_lifetime]() {
        auto * owner = acquire_update_callback(callback_lifetime);
        if (!owner) {
          return;
        }
        UpdateCallbackLease lease(callback_lifetime);
        std::lock_guard<std::mutex> lock(owner->update_mutex_);
        owner->stabilize();
      });
    joint_snapshot_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
      [callback_lifetime](const gazebo::common::UpdateInfo & update_info) {
        auto * owner = acquire_update_callback(callback_lifetime);
        if (!owner) {
          return;
        }
        UpdateCallbackLease lease(callback_lifetime);
        std::lock_guard<std::mutex> lock(owner->update_mutex_);
        owner->capture_joint_positions(update_info);
      });
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    height_is_locked_ = false;
    has_joint_snapshot_ = false;
    planar_pose_is_locked_ = false;
    planar_motion_observed_at_begin_ = false;
    base_command_moving_->store(false);
    schedule_height_lock();
    // A world reset reinstates ODE's per-body auto-disable defaults, so the
    // slider links must be re-exempted after every reset, not only at spawn.
    keep_slider_links_awake();
  }

private:
  struct UpdateCallbackLifetime
  {
    std::mutex mutex;
    std::condition_variable condition;
    PlanarStabilizerPlugin * owner{nullptr};
    std::size_t active_callbacks{0U};
    bool shutting_down{false};
  };

  class UpdateCallbackLease
  {
public:
    explicit UpdateCallbackLease(
      std::shared_ptr<UpdateCallbackLifetime> lifetime)
    : lifetime_(std::move(lifetime)) {}

    UpdateCallbackLease(const UpdateCallbackLease &) = delete;
    UpdateCallbackLease & operator=(const UpdateCallbackLease &) = delete;

    ~UpdateCallbackLease()
    {
      std::lock_guard<std::mutex> lock(lifetime_->mutex);
      if (lifetime_->active_callbacks > 0U) {
        --lifetime_->active_callbacks;
      }
      lifetime_->condition.notify_all();
    }

private:
    std::shared_ptr<UpdateCallbackLifetime> lifetime_;
  };

  static PlanarStabilizerPlugin * acquire_update_callback(
    const std::shared_ptr<UpdateCallbackLifetime> & lifetime)
  {
    std::lock_guard<std::mutex> lock(lifetime->mutex);
    if (lifetime->shutting_down || !lifetime->owner) {
      return nullptr;
    }
    ++lifetime->active_callbacks;
    return lifetime->owner;
  }

  void load_held_joints(const sdf::ElementPtr & sdf)
  {
    if (!sdf->HasElement("hold_joint")) {
      return;
    }

    sdf::ElementPtr joint_element = sdf->GetElement("hold_joint");
    while (joint_element) {
      const std::string joint_name = joint_element->Get<std::string>();
      const gazebo::physics::JointPtr joint = model_->GetJoint(joint_name);
      if (joint) {
        // ODE's SliderJoint backend cannot service the anchor update reached
        // by Joint::SetPosition.  Tool sliders are driven through the
        // ros2_control effort path, so they must not also be restored here on
        // every physics cycle.  Besides being redundant, that second write
        // floods Gazebo's server log with
        // ``ODESliderJoint::Anchor not implemented``.
        const bool is_slider =
          (joint->GetType() & gazebo::physics::Base::SLIDER_JOINT) != 0U;
        if (is_slider) {
          RCLCPP_INFO(
            ros_node_->get_logger(),
            "Planar stabilizer leaves prismatic joint '%s' to "
            "gazebo_ros2_control.",
            joint_name.c_str());
        } else {
          held_joints_.push_back(joint);
        }
      } else {
        gzerr << "Planar stabilizer could not find joint ["
              << joint_name << "].\n";
      }
      joint_element = joint_element->GetNextElement("hold_joint");
    }
  }

  // ODE's auto-disable freezes a rigid body that has been at rest for a
  // short time.  A frozen body ignores forces applied through its joints --
  // only direct contact or an explicit enable wakes it -- so an end-effector
  // slider that falls asleep between operations pins at its last coordinate
  // and silently refuses every commanded effort (diagnosed as the right tool
  // rods staying put while a sibling stage on the same controller tracked
  // perfectly).  The tool sliders are driven exclusively by the
  // gazebo_ros2_control effort path, so the small prismatic carriage bodies
  // must never be allowed to auto-disable.  Exempting them from the sleep
  // heuristic costs nothing: they carry negligible mass and are no heavier
  // for the solver than an always-awake body.
  void keep_slider_links_awake()
  {
    for (const gazebo::physics::JointPtr & joint : model_->GetJoints()) {
      if ((joint->GetType() & gazebo::physics::Base::SLIDER_JOINT) == 0U) {
        continue;
      }
      for (const int index : {0, 1}) {
        gazebo::physics::LinkPtr link = joint->GetJointLink(index);
        if (link) {
          link->SetAutoDisable(false);
          // SetAutoDisable only clears the flag; a body that is already
          // frozen (e.g. after a world reset mid-idle) needs an explicit
          // enable before it will accept forces again.
          link->SetEnabled(true);
        }
      }
    }
  }

  void schedule_height_lock()
  {
    height_lock_sim_time_ =
      model_->GetWorld()->SimTime().Double() + settling_duration_;
  }

  void capture_joint_positions(const gazebo::common::UpdateInfo &)
  {
    // The planar-move plugin is declared before this plugin and writes its
    // requested model velocity at WorldUpdateBegin.  Sampling here closes the
    // one-message race between their independent ROS subscriptions: a new
    // motion command always releases the brake before WorldUpdateEnd, while a
    // stop command is not latched until the mover has actually written zero.
    constexpr double command_epsilon = 1.0e-6;
    const ignition::math::Vector3d linear_velocity =
      model_->WorldLinearVel();
    const ignition::math::Vector3d angular_velocity =
      model_->WorldAngularVel();
    planar_motion_observed_at_begin_ =
      !linear_velocity.IsFinite() ||
      !angular_velocity.IsFinite() ||
      std::abs(linear_velocity.X()) > command_epsilon ||
      std::abs(linear_velocity.Y()) > command_epsilon ||
      std::abs(angular_velocity.Z()) > command_epsilon;

    for (std::size_t index = 0; index < held_joints_.size(); ++index) {
      held_joint_positions_[index] = held_joints_[index]->Position(0);
    }
    has_joint_snapshot_ = true;
  }

  void restore_joint_positions()
  {
    if (!has_joint_snapshot_) {
      return;
    }

    // ``Joint::SetPosition`` reaches ODE's anchor update path even when the
    // requested coordinate is already the current coordinate.  ODE does not
    // implement that path for slider joints, so unconditionally reapplying a
    // held prismatic tool coordinate produced one warning per tool joint on
    // every physics tick (and eventually multi-gigabyte Gazebo logs).  A
    // snapshot is only a correction target: preserve the existing behavior
    // when a joint actually drifts, but do not submit a no-op write to the
    // physics backend.
    constexpr double position_restore_tolerance = 1.0e-6;
    for (std::size_t index = 0; index < held_joints_.size(); ++index) {
      const double current_position = held_joints_[index]->Position(0);
      if (std::isfinite(current_position) &&
        std::abs(current_position - held_joint_positions_[index]) >
        position_restore_tolerance)
      {
        held_joints_[index]->SetPosition(
          0, held_joint_positions_[index], true);
      }
      held_joints_[index]->SetVelocity(0, 0.0);
    }
  }

  void stabilize()
  {
    // Gazebo's solver naturally leaves sub-micrometre / sub-microradian
    // noise after a physics update.  Treating 1e-9 as an error made this
    // plugin call Model::SetWorldPose on virtually every 500 Hz update.  That
    // operation recursively touches slider children and ODE logs an
    // unsupported anchor update for each prismatic tool joint.  Ten microns
    // (and ten microradians for attitude) remains far below the robot's
    // mechanical/control tolerances while avoiding those no-op pose writes.
    constexpr double tilt_tolerance = 1.0e-5;
    constexpr double height_tolerance = 1.0e-5;
    constexpr double planar_tolerance = 1.0e-5;
    constexpr double two_pi = 6.28318530717958647692;

    const ignition::math::Pose3d world_pose = model_->WorldPose();
    const double roll = world_pose.Rot().Roll();
    const double pitch = world_pose.Rot().Pitch();
    const double sim_time = model_->GetWorld()->SimTime().Double();

    if (!height_is_locked_ && sim_time >= height_lock_sim_time_) {
      locked_height_ = world_pose.Pos().Z();
      height_is_locked_ = true;
    }

    const bool hold_planar_pose =
      !base_command_moving_->load() &&
      !planar_motion_observed_at_begin_;
    if (hold_planar_pose && !planar_pose_is_locked_) {
      locked_planar_x_ = world_pose.Pos().X();
      locked_planar_y_ = world_pose.Pos().Y();
      locked_planar_yaw_ = world_pose.Rot().Yaw();
      planar_pose_is_locked_ = true;
    } else if (!hold_planar_pose) {
      planar_pose_is_locked_ = false;
    }

    // A teleport (scene switch through SetEntityState) relocates the chassis
    // far beyond the drift the velocity-zeroing lock can produce.  Accept the
    // new pose as the lock target instead of snapping back through the world:
    // the lock's job is to hold the chassis where it last came to rest, and a
    // teleport is a legitimate new rest point.  Height is deliberately left
    // alone so the chassis still settles onto the ground under gravity: the
    // stale height lock is cleared and re-armed after the settling window
    // (the same idiom Reset uses for a re-spawn) instead of snapping Z back to
    // the previous rest height on every tick.  A pure Z relocation (e.g. a
    // lift that leaves the chassis over different ground) is detected as a
    // teleport too.
    constexpr double teleport_planar_tolerance = 0.1;
    if (planar_pose_is_locked_ &&
      (std::abs(world_pose.Pos().X() - locked_planar_x_) >
         teleport_planar_tolerance ||
       std::abs(world_pose.Pos().Y() - locked_planar_y_) >
         teleport_planar_tolerance ||
       std::abs(
         std::remainder(
           world_pose.Rot().Yaw() - locked_planar_yaw_,
           two_pi)) > teleport_planar_tolerance ||
       (height_is_locked_ &&
        std::abs(world_pose.Pos().Z() - locked_height_) >
          teleport_planar_tolerance)))
    {
      locked_planar_x_ = world_pose.Pos().X();
      locked_planar_y_ = world_pose.Pos().Y();
      locked_planar_yaw_ = world_pose.Rot().Yaw();
      height_is_locked_ = false;
      schedule_height_lock();
    }

    const bool has_tilt =
      std::abs(roll) > tilt_tolerance ||
      std::abs(pitch) > tilt_tolerance;
    const bool has_height_error =
      height_is_locked_ &&
      std::abs(world_pose.Pos().Z() - locked_height_) >
      height_tolerance;
    const bool has_planar_error =
      planar_pose_is_locked_ && (
      std::abs(world_pose.Pos().X() - locked_planar_x_) > planar_tolerance ||
      std::abs(world_pose.Pos().Y() - locked_planar_y_) > planar_tolerance ||
      std::abs(
        std::remainder(
          world_pose.Rot().Yaw() - locked_planar_yaw_,
          two_pi)) > planar_tolerance);

    if (has_tilt || has_height_error || has_planar_error) {
      ignition::math::Vector3d planar_position = world_pose.Pos();
      if (height_is_locked_) {
        planar_position.Z(locked_height_);
      }
      if (planar_pose_is_locked_) {
        planar_position.X(locked_planar_x_);
        planar_position.Y(locked_planar_y_);
      }
      const ignition::math::Quaterniond planar_rotation(
        0.0, 0.0,
        planar_pose_is_locked_ ?
        locked_planar_yaw_ : world_pose.Rot().Yaw());
      model_->SetWorldPose(
        ignition::math::Pose3d(planar_position, planar_rotation),
        true,
        false);
    }

    if (height_is_locked_ || planar_pose_is_locked_) {
      ignition::math::Vector3d linear_velocity =
        model_->WorldLinearVel();
      if (height_is_locked_) {
        linear_velocity.Z(0.0);
      }
      if (planar_pose_is_locked_) {
        linear_velocity.X(0.0);
        linear_velocity.Y(0.0);
      }
      model_->SetLinearVel(linear_velocity);
    }

    ignition::math::Vector3d angular_velocity =
      model_->WorldAngularVel();
    angular_velocity.X(0.0);
    angular_velocity.Y(0.0);
    if (planar_pose_is_locked_) {
      angular_velocity.Z(0.0);
    }
    model_->SetAngularVel(angular_velocity);

    // Protect the spawn pose only until verify_initial_pose confirms it and
    // publishes joint_hold_enabled=false.  Keeping this correction enabled
    // later races ros2_control at every physics step and cancels ordinary
    // arm/tool trajectories; during a cabinet grasp it is likewise released.
    if (joint_hold_enabled_->load() && !grasp_active_->load()) {
      restore_joint_positions();
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    grasp_active_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    joint_hold_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_subscription_;
  std::shared_ptr<std::atomic<bool>> grasp_active_{
    std::make_shared<std::atomic<bool>>(false)};
  std::shared_ptr<std::atomic<bool>> joint_hold_enabled_{
    std::make_shared<std::atomic<bool>>(true)};
  std::shared_ptr<std::atomic<bool>> base_command_moving_{
    std::make_shared<std::atomic<bool>>(false)};
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::event::ConnectionPtr joint_snapshot_connection_;
  std::shared_ptr<UpdateCallbackLifetime> callback_lifetime_;
  std::mutex update_mutex_;
  std::vector<gazebo::physics::JointPtr> held_joints_;
  std::vector<double> held_joint_positions_;
  double settling_duration_{0.5};
  double height_lock_sim_time_{0.0};
  double locked_height_{0.0};
  double locked_planar_x_{0.0};
  double locked_planar_y_{0.0};
  double locked_planar_yaw_{0.0};
  bool height_is_locked_{false};
  bool planar_pose_is_locked_{false};
  bool planar_motion_observed_at_begin_{false};
  bool has_joint_snapshot_{false};
};

GZ_REGISTER_MODEL_PLUGIN(PlanarStabilizerPlugin)

}  // namespace xczs_inspection_robot_control
