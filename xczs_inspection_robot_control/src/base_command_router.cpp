// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "xczs_inspection_robot_control/router_utils.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using Twist = geometry_msgs::msg::Twist;
using SteadyClock = std::chrono::steady_clock;

bool is_finite_command(const Twist & command)
{
  return
    std::isfinite(command.linear.x) &&
    std::isfinite(command.linear.y) &&
    std::isfinite(command.angular.z);
}

bool is_moving(const Twist & command)
{
  constexpr double kCommandEpsilon = 1.0e-6;
  return
    std::abs(command.linear.x) > kCommandEpsilon ||
    std::abs(command.linear.y) > kCommandEpsilon ||
    std::abs(command.angular.z) > kCommandEpsilon;
}

}  // namespace

class BaseCommandRouter final : public rclcpp::Node
{
public:
  BaseCommandRouter()
  : Node("xczs_base_command_router")
  {
    const auto manual_topic = absolute_ros_name_parameter(
      "manual_cmd_vel_topic", "/xczs/manual_cmd_vel", false);
    const auto navigation_topic = absolute_ros_name_parameter(
      "navigation_cmd_vel_topic", "/cmd_vel", false);
    const auto output_topic = absolute_ros_name_parameter(
      "base_output_topic", "/xczs/cmd_vel", false);
    const auto navigation_mode_topic = absolute_ros_name_parameter(
      "navigation_mode_topic", "/xczs/navigation_mode", false);
    const auto navigation_mode_service = absolute_ros_name_parameter(
      "navigation_mode_service", "/xczs/set_navigation_mode", true);
    navigation_velocity_yaw_offset_ = finite_parameter(
      "navigation_velocity_yaw_offset", 1.57079632679);
    navigation_enabled_ = declare_parameter<bool>(
      "navigation_enabled", false);
    command_timeout_ = positive_parameter("command_timeout", 0.5);
    publish_rate_ = positive_parameter("publish_rate", 50.0);
    max_linear_speed_ = positive_parameter("max_linear_speed", 0.5);
    max_angular_speed_ = positive_parameter("max_angular_speed", 1.2);

    output_publisher_ = create_publisher<Twist>(output_topic, 10);
    navigation_mode_publisher_ = create_publisher<std_msgs::msg::Bool>(
      navigation_mode_topic,
      rclcpp::QoS(1).reliable().transient_local());
    manual_subscription_ = create_subscription<Twist>(
      manual_topic,
      10,
      [this](const Twist::SharedPtr message) {
        receive_manual_command(*message);
      });
    navigation_subscription_ = create_subscription<Twist>(
      navigation_topic,
      10,
      [this](const Twist::SharedPtr message) {
        receive_navigation_command(*message);
      });
    mode_service_ = create_service<std_srvs::srv::SetBool>(
      navigation_mode_service,
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        set_navigation_mode(request->data, *response);
      });

    const auto timer_period = std::chrono::duration<double>(
      1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      [this]() {
        publish_active_command();
      });

    publish_stop();
    publish_navigation_mode();
    RCLCPP_INFO(
      get_logger(),
      "Base command router started in %s mode. Output: %s",
      navigation_enabled_ ? "navigation" : "manual",
      output_topic.c_str());
  }

private:
  std::string absolute_ros_name_parameter(
    const std::string & name,
    const std::string & default_value,
    bool is_service)
  {
    const auto value = declare_parameter<std::string>(name, default_value);
    if (value.empty() || value.front() != '/') {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be an absolute ROS name.");
    }
    try {
      (void)rclcpp::expand_topic_or_service_name(
        value, get_name(), get_namespace(), is_service);
    } catch (const std::exception & error) {
      throw std::invalid_argument(
              "Parameter '" + name +
              "' must be a valid absolute ROS name: " + error.what());
    }
    return value;
  }

  double finite_parameter(
    const std::string & name,
    double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be finite.");
    }
    return value;
  }

  double positive_parameter(
    const std::string & name,
    double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be positive.");
    }
    return value;
  }

  Twist sanitize_command(const Twist & input) const
  {
    Twist output;
    output.linear.x = std::clamp(
      input.linear.x, -max_linear_speed_, max_linear_speed_);
    output.linear.y = std::clamp(
      input.linear.y, -max_linear_speed_, max_linear_speed_);
    output.angular.z = std::clamp(
      input.angular.z, -max_angular_speed_, max_angular_speed_);
    return output;
  }

  Twist navigation_to_model_frame(const Twist & navigation_command) const
  {
    const Twist sanitized = sanitize_command(navigation_command);
    const auto rotated = rotate_planar_vector(
      {sanitized.linear.x, sanitized.linear.y},
      navigation_velocity_yaw_offset_);
    Twist output;
    output.linear.x = rotated.x;
    output.linear.y = rotated.y;
    output.angular.z = sanitized.angular.z;
    return sanitize_command(output);
  }

  void receive_manual_command(const Twist & command)
  {
    if (!is_finite_command(command)) {
      RCLCPP_ERROR(get_logger(), "Rejected a non-finite manual base command.");
      return;
    }

    manual_command_ = sanitize_command(command);
    manual_command_time_ = SteadyClock::now();
    has_manual_command_ = true;
    if (navigation_enabled_ && is_moving(command)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring manual base motion while navigation mode is enabled.");
    }
  }

  void receive_navigation_command(const Twist & command)
  {
    if (!is_finite_command(command)) {
      RCLCPP_ERROR(
        get_logger(), "Rejected a non-finite Nav2 base command.");
      return;
    }

    navigation_command_ = navigation_to_model_frame(command);
    navigation_command_time_ = SteadyClock::now();
    has_navigation_command_ = true;
    if (!navigation_enabled_ && is_moving(command)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring Nav2 base motion while manual mode is enabled.");
    }
  }

  void set_navigation_mode(
    bool navigation_enabled,
    std_srvs::srv::SetBool::Response & response)
  {
    navigation_enabled_ = navigation_enabled;
    has_manual_command_ = false;
    has_navigation_command_ = false;
    publish_stop();
    publish_navigation_mode();

    response.success = true;
    response.message = navigation_enabled_ ?
      "Navigation mode enabled; manual base commands are blocked." :
      "Manual mode enabled; Nav2 base commands are blocked.";
    RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
  }

  bool is_fresh(
    bool has_command,
    const SteadyClock::time_point & command_time) const
  {
    if (!has_command) {
      return false;
    }
    const double age = std::chrono::duration<double>(
      SteadyClock::now() - command_time).count();
    return age <= command_timeout_;
  }

  void publish_active_command()
  {
    const bool command_is_fresh = navigation_enabled_ ?
      is_fresh(has_navigation_command_, navigation_command_time_) :
      is_fresh(has_manual_command_, manual_command_time_);
    if (!command_is_fresh) {
      if (!stop_is_published_) {
        publish_stop();
      }
      return;
    }

    const auto & command = navigation_enabled_ ?
      navigation_command_ :
      manual_command_;
    output_publisher_->publish(command);
    stop_is_published_ = !is_moving(command);
  }

  void publish_stop()
  {
    output_publisher_->publish(Twist{});
    stop_is_published_ = true;
  }

  void publish_navigation_mode()
  {
    std_msgs::msg::Bool message;
    message.data = navigation_enabled_;
    navigation_mode_publisher_->publish(message);
  }

  rclcpp::Publisher<Twist>::SharedPtr output_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    navigation_mode_publisher_;
  rclcpp::Subscription<Twist>::SharedPtr manual_subscription_;
  rclcpp::Subscription<Twist>::SharedPtr navigation_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  Twist manual_command_;
  Twist navigation_command_;
  SteadyClock::time_point manual_command_time_;
  SteadyClock::time_point navigation_command_time_;
  double command_timeout_{0.5};
  double publish_rate_{50.0};
  double max_linear_speed_{0.5};
  double max_angular_speed_{1.2};
  double navigation_velocity_yaw_offset_{1.57079632679};
  bool navigation_enabled_{false};
  bool has_manual_command_{false};
  bool has_navigation_command_{false};
  bool stop_is_published_{false};
};

}  // namespace xczs_inspection_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      xczs_inspection_robot_control::BaseCommandRouter>());
  rclcpp::shutdown();
  return 0;
}
