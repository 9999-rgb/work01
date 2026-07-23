// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

using SteadyClock = std::chrono::steady_clock;

const std::array<std::string, 6> kArmJoints = {
  "body_arm1",
  "arm1_arm2",
  "arm2_arm3",
  "arm3_arm4",
  "arm4_arm5",
  "arm5_end",
};

const std::array<std::string, 8> kControlledJoints = {
  "body_arm1",
  "arm1_arm2",
  "arm2_arm3",
  "arm3_arm4",
  "arm4_arm5",
  "arm5_end",
  "end_worklink1",
  "end_worklink2",
};

constexpr char kHelpText[] = R"(
XCZS inspection robot keyboard control

Base:
  W / S       move forward / backward
  A / D       turn left / right
  X or SPACE  stop the base

Arm:
  1 ... 6     select an arm joint
  [ / ]       decrease / increase the selected joint angle
  R           reset all arm joints to zero

Gripper:
  O / P       open / close the gripper

Other:
  H or ?      show this help
  Q or Ctrl-C stop and quit
)";

class TerminalGuard
{
public:
  TerminalGuard()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("Keyboard input requires an interactive terminal.");
    }

    if (tcgetattr(STDIN_FILENO, &original_settings_) != 0) {
      throw std::runtime_error("Failed to read terminal settings.");
    }

    termios raw_settings = original_settings_;
    cfmakeraw(&raw_settings);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_settings) != 0) {
      throw std::runtime_error("Failed to enable raw terminal mode.");
    }
    active_ = true;
  }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard & operator=(const TerminalGuard &) = delete;

  ~TerminalGuard()
  {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &original_settings_);
    }
  }

private:
  termios original_settings_{};
  bool active_{false};
};

bool read_key(char & key, std::chrono::milliseconds timeout)
{
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(STDIN_FILENO, &read_set);

  timeval timeout_value{};
  timeout_value.tv_sec = static_cast<decltype(timeout_value.tv_sec)>(
    timeout.count() / 1000);
  timeout_value.tv_usec = static_cast<decltype(timeout_value.tv_usec)>(
    (timeout.count() % 1000) * 1000);

  const int result = select(
    STDIN_FILENO + 1,
    &read_set,
    nullptr,
    nullptr,
    &timeout_value);

  if (result < 0) {
    if (errno == EINTR) {
      return false;
    }
    throw std::runtime_error("Failed while waiting for keyboard input.");
  }

  if (result == 0) {
    return false;
  }

  return ::read(STDIN_FILENO, &key, 1) == 1;
}

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("xczs_keyboard_teleop")
  {
    linear_speed_ = positive_parameter("linear_speed", 0.25);
    angular_speed_ = positive_parameter("angular_speed", 0.60);
    joint_step_ = positive_parameter("joint_step", 0.10);
    joint_limit_ = positive_parameter("joint_limit", 2.80);
    gripper_open_angle_ = positive_parameter("gripper_open_angle", 0.35);
    joint_command_rate_ = positive_parameter("joint_command_rate", 20.0);
    command_timeout_ = positive_parameter("command_timeout", 0.50);

    joint_command_repeats_ = declare_parameter<int>("joint_command_repeats", 5);
    if (joint_command_repeats_ <= 0) {
      throw std::invalid_argument(
              "Parameter 'joint_command_repeats' must be positive.");
    }

    const auto cmd_vel_topic = declare_parameter<std::string>(
      "cmd_vel_topic", "/xczs/cmd_vel");
    const auto trajectory_topic = declare_parameter<std::string>(
      "joint_trajectory_topic", "/xczs/joint_trajectory");
    const auto joint_state_topic = declare_parameter<std::string>(
      "joint_state_topic", "/xczs/joint_states");

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10);
    trajectory_publisher_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic, 10);
    joint_state_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic,
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        joint_state_callback(*message);
      });

    joint_publish_period_ = 1.0 / joint_command_rate_;
    last_base_key_time_ = SteadyClock::now();
    last_joint_publish_time_ = SteadyClock::now();
    pending_joint_publications_ = joint_command_repeats_;

    RCLCPP_INFO(
      get_logger(), "Publishing base commands on %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(
      get_logger(), "Publishing joint commands on %s",
      trajectory_topic.c_str());
  }

  bool handle_key(char key)
  {
    const char normalized_key = static_cast<char>(
      std::tolower(static_cast<unsigned char>(key)));

    if (key == '\x03' || normalized_key == 'q') {
      return false;
    }

    if (normalized_key == 'w') {
      set_base_command(linear_speed_, 0.0);
      write_status("Base: forward");
    } else if (normalized_key == 's') {
      set_base_command(-linear_speed_, 0.0);
      write_status("Base: backward");
    } else if (normalized_key == 'a') {
      set_base_command(0.0, angular_speed_);
      write_status("Base: turn left");
    } else if (normalized_key == 'd') {
      set_base_command(0.0, -angular_speed_);
      write_status("Base: turn right");
    } else if (normalized_key == 'x' || key == ' ') {
      stop_base();
      write_status("Base: stopped");
    } else if (key >= '1' && key <= '6') {
      selected_joint_index_ = static_cast<std::size_t>(key - '1');
      write_status(
        "Selected joint " + std::string(1, key) + ": " +
        kArmJoints[selected_joint_index_]);
    } else if (key == '[') {
      move_selected_joint(-joint_step_);
    } else if (key == ']') {
      move_selected_joint(joint_step_);
    } else if (normalized_key == 'r') {
      for (std::size_t index = 0; index < kArmJoints.size(); ++index) {
        joint_targets_[index] = 0.0;
      }
      schedule_joint_trajectory();
      write_status("Arm: reset to zero");
    } else if (normalized_key == 'o') {
      joint_targets_[6] = gripper_open_angle_;
      joint_targets_[7] = -gripper_open_angle_;
      schedule_joint_trajectory();
      write_status("Gripper: open");
    } else if (normalized_key == 'p') {
      joint_targets_[6] = 0.0;
      joint_targets_[7] = 0.0;
      schedule_joint_trajectory();
      write_status("Gripper: closed");
    } else if (normalized_key == 'h' || key == '?') {
      print_help();
    }

    return true;
  }

  void update()
  {
    const auto current_time = SteadyClock::now();
    const double seconds_since_joint_publish =
      std::chrono::duration<double>(
      current_time - last_joint_publish_time_).count();

    if (
      pending_joint_publications_ > 0 &&
      seconds_since_joint_publish >= joint_publish_period_)
    {
      publish_joint_trajectory();
    }

    if (!base_is_moving_) {
      return;
    }

    const double seconds_since_base_key =
      std::chrono::duration<double>(
      current_time - last_base_key_time_).count();
    if (seconds_since_base_key >= command_timeout_) {
      stop_base();
      write_status("Base: auto-stopped");
    } else {
      cmd_vel_publisher_->publish(base_command_);
    }
  }

  void stop_base()
  {
    base_command_ = geometry_msgs::msg::Twist{};
    base_is_moving_ = false;
    cmd_vel_publisher_->publish(base_command_);
  }

  void print_help() const
  {
    std::cout << "\r\n" << kHelpText << "\r\n" << std::flush;
  }

private:
  double positive_parameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be positive.");
    }
    return value;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState & message)
  {
    const std::size_t value_count =
      std::min(message.name.size(), message.position.size());
    for (std::size_t index = 0; index < value_count; ++index) {
      const auto joint_iterator = std::find(
        kControlledJoints.begin(),
        kControlledJoints.end(),
        message.name[index]);
      if (
        joint_iterator != kControlledJoints.end() &&
        std::isfinite(message.position[index]))
      {
        const auto target_index = static_cast<std::size_t>(
          std::distance(kControlledJoints.begin(), joint_iterator));
        measured_joint_positions_[target_index] = message.position[index];
      }
    }
  }

  void set_base_command(double linear_velocity, double angular_velocity)
  {
    base_command_ = geometry_msgs::msg::Twist{};
    base_command_.linear.x = linear_velocity;
    base_command_.angular.z = angular_velocity;
    last_base_key_time_ = SteadyClock::now();
    base_is_moving_ = true;
    cmd_vel_publisher_->publish(base_command_);
  }

  void move_selected_joint(double increment)
  {
    const auto target = std::clamp(
      joint_targets_[selected_joint_index_] + increment,
      -joint_limit_,
      joint_limit_);
    joint_targets_[selected_joint_index_] = target;
    schedule_joint_trajectory();

    const std::string sign = target >= 0.0 ? "+" : "";
    write_status(
      kArmJoints[selected_joint_index_] + ": " + sign +
      std::to_string(target) + " rad");
  }

  void schedule_joint_trajectory()
  {
    pending_joint_publications_ = joint_command_repeats_;
    publish_joint_trajectory();
  }

  void publish_joint_trajectory()
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.frame_id = "world";
    trajectory.joint_names.assign(
      kControlledJoints.begin(), kControlledJoints.end());

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(joint_targets_.begin(), joint_targets_.end());
    trajectory.points.push_back(std::move(point));

    trajectory_publisher_->publish(trajectory);
    pending_joint_publications_ =
      std::max(0, pending_joint_publications_ - 1);
    last_joint_publish_time_ = SteadyClock::now();
  }

  static void write_status(const std::string & message)
  {
    std::cout << "\r\033[2K" << message << std::flush;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;

  std::array<double, 8> joint_targets_{};
  std::array<double, 8> measured_joint_positions_{};
  std::size_t selected_joint_index_{0};

  geometry_msgs::msg::Twist base_command_;
  SteadyClock::time_point last_base_key_time_;
  SteadyClock::time_point last_joint_publish_time_;

  double linear_speed_{0.25};
  double angular_speed_{0.60};
  double joint_step_{0.10};
  double joint_limit_{2.80};
  double gripper_open_angle_{0.35};
  double joint_command_rate_{20.0};
  double command_timeout_{0.50};
  double joint_publish_period_{0.05};

  int joint_command_repeats_{5};
  int pending_joint_publications_{0};
  bool base_is_moving_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  int exit_code = 0;

  try {
    {
      TerminalGuard terminal_guard;
      node->print_help();

      bool keep_running = true;
      while (rclcpp::ok() && keep_running) {
        rclcpp::spin_some(node);

        char key = '\0';
        if (read_key(key, std::chrono::milliseconds(50))) {
          keep_running = node->handle_key(key);
        }
        node->update();
      }
    }

    node->stop_base();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    node->stop_base();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
    exit_code = 1;
  }

  std::cout << "\r\nKeyboard control stopped.\r\n";
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
