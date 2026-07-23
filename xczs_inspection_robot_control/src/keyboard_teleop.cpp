// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "xczs_inspection_robot_control/jerk_limited_velocity_profile.hpp"

namespace
{

using SteadyClock = std::chrono::steady_clock;
using xczs_inspection_robot_control::JerkLimitedVelocityProfile;

enum class BaseMotion
{
  kForward,
  kBackward,
  kTurnLeft,
  kTurnRight,
};

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

constexpr char kHelpText[] =
  R"(
XCZS 巡操机器人键盘遥控

底盘：
  W / S       前进 / 后退
  A / D       左转 / 右转
  X           平滑停止
  空格        紧急停止

六轴机械臂：
  1 ... 6     选择对应关节
  [ / ]       减小 / 增大所选关节角度
  - / =       减小 / 增大所选关节角度
  R           六个关节回到零位

夹爪：
  O / P       打开 / 关闭

其他：
  H 或 ?      显示帮助
  Q 或 Ctrl-C 停止底盘并退出
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
    linear_acceleration_ = positive_parameter("linear_acceleration", 0.50);
    angular_acceleration_ = positive_parameter("angular_acceleration", 1.20);
    linear_jerk_ = positive_parameter("linear_jerk", 2.00);
    angular_jerk_ = positive_parameter("angular_jerk", 4.80);
    base_command_rate_ = positive_parameter("base_command_rate", 50.0);
    joint_step_ = positive_parameter("joint_step", 0.10);
    joint_limit_ = positive_parameter("joint_limit", 2.80);
    gripper_open_angle_ = positive_parameter("gripper_open_angle", 0.35);
    joint_command_rate_ = positive_parameter("joint_command_rate", 20.0);
    command_timeout_ = positive_parameter("command_timeout", 0.75);

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

    linear_velocity_profile_.configure(
      linear_acceleration_,
      linear_jerk_);
    angular_velocity_profile_.configure(
      angular_acceleration_,
      angular_jerk_);
    base_publish_period_ = 1.0 / base_command_rate_;
    joint_publish_period_ = 1.0 / joint_command_rate_;
    last_base_key_time_ = SteadyClock::now();
    last_base_update_time_ = SteadyClock::now();
    last_joint_publish_time_ = SteadyClock::now();

    RCLCPP_INFO(
      get_logger(), "Publishing base commands on %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(
      get_logger(), "Publishing joint commands on %s",
      trajectory_topic.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Base velocity uses a %.1f Hz jerk-limited S-curve profile.",
      base_command_rate_);
  }

  bool handle_key(char key)
  {
    const char normalized_key = static_cast<char>(
      std::tolower(static_cast<unsigned char>(key)));

    if (key == '\x03' || normalized_key == 'q') {
      return false;
    }

    if (key >= '1' && key <= '6') {
      select_arm_joint(static_cast<std::size_t>(key - '1'));
      return true;
    }

    switch (normalized_key) {
      case 'w':
        start_base_motion(BaseMotion::kForward);
        break;
      case 's':
        start_base_motion(BaseMotion::kBackward);
        break;
      case 'a':
        start_base_motion(BaseMotion::kTurnLeft);
        break;
      case 'd':
        start_base_motion(BaseMotion::kTurnRight);
        break;
      case 'x':
        request_smooth_stop();
        write_status("底盘：正在平滑停止");
        break;
      case ' ':
        stop_base();
        write_status("底盘：已紧急停止");
        break;
      case '[':
      case '-':
        move_selected_joint(-joint_step_);
        break;
      case ']':
      case '=':
        move_selected_joint(joint_step_);
        break;
      case 'r':
        reset_arm();
        break;
      case 'o':
        set_gripper(true);
        break;
      case 'p':
        set_gripper(false);
        break;
      case 'h':
      case '?':
        print_help();
        break;
      default:
        break;
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

    if (base_motion_requested_) {
      const double seconds_since_base_key =
        std::chrono::duration<double>(
        current_time - last_base_key_time_).count();
      if (seconds_since_base_key >= command_timeout_) {
        target_base_command_ = geometry_msgs::msg::Twist{};
        base_motion_requested_ = false;
        write_status("底盘：安全超时，正在减速停止");
      }
    }

    const double seconds_since_base_update =
      std::chrono::duration<double>(
      current_time - last_base_update_time_).count();
    if (seconds_since_base_update < base_publish_period_) {
      return;
    }

    const double update_period = std::clamp(
      seconds_since_base_update,
      0.0,
      2.0 * base_publish_period_);
    last_base_update_time_ = current_time;

    linear_velocity_profile_.set_target(target_base_command_.linear.y);
    angular_velocity_profile_.set_target(target_base_command_.angular.z);
    base_command_.linear.y =
      linear_velocity_profile_.update(update_period);
    base_command_.angular.z =
      angular_velocity_profile_.update(update_period);
    cmd_vel_publisher_->publish(base_command_);
  }

  void stop_base()
  {
    base_command_ = geometry_msgs::msg::Twist{};
    target_base_command_ = geometry_msgs::msg::Twist{};
    linear_velocity_profile_.reset();
    angular_velocity_profile_.reset();
    base_motion_requested_ = false;
    cmd_vel_publisher_->publish(base_command_);
  }

  void request_smooth_stop()
  {
    target_base_command_ = geometry_msgs::msg::Twist{};
    base_motion_requested_ = false;
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
        joint_state_received_[target_index] = true;
      }
    }

    if (
      !joint_targets_initialized_ &&
      std::all_of(
        joint_state_received_.begin(),
        joint_state_received_.end(),
        [](bool received) {return received;}))
    {
      joint_targets_ = measured_joint_positions_;
      joint_targets_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Joint targets initialized from /xczs/joint_states.");
      write_status("关节状态已同步，可以控制机械臂和夹爪");
    }
  }

  void start_base_motion(BaseMotion motion)
  {
    target_base_command_ = geometry_msgs::msg::Twist{};

    std::string status;
    switch (motion) {
      case BaseMotion::kForward:
        // The imported model's visual front points along body +Y.
        target_base_command_.linear.y = linear_speed_;
        status = "底盘：前进";
        break;
      case BaseMotion::kBackward:
        target_base_command_.linear.y = -linear_speed_;
        status = "底盘：后退";
        break;
      case BaseMotion::kTurnLeft:
        target_base_command_.angular.z = angular_speed_;
        status = "底盘：左转";
        break;
      case BaseMotion::kTurnRight:
        target_base_command_.angular.z = -angular_speed_;
        status = "底盘：右转";
        break;
    }

    last_base_key_time_ = SteadyClock::now();
    base_motion_requested_ = true;
    write_status(status);
  }

  void select_arm_joint(std::size_t joint_index)
  {
    selected_joint_index_ = joint_index;
    std::string status =
      "机械臂：已选择关节 " + std::to_string(joint_index + 1) +
      "（" + kArmJoints[joint_index] + "）";
    if (joint_targets_initialized_) {
      status += "，目标角度 " + format_angle(joint_targets_[joint_index]);
    } else {
      status += "，正在等待关节状态";
    }
    write_status(status);
  }

  bool joint_control_ready() const
  {
    if (joint_targets_initialized_) {
      return true;
    }
    write_status("关节状态尚未就绪，请稍后重试");
    return false;
  }

  void move_selected_joint(double increment)
  {
    if (!joint_control_ready()) {
      return;
    }

    const auto target = std::clamp(
      joint_targets_[selected_joint_index_] + increment,
      -joint_limit_,
      joint_limit_);
    joint_targets_[selected_joint_index_] = target;
    schedule_joint_trajectory();

    write_status(
      "机械臂：关节 " + std::to_string(selected_joint_index_ + 1) +
      "（" + kArmJoints[selected_joint_index_] + "）目标角度 " +
      format_angle(target));
  }

  void reset_arm()
  {
    if (!joint_control_ready()) {
      return;
    }

    for (std::size_t index = 0; index < kArmJoints.size(); ++index) {
      joint_targets_[index] = 0.0;
    }
    schedule_joint_trajectory();
    write_status("机械臂：六个关节已回到零位");
  }

  void set_gripper(bool open)
  {
    if (!joint_control_ready()) {
      return;
    }

    const double target = open ? gripper_open_angle_ : 0.0;
    joint_targets_[6] = target;
    joint_targets_[7] = -target;
    schedule_joint_trajectory();
    write_status(open ? "夹爪：已打开" : "夹爪：已关闭");
  }

  void schedule_joint_trajectory()
  {
    pending_joint_publications_ = joint_command_repeats_;
    publish_joint_trajectory();
  }

  void publish_joint_trajectory()
  {
    if (!joint_targets_initialized_) {
      pending_joint_publications_ = 0;
      return;
    }

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

  static std::string format_angle(double angle)
  {
    if (std::abs(angle) < 0.005) {
      angle = 0.0;
    }

    std::ostringstream stream;
    stream << std::showpos << std::fixed << std::setprecision(2)
           << angle << " rad";
    return stream.str();
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;

  std::array<double, 8> joint_targets_{};
  std::array<double, 8> measured_joint_positions_{};
  std::array<bool, 8> joint_state_received_{};
  std::size_t selected_joint_index_{0};

  geometry_msgs::msg::Twist base_command_;
  geometry_msgs::msg::Twist target_base_command_;
  JerkLimitedVelocityProfile linear_velocity_profile_;
  JerkLimitedVelocityProfile angular_velocity_profile_;
  SteadyClock::time_point last_base_key_time_;
  SteadyClock::time_point last_base_update_time_;
  SteadyClock::time_point last_joint_publish_time_;

  double linear_speed_{0.25};
  double angular_speed_{0.60};
  double linear_acceleration_{0.50};
  double angular_acceleration_{1.20};
  double linear_jerk_{2.00};
  double angular_jerk_{4.80};
  double base_command_rate_{50.0};
  double joint_step_{0.10};
  double joint_limit_{2.80};
  double gripper_open_angle_{0.35};
  double joint_command_rate_{20.0};
  double command_timeout_{0.75};
  double base_publish_period_{0.02};
  double joint_publish_period_{0.05};

  int joint_command_repeats_{5};
  int pending_joint_publications_{0};
  bool base_motion_requested_{false};
  bool joint_targets_initialized_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<KeyboardTeleop> node;
  int exit_code = 0;

  try {
    node = std::make_shared<KeyboardTeleop>();
    TerminalGuard terminal_guard;
    node->print_help();

    bool keep_running = true;
    while (rclcpp::ok() && keep_running) {
      rclcpp::spin_some(node);

      char key = '\0';
      if (read_key(key, std::chrono::milliseconds(10))) {
        keep_running = node->handle_key(key);
      }
      node->update();
    }
  } catch (const std::exception & exception) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
    } else {
      std::cerr << "Failed to start keyboard control: "
                << exception.what() << '\n';
    }
    exit_code = 1;
  }

  if (node && rclcpp::ok()) {
    node->stop_base();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    node->stop_base();
  }

  std::cout << "\r\n键盘遥控已关闭。\r\n";
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
