// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <QApplication>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
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

class InspectionRobotWindow : public QMainWindow
{
public:
  explicit InspectionRobotWindow(
    const std::shared_ptr<rclcpp::Node> & node)
  : node_(node)
  {
    load_parameters();
    create_ros_interfaces();
    create_widgets();
    create_timers();

    linear_velocity_profile_.configure(
      linear_acceleration_,
      linear_jerk_);
    angular_velocity_profile_.configure(
      angular_acceleration_,
      angular_jerk_);
    last_base_update_time_ = SteadyClock::now();
    RCLCPP_INFO(
      node_->get_logger(),
      "GUI controller publishes base commands on %s.",
      cmd_vel_topic_.c_str());

    setWindowTitle("XCZS 巡操机器人控制台");
    setMinimumSize(620, 620);
    setStyleSheet(
      "QMainWindow { background: #eef2f6; }"
      "QGroupBox { font-weight: bold; border: 1px solid #9aa8b6;"
      " border-radius: 6px; margin-top: 10px; padding-top: 10px; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 10px;"
      " padding: 0 5px; }"
      "QPushButton { min-height: 34px; padding: 4px 12px; }"
      "QPushButton#motionButton { background: #0b4f7c; color: white;"
      " border: none; border-radius: 4px; }"
      "QPushButton#stopButton { background: #ef7d22; color: white;"
      " border: none; border-radius: 4px; }"
      "QPushButton#emergencyButton { background: #b3261e; color: white;"
      " border: none; border-radius: 4px; }");
  }

protected:
  void closeEvent(QCloseEvent * event) override
  {
    emergency_stop();
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    }
    QMainWindow::closeEvent(event);
  }

private:
  double positive_parameter(const std::string & name, double default_value)
  {
    const double value = node_->declare_parameter<double>(
      name,
      default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be positive.");
    }
    return value;
  }

  void load_parameters()
  {
    linear_speed_ = positive_parameter("linear_speed", 0.25);
    angular_speed_ = positive_parameter("angular_speed", 0.60);
    linear_acceleration_ = positive_parameter(
      "linear_acceleration",
      0.50);
    angular_acceleration_ = positive_parameter(
      "angular_acceleration",
      1.20);
    linear_jerk_ = positive_parameter("linear_jerk", 2.00);
    angular_jerk_ = positive_parameter("angular_jerk", 4.80);
    base_command_rate_ = positive_parameter("base_command_rate", 50.0);
    joint_step_ = positive_parameter("joint_step", 0.10);
    joint_limit_ = positive_parameter("joint_limit", 2.80);
    gripper_open_angle_ = positive_parameter(
      "gripper_open_angle",
      0.35);
    joint_command_rate_ = positive_parameter(
      "joint_command_rate",
      20.0);

    joint_command_repeats_ = node_->declare_parameter<int>(
      "joint_command_repeats",
      5);
    if (joint_command_repeats_ <= 0) {
      throw std::invalid_argument(
              "Parameter 'joint_command_repeats' must be positive.");
    }

    cmd_vel_topic_ = node_->declare_parameter<std::string>(
      "cmd_vel_topic",
      "/xczs/cmd_vel");
    trajectory_topic_ = node_->declare_parameter<std::string>(
      "joint_trajectory_topic",
      "/xczs/joint_trajectory");
    joint_state_topic_ = node_->declare_parameter<std::string>(
      "joint_state_topic",
      "/xczs/joint_states");
  }

  void create_ros_interfaces()
  {
    cmd_vel_publisher_ =
      node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10);
    trajectory_publisher_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic_,
      10);
    joint_state_subscription_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        joint_state_callback(*message);
      });
  }

  QPushButton * create_motion_button(const QString & text)
  {
    auto * button = new QPushButton(text);
    button->setObjectName("motionButton");
    return button;
  }

  void create_widgets()
  {
    auto * central_widget = new QWidget;
    auto * main_layout = new QVBoxLayout(central_widget);

    auto * base_group = new QGroupBox("底盘");
    auto * base_layout = new QGridLayout(base_group);
    auto * forward_button = create_motion_button("前进");
    auto * backward_button = create_motion_button("后退");
    auto * left_button = create_motion_button("左转");
    auto * right_button = create_motion_button("右转");
    auto * stop_button = new QPushButton("平滑停止");
    auto * emergency_button = new QPushButton("紧急停止");
    stop_button->setObjectName("stopButton");
    emergency_button->setObjectName("emergencyButton");

    base_layout->addWidget(forward_button, 0, 1);
    base_layout->addWidget(left_button, 1, 0);
    base_layout->addWidget(stop_button, 1, 1);
    base_layout->addWidget(right_button, 1, 2);
    base_layout->addWidget(backward_button, 2, 1);
    base_layout->addWidget(emergency_button, 3, 0, 1, 3);

    base_status_label_ = new QLabel("线速度 0.000 m/s ｜ 角速度 0.000 rad/s");
    base_status_label_->setAlignment(Qt::AlignCenter);
    base_layout->addWidget(base_status_label_, 4, 0, 1, 3);

    connect(
      forward_button, &QPushButton::pressed, this,
      [this]() {request_base_motion(linear_speed_, 0.0, "前进");});
    connect(
      backward_button, &QPushButton::pressed, this,
      [this]() {request_base_motion(-linear_speed_, 0.0, "后退");});
    connect(
      left_button, &QPushButton::pressed, this,
      [this]() {request_base_motion(0.0, angular_speed_, "左转");});
    connect(
      right_button, &QPushButton::pressed, this,
      [this]() {request_base_motion(0.0, -angular_speed_, "右转");});

    for (auto * button :
      {forward_button, backward_button, left_button, right_button})
    {
      connect(
        button, &QPushButton::released, this,
        [this]() {request_smooth_stop();});
    }
    connect(
      stop_button, &QPushButton::clicked, this,
      [this]() {request_smooth_stop();});
    connect(
      emergency_button, &QPushButton::clicked, this,
      [this]() {emergency_stop();});

    joint_group_ = new QGroupBox("六轴机械臂");
    auto * joint_layout = new QGridLayout(joint_group_);
    for (std::size_t index = 0; index < arm_spin_boxes_.size(); ++index) {
      auto * name_label = new QLabel(
        QString("关节 %1").arg(static_cast<int>(index + 1)));
      auto * spin_box = new QDoubleSpinBox;
      spin_box->setRange(-joint_limit_, joint_limit_);
      spin_box->setSingleStep(joint_step_);
      spin_box->setDecimals(2);
      spin_box->setSuffix(" rad");
      arm_spin_boxes_[index] = spin_box;

      joint_layout->addWidget(
        name_label,
        static_cast<int>(index),
        0);
      joint_layout->addWidget(
        spin_box,
        static_cast<int>(index),
        1);
      connect(
        spin_box,
        qOverload<double>(&QDoubleSpinBox::valueChanged),
        this,
        [this, index](double value) {
          if (syncing_joint_widgets_) {
            return;
          }
          joint_targets_[index] = value;
          schedule_joint_trajectory();
          status_label_->setText(
            QString("已发送关节 %1 目标").arg(
              static_cast<int>(index + 1)));
        });
    }

    auto * reset_arm_button = new QPushButton("机械臂回零");
    joint_layout->addWidget(
      reset_arm_button,
      static_cast<int>(arm_spin_boxes_.size()),
      0,
      1,
      2);
    connect(
      reset_arm_button, &QPushButton::clicked, this,
      [this]() {reset_arm();});
    joint_group_->setEnabled(false);

    gripper_group_ = new QGroupBox("夹爪");
    auto * gripper_layout = new QHBoxLayout(gripper_group_);
    auto * open_button = new QPushButton("打开");
    auto * close_button = new QPushButton("关闭");
    gripper_layout->addWidget(open_button);
    gripper_layout->addWidget(close_button);
    connect(
      open_button, &QPushButton::clicked, this,
      [this]() {set_gripper(true);});
    connect(
      close_button, &QPushButton::clicked, this,
      [this]() {set_gripper(false);});
    gripper_group_->setEnabled(false);

    status_label_ = new QLabel("正在等待 /xczs/joint_states");
    status_label_->setAlignment(Qt::AlignCenter);

    main_layout->addWidget(base_group);
    main_layout->addWidget(joint_group_);
    main_layout->addWidget(gripper_group_);
    main_layout->addWidget(status_label_);
    setCentralWidget(central_widget);
  }

  void create_timers()
  {
    ros_timer_ = new QTimer(this);
    ros_timer_->setInterval(10);
    connect(
      ros_timer_, &QTimer::timeout, this,
      [this]() {
        if (!rclcpp::ok()) {
          QApplication::quit();
          return;
        }
        rclcpp::spin_some(node_);
      });
    ros_timer_->start();

    base_timer_ = new QTimer(this);
    base_timer_->setInterval(
      std::max(
        1, static_cast<int>(std::lround(
          1000.0 / base_command_rate_))));
    connect(
      base_timer_, &QTimer::timeout, this,
      [this]() {publish_base_command();});
    base_timer_->start();

    joint_timer_ = new QTimer(this);
    joint_timer_->setInterval(
      std::max(
        1, static_cast<int>(std::lround(
          1000.0 / joint_command_rate_))));
    connect(
      joint_timer_, &QTimer::timeout, this,
      [this]() {
        if (pending_joint_publications_ > 0) {
          publish_joint_trajectory();
        }
      });
    joint_timer_->start();
  }

  void request_base_motion(
    double linear_velocity,
    double angular_velocity,
    const QString & status)
  {
    target_base_command_ = geometry_msgs::msg::Twist{};
    target_base_command_.linear.y = linear_velocity;
    target_base_command_.angular.z = angular_velocity;
    status_label_->setText("底盘：" + status);
  }

  void request_smooth_stop()
  {
    target_base_command_ = geometry_msgs::msg::Twist{};
    status_label_->setText("底盘：正在平滑停止");
  }

  void emergency_stop()
  {
    target_base_command_ = geometry_msgs::msg::Twist{};
    base_command_ = geometry_msgs::msg::Twist{};
    linear_velocity_profile_.reset();
    angular_velocity_profile_.reset();
    if (cmd_vel_publisher_ && rclcpp::ok()) {
      cmd_vel_publisher_->publish(base_command_);
    }
    if (status_label_) {
      status_label_->setText("底盘：已紧急停止");
    }
  }

  void publish_base_command()
  {
    if (!rclcpp::ok()) {
      return;
    }

    const auto current_time = SteadyClock::now();
    const double elapsed_seconds = std::clamp(
      std::chrono::duration<double>(
        current_time - last_base_update_time_).count(),
      0.0,
      2.0 / base_command_rate_);
    last_base_update_time_ = current_time;

    linear_velocity_profile_.set_target(target_base_command_.linear.y);
    angular_velocity_profile_.set_target(target_base_command_.angular.z);
    base_command_.linear.y =
      linear_velocity_profile_.update(elapsed_seconds);
    base_command_.angular.z =
      angular_velocity_profile_.update(elapsed_seconds);
    cmd_vel_publisher_->publish(base_command_);

    base_status_label_->setText(
      QString("线速度 %1 m/s ｜ 角速度 %2 rad/s")
      .arg(base_command_.linear.y, 0, 'f', 3)
      .arg(base_command_.angular.z, 0, 'f', 3));
  }

  void joint_state_callback(
    const sensor_msgs::msg::JointState & message)
  {
    if (joint_targets_initialized_) {
      return;
    }

    const std::size_t value_count =
      std::min(message.name.size(), message.position.size());
    for (std::size_t index = 0; index < value_count; ++index) {
      const auto joint_iterator = std::find(
        kControlledJoints.begin(),
        kControlledJoints.end(),
        message.name[index]);
      if (
        joint_iterator == kControlledJoints.end() ||
        !std::isfinite(message.position[index]))
      {
        continue;
      }

      const auto target_index = static_cast<std::size_t>(
        std::distance(kControlledJoints.begin(), joint_iterator));
      joint_targets_[target_index] = message.position[index];
      joint_state_received_[target_index] = true;
    }

    if (!std::all_of(
        joint_state_received_.begin(),
        joint_state_received_.end(),
        [](bool received) {return received;}))
    {
      return;
    }

    joint_targets_initialized_ = true;
    syncing_joint_widgets_ = true;
    for (std::size_t index = 0; index < arm_spin_boxes_.size(); ++index) {
      arm_spin_boxes_[index]->setValue(joint_targets_[index]);
    }
    syncing_joint_widgets_ = false;
    joint_group_->setEnabled(true);
    gripper_group_->setEnabled(true);
    status_label_->setText("机器人关节状态已同步");
    RCLCPP_INFO(
      node_->get_logger(),
      "GUI joint controls initialized from %s.",
      joint_state_topic_.c_str());
  }

  void reset_arm()
  {
    if (!joint_targets_initialized_) {
      return;
    }

    syncing_joint_widgets_ = true;
    for (std::size_t index = 0; index < kArmJoints.size(); ++index) {
      joint_targets_[index] = 0.0;
      arm_spin_boxes_[index]->setValue(0.0);
    }
    syncing_joint_widgets_ = false;
    schedule_joint_trajectory();
    status_label_->setText("机械臂：已发送回零目标");
  }

  void set_gripper(bool open)
  {
    if (!joint_targets_initialized_) {
      return;
    }

    const double target = open ? gripper_open_angle_ : 0.0;
    joint_targets_[6] = target;
    joint_targets_[7] = -target;
    schedule_joint_trajectory();
    status_label_->setText(open ? "夹爪：打开" : "夹爪：关闭");
  }

  void schedule_joint_trajectory()
  {
    pending_joint_publications_ = joint_command_repeats_;
    publish_joint_trajectory();
  }

  void publish_joint_trajectory()
  {
    if (!rclcpp::ok() || !joint_targets_initialized_) {
      pending_joint_publications_ = 0;
      return;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.frame_id = "world";
    trajectory.joint_names.assign(
      kControlledJoints.begin(),
      kControlledJoints.end());

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(joint_targets_.begin(), joint_targets_.end());
    trajectory.points.push_back(std::move(point));
    trajectory_publisher_->publish(trajectory);
    pending_joint_publications_ =
      std::max(0, pending_joint_publications_ - 1);
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;

  QTimer * ros_timer_{nullptr};
  QTimer * base_timer_{nullptr};
  QTimer * joint_timer_{nullptr};
  QGroupBox * joint_group_{nullptr};
  QGroupBox * gripper_group_{nullptr};
  QLabel * base_status_label_{nullptr};
  QLabel * status_label_{nullptr};
  std::array<QDoubleSpinBox *, 6> arm_spin_boxes_{};

  std::array<double, 8> joint_targets_{};
  std::array<bool, 8> joint_state_received_{};
  geometry_msgs::msg::Twist target_base_command_;
  geometry_msgs::msg::Twist base_command_;
  JerkLimitedVelocityProfile linear_velocity_profile_;
  JerkLimitedVelocityProfile angular_velocity_profile_;
  SteadyClock::time_point last_base_update_time_;

  std::string cmd_vel_topic_;
  std::string trajectory_topic_;
  std::string joint_state_topic_;
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
  int joint_command_repeats_{5};
  int pending_joint_publications_{0};
  bool joint_targets_initialized_{false};
  bool syncing_joint_widgets_{false};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int qt_argc = 1;
  char * qt_argv[] = {argv[0], nullptr};
  QApplication application(qt_argc, qt_argv);

  int exit_code = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>(
      "xczs_inspection_robot_gui");
    InspectionRobotWindow window(node);
    window.show();
    exit_code = application.exec();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("xczs_inspection_robot_gui"),
      "%s",
      exception.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
