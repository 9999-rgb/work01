// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace xczs_inspection_robot_control
{

class CabinetButtonPlugin final : public gazebo::ModelPlugin
{
public:
  ~CabinetButtonPlugin() override
  {
    update_connection_.reset();
  }

  void Load(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_) {
      gzerr << "Cabinet button plugin could not create a ROS 2 node.\n";
      return;
    }
    const std::string joint_name = sdf->Get<std::string>(
      "joint_name", "box_10_box_10_button_1").first;
    joint_ = model_->GetJoint(joint_name);
    if (!joint_) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Cabinet button plugin could not find joint '%s'.",
        joint_name.c_str());
      return;
    }

    std::string joint_state_topic;
    std::string pressed_topic;
    try {
      spring_stiffness_ = positive_sdf_value(
        sdf, "spring_stiffness", 200.0);
      spring_damping_ = positive_sdf_value(
        sdf, "spring_damping", 6.0);
      press_threshold_ = positive_sdf_value(
        sdf, "press_threshold", 0.006);
      release_threshold_ = positive_sdf_value(
        sdf, "release_threshold", 0.003);
      publish_rate_ = positive_sdf_value(
        sdf, "state_publish_rate", 50.0);
      joint_state_topic = sdf->Get<std::string>(
        "joint_state_topic",
        "/xczs/cabinet/box_10_button_1/joint_states").first;
      pressed_topic = sdf->Get<std::string>(
        "pressed_topic",
        "/xczs/cabinet/box_10_button_1/pressed").first;
      if (joint_state_topic.empty() || joint_state_topic.front() != '/' ||
        pressed_topic.empty() || pressed_topic.front() != '/')
      {
        throw std::invalid_argument(
                "Cabinet button state topics must use absolute ROS names.");
      }
      if (release_threshold_ >= press_threshold_) {
        throw std::invalid_argument(
                "Cabinet button release threshold must be below the press "
                "threshold.");
      }
      if (press_threshold_ >= joint_->UpperLimit(0)) {
        throw std::invalid_argument(
                "Cabinet button press threshold must be below the joint "
                "upper limit.");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", error.what());
      joint_.reset();
      return;
    }

    joint_state_publisher_ =
      ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic, 10);
    pressed_publisher_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
      pressed_topic,
      rclcpp::QoS(1).reliable().transient_local());

    joint_name_ = joint_name;
    publish_period_ = 1.0 / publish_rate_;
    last_publish_time_ = model_->GetWorld()->SimTime().Double() -
      publish_period_;
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&CabinetButtonPlugin::on_update, this));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Interactive cabinet button ready on joint %s (travel threshold %.1f mm).",
      joint_name_.c_str(), press_threshold_ * 1000.0);
  }

  void Reset() override
  {
    pressed_ = false;
    last_publish_time_ = model_->GetWorld()->SimTime().Double() -
      publish_period_;
  }

private:
  static double positive_sdf_value(
    const sdf::ElementPtr & sdf,
    const std::string & name,
    double default_value)
  {
    const double value = sdf->Get<double>(name, default_value).first;
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
              "Cabinet button parameter '" + name +
              "' must be positive.");
    }
    return value;
  }

  void on_update()
  {
    const double raw_position = joint_->Position(0);
    const double velocity = joint_->GetVelocity(0);
    joint_->SetForce(
      0, -spring_stiffness_ * raw_position - spring_damping_ * velocity);

    const double position = std::max(0.0, raw_position);
    if (!pressed_ && position >= press_threshold_) {
      pressed_ = true;
    } else if (pressed_ && position <= release_threshold_) {
      pressed_ = false;
    }

    const double simulation_time =
      model_->GetWorld()->SimTime().Double();
    if (simulation_time - last_publish_time_ < publish_period_) {
      return;
    }
    last_publish_time_ = simulation_time;

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = ros_node_->get_clock()->now();
    joint_state.name.push_back(joint_name_);
    joint_state.position.push_back(position);
    joint_state.velocity.push_back(velocity);
    joint_state_publisher_->publish(joint_state);

    std_msgs::msg::Bool pressed_message;
    pressed_message.data = pressed_;
    pressed_publisher_->publish(pressed_message);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pressed_publisher_;

  std::string joint_name_;
  double spring_stiffness_{200.0};
  double spring_damping_{6.0};
  double press_threshold_{0.006};
  double release_threshold_{0.003};
  double publish_rate_{50.0};
  double publish_period_{0.02};
  double last_publish_time_{0.0};
  bool pressed_{false};
};

GZ_REGISTER_MODEL_PLUGIN(CabinetButtonPlugin)

}  // namespace xczs_inspection_robot_control
