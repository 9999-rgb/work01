// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

struct PlannerOptions
{
  std::string group{"manipulator"};
  std::string named_target{"home"};
  std::string frame_id{"odom"};
  bool execute{false};
  bool has_pose{false};
  std::vector<double> pose;
};

double parse_double(const std::string & value, const std::string & option)
{
  std::size_t parsed_length = 0;
  const double result = std::stod(value, &parsed_length);
  if (parsed_length != value.size()) {
    throw std::invalid_argument("Invalid number for " + option + ": " + value);
  }
  return result;
}

PlannerOptions parse_arguments(const std::vector<std::string> & arguments)
{
  PlannerOptions options;
  for (std::size_t index = 1; index < arguments.size(); ++index) {
    const auto & argument = arguments[index];
    if (argument == "--group" && index + 1 < arguments.size()) {
      options.group = arguments[++index];
    } else if (argument == "--named" && index + 1 < arguments.size()) {
      options.named_target = arguments[++index];
      options.has_pose = false;
    } else if (argument == "--frame" && index + 1 < arguments.size()) {
      options.frame_id = arguments[++index];
    } else if (argument == "--pose" && index + 7 < arguments.size()) {
      options.pose.clear();
      for (std::size_t value_index = 0; value_index < 7; ++value_index) {
        options.pose.push_back(
          parse_double(arguments[++index], "--pose"));
      }
      options.has_pose = true;
    } else if (argument == "--execute") {
      options.execute = true;
    } else if (argument == "--plan-only") {
      options.execute = false;
    } else if (argument == "-h" || argument == "--help") {
      std::cout
        << "Usage:\n"
        << "  moveit_planner --named home [--execute]\n"
        << "  moveit_planner --pose X Y Z QX QY QZ QW "
        << "[--frame odom] [--execute]\n"
        << "Options:\n"
        << "  --group NAME    MoveIt planning group (default: manipulator)\n"
        << "  --plan-only     Plan without execution (default)\n"
        << "  --execute       Execute a successful plan\n";
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("Unknown or incomplete option: " + argument);
    }
  }
  return options;
}

void copy_kinematics_parameters(const rclcpp::Node::SharedPtr & node)
{
  const std::vector<std::string> parameter_names{
    "robot_description_kinematics.manipulator.kinematics_solver",
    "robot_description_kinematics.manipulator.kinematics_solver_attempts",
    "robot_description_kinematics.manipulator.kinematics_solver_search_resolution",
    "robot_description_kinematics.manipulator.kinematics_solver_timeout",
  };
  const auto parameter_client =
    std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!parameter_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(
      node->get_logger(),
      "MoveIt kinematics parameters are unavailable from /move_group.");
    return;
  }

  for (const auto & parameter :
    parameter_client->get_parameters(parameter_names))
  {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      node->declare_parameter(
        parameter.get_name(), parameter.get_parameter_value());
    }
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = EXIT_FAILURE;

  try {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    const auto options = parse_arguments(arguments);
    auto node_options = rclcpp::NodeOptions();
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.arguments(
      {"--ros-args", "--remap", "joint_states:=/xczs/joint_states"});
    const auto node = rclcpp::Node::make_shared(
      "xczs_moveit_planner", node_options);

    copy_kinematics_parameters(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() {executor.spin();});

    try {
      moveit::planning_interface::MoveGroupInterface move_group(
        node, options.group);
      move_group.setPlanningTime(5.0);
      move_group.setNumPlanningAttempts(5);
      move_group.setMaxVelocityScalingFactor(0.25);
      move_group.setMaxAccelerationScalingFactor(0.25);
      move_group.setGoalPositionTolerance(0.005);
      move_group.setGoalOrientationTolerance(0.01);
      move_group.setGoalJointTolerance(0.001);

      if (options.has_pose) {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = options.frame_id;
        target.header.stamp = node->now();
        target.pose.position.x = options.pose[0];
        target.pose.position.y = options.pose[1];
        target.pose.position.z = options.pose[2];
        target.pose.orientation.x = options.pose[3];
        target.pose.orientation.y = options.pose[4];
        target.pose.orientation.z = options.pose[5];
        target.pose.orientation.w = options.pose[6];
        move_group.setPoseTarget(target, "end");
      } else if (!move_group.setNamedTarget(options.named_target)) {
        throw std::runtime_error(
                "Unknown named target '" + options.named_target +
                "' for group '" + options.group + "'.");
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned =
        move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      if (!planned) {
        RCLCPP_ERROR(node->get_logger(), "MoveIt failed to find a valid plan.");
      } else if (!options.execute) {
        RCLCPP_INFO(
          node->get_logger(),
          "MoveIt plan succeeded. Re-run with --execute to execute it.");
        exit_code = EXIT_SUCCESS;
      } else {
        const bool executed =
          move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (executed) {
          RCLCPP_INFO(node->get_logger(), "MoveIt trajectory execution succeeded.");
          exit_code = EXIT_SUCCESS;
        } else {
          RCLCPP_ERROR(node->get_logger(), "MoveIt trajectory execution failed.");
        }
      }
    } catch (...) {
      executor.cancel();
      executor_thread.join();
      throw;
    }

    executor.cancel();
    executor_thread.join();
  } catch (const std::exception & error) {
    std::cerr << "MoveIt planner error: " << error.what() << '\n';
  }

  rclcpp::shutdown();
  return exit_code;
}
