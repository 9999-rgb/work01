#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <iostream>
#include <cstdlib>
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
    rclcpp::Parameter("moveit_controller_manager", "moveit_simple_controller_manager/MoveItSimpleControllerManager"),
    rclcpp::Parameter("moveit_simple_controller_manager.controller_names", std::vector<std::string>{"probe"}),
    rclcpp::Parameter("moveit_simple_controller_manager.probe.type", "FollowJointTrajectory"),
    rclcpp::Parameter("moveit_simple_controller_manager.probe.action_ns", "follow_joint_trajectory"),
    rclcpp::Parameter("moveit_simple_controller_manager.probe.joints", std::vector<std::string>{"probe_joint"})});
  auto node = std::make_shared<rclcpp::Node>("tem_shutdown_probe", options);
  using Action = control_msgs::action::FollowJointTrajectory;
  auto server = rclcpp_action::create_server<Action>(node, "/probe/follow_joint_trajectory",
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) { return rclcpp_action::GoalResponse::REJECT; },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>) {});
  auto manager = std::make_unique<trajectory_execution_manager::TrajectoryExecutionManager>(node, nullptr, nullptr, false);
  std::vector<std::string> controllers;
  if (!manager->getControllerManager())
    std::_Exit(2);
  manager->getControllerManager()->getControllersList(controllers);
  if (controllers != std::vector<std::string>{"probe"} ||
      !manager->getControllerManager()->getControllerHandle("probe"))
    std::_Exit(3);
  std::cout << "Controller manager constructed; requesting shutdown" << std::endl;
  const bool shutdown_first = argc < 2 || std::string(argv[1]) != "live";
  if (shutdown_first)
    rclcpp::shutdown();
  manager.reset();
  if (!shutdown_first)
    rclcpp::shutdown();
  std::cout << "Controller manager destroyed cleanly" << std::endl;
  return 0;
}
