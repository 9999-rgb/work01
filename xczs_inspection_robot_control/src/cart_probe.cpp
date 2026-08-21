// cart_probe.cpp — dock x standoff reliability sweep for the box_6 button
// approach, validating the dock/prepress_distance combination committed in
// cabinet_robot_adapter.yaml.
//
// Why: with the corrected axial tool mount, OMPL's folded-home -> box_6
// prepress swing sweeps the 0.39 m three-cylinder fingers through the adjacent
// box_5 module controls (box_5_knob / box_5_button_2), and move_group's dense
// path validation rejects those interpolated states ("Computed path is not
// valid").  The prepress TOOL pose is fixed in odom, so the arm articulation
// depends only on the dock; a larger prepress standoff pulls the OMPL goal away
// from the panel and clears the swing, while the final straight push is covered
// by the operator's Cartesian approach segment (prepress -> contact).
//
// This tool replicates the operator's set_pose_target_with_calibrated_ik_seed
// for both box_6 buttons, then measures P(success) of move_group's plan()
// VALIDATION across (dock x standoff) combos x 10 attempts.  The committed
// combo (dock 1.00,-0.37 body yaw -pi/2, prepress_distance 0.12) must hold
// successes_in_3 >= 1 (the operator retries motion_planning_attempts_=3 times
// and executes the first valid path) and ideally first_success == 1.
//
// Known limitation: is_free() only checks self-collision plus cabinet objects
// when they are present in the live GetPlanningScene response (kept_objs > 0);
// move_group's own plan() validation (which uses the real cabinet scene) is the
// ground truth.
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <map>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>

using moveit::planning_interface::MoveGroupInterface;

static void set_arm(moveit::core::RobotState & s, const std::vector<double> & v)
{
  const std::vector<std::string> names = {
    "r_arm_0_joint", "r_arm_1_joint", "r_arm_2_joint",
    "r_arm_3_joint", "r_arm_4_joint", "r_arm_5_joint", "r_arm_6_joint"};
  for (size_t i = 0; i < names.size() && i < v.size(); ++i) {
    s.setVariablePosition(names[i], v[i]);
  }
  s.update(true);
}

// Replicate the operator's make_tool_pose for the box_6 prepress contact.
static geometry_msgs::msg::Pose tool_pose(
  const Eigen::Quaterniond & base_q, double roll,
  const Eigen::Vector3d & tip_pos)
{
  Eigen::Quaterniond r = Eigen::Quaterniond(
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q = base_q * r;
  q.normalize();
  const Eigen::Vector3d tip_offset(0.0265, -0.061735, -0.3885);
  const Eigen::Vector3d link_pos = tip_pos - q * tip_offset;
  geometry_msgs::msg::Pose pose;
  pose.position.x = link_pos.x();
  pose.position.y = link_pos.y();
  pose.position.z = link_pos.z();
  pose.orientation.w = q.w(); pose.orientation.x = q.x();
  pose.orientation.y = q.y(); pose.orientation.z = q.z();
  return pose;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cart_probe");
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  for (const auto & arm : {"right_arm", "left_arm"}) {
    node->declare_parameter(
      "robot_description_kinematics." + std::string(arm) + ".kinematics_solver",
      "lma_kinematics_plugin/LMAKinematicsPlugin");
    node->declare_parameter(
      "robot_description_kinematics." + std::string(arm) + ".kinematics_solver_search_resolution",
      0.005);
    node->declare_parameter(
      "robot_description_kinematics." + std::string(arm) + ".kinematics_solver_timeout",
      0.3);
  }

  MoveGroupInterface mgi(node, "right_arm");
  mgi.setPoseReferenceFrame("odom");
  mgi.setEndEffectorLink("r_three_cyl_base");

  const moveit::core::JointModelGroup * rgroup =
    mgi.getRobotModel()->getJointModelGroup("right_arm");

  const Eigen::Quaterniond base_q(-0.70710678, 0.0, 0.70710678, 0.0);
  const Eigen::Vector3d button_face(1.9885, -0.119, 1.218);
  const Eigen::Vector3d inward(1.0, 0.0, 0.0);
  const Eigen::Vector3d prepress_pos = button_face - inward * 0.06;
  const geometry_msgs::msg::Pose prepress =
    tool_pose(base_q, 0.0, prepress_pos);

  RCLCPP_INFO(node->get_logger(),
              "prepress tool-link target: (%.4f, %.4f, %.4f)",
              prepress.position.x, prepress.position.y, prepress.position.z);

  const std::vector<std::pair<std::string, double>> home_poses = {
    {"l_arm_0_joint", 0.0}, {"l_arm_1_joint", -1.4}, {"l_arm_2_joint", 0.0},
    {"l_arm_3_joint", -0.3}, {"l_arm_4_joint", 0.0}, {"l_arm_5_joint", 0.7},
    {"l_arm_6_joint", -0.85},
    {"r_arm_0_joint", 0.0}, {"r_arm_1_joint", -1.6},
    {"r_arm_2_joint", -0.4}, {"r_arm_3_joint", 0.0},
    {"r_arm_4_joint", 0.0}, {"r_arm_5_joint", 0.8},
    {"r_arm_6_joint", 0.1},
    {"r_three_cyl_finger1_joint", 0.0}, {"r_three_cyl_finger2_joint", 0.0},
    {"r_three_cyl_finger3_joint", 0.0},
    {"l_two_cyl_finger1_joint", 0.0}, {"l_two_cyl_finger2_joint", 0.0},
    {"r_wheel_0_joint", 0.0}, {"r_wheel_1_joint", 0.0},
    {"l_wheel_0_joint", 0.0}, {"l_wheel_1_joint", 0.0},
  };

  // Clean cabinet-only scene: fresh PlanningScene + all cabinet_* collision
  // objects from the live monitored scene (their odom poses are correct; the
  // robot's own links as world objects are deliberately dropped).
  auto scene = std::make_shared<planning_scene::PlanningScene>(mgi.getRobotModel());
  int kept_objs = 0;
  {
    auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node->get_logger(), "get_planning_scene service not available");
      return 1;
    }
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components = 0xFFFF;  // request everything
    auto f = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, f) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "get_planning_scene call failed");
      return 1;
    }
    int total = 0;
    for (const auto & obj : f.get()->scene.world.collision_objects) {
      ++total;
      if (obj.id.rfind("cabinet_", 0) != 0) continue;
      scene->processCollisionObjectMsg(obj);
      ++kept_objs;
    }
    RCLCPP_INFO(node->get_logger(), "world objects: %d, cabinet kept: %d",
                total, kept_objs);
  }

  // Sanity: verify this fresh scene's self-collision verdict matches move_group's
  // own verdict (r_arm_0 <-> r_arm_5) for the exact seeded goal that failed.
  {
    auto g = std::make_shared<moveit::core::RobotState>(mgi.getRobotModel());
    g->setToDefaultValues();
    const auto * root_joint = mgi.getRobotModel()->getRootJoint();
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    root.linear() = Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    root.translation() = Eigen::Vector3d(1.230, -0.470, 0.580);
    g->setJointPositions(root_joint, root);
    for (const auto & [name, value] : home_poses) g->setVariablePosition(name, value);
    set_arm(*g, {1.764, 1.174, 0.031, 2.691, -1.397, -0.114, 2.287});
    g->update(true);
    collision_detection::CollisionRequest creq;
    creq.contacts = true; creq.max_contacts = 8;
    collision_detection::CollisionResult cself;
    scene->setCurrentState(*g);
    scene->checkSelfCollision(creq, cself);
    RCLCPP_INFO(node->get_logger(), "sanity seeded goal (move_group: r_arm_0<->r_arm_5): "
                "self_coll=%d", cself.collision ? 1 : 0);
    for (const auto & [names, contacts] : cself.contacts) {
      RCLCPP_INFO(node->get_logger(), "  sanity contact: %s <-> %s",
                  names.first.c_str(), names.second.c_str());
    }
  }

  auto state_from_dock = [&](double px, double py, double yaw) {
    auto s = std::make_shared<moveit::core::RobotState>(mgi.getRobotModel());
    s->setToDefaultValues();
    const auto * root_joint = mgi.getRobotModel()->getRootJoint();
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    root.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    root.translation() = Eigen::Vector3d(px, py, 0.580);
    if (root_joint != nullptr && root_joint->getVariableCount() > 0U) {
      s->setJointPositions(root_joint, root);
    }
    for (const auto & [name, value] : home_poses) {
      s->setVariablePosition(name, value);
    }
    s->update(true);
    return s;
  };

  auto is_free = [&](const moveit::core::RobotState & s) {
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 20;
    req.max_contacts_per_pair = 4;
    req.distance = true;
    req.verbose = false;
    collision_detection::CollisionResult self, world;
    scene->setCurrentState(s);
    scene->checkSelfCollision(req, self);
    if (self.collision) {
      for (const auto & [names, contacts] : self.contacts) {
        return names.first + " <-> " + names.second;
      }
      return std::string("self");
    }
    // robot vs cabinet: only meaningful if cabinet objects actually loaded
    if (kept_objs > 0) {
      scene->checkCollision(req, world, s);
      if (world.collision) {
        for (const auto & [names, contacts] : world.contacts) {
          return names.first + " <-> " + names.second;
        }
        return std::string("world");
      }
    }
    return std::string();  // free
  };

  // Buttons (odom faces, derived from cabinet_a frame at (2.0,0.33,0) quat
  // (0.5,-0.5,-0.5,0.5) applied to cabinet-local positions):
  //   box_6_button_1 local [0.449,1.218,0.011494] -> (1.9885,-0.119,1.218)
  //   box_6_button_2 local [0.499,1.218,0.011494] -> (1.9885,-0.169,1.218)
  //
  // The three-cylinder tool's long fingers (~0.39 m from the wrist) sweep
  // through the ADJACENT module's controls while the arm swings from the folded
  // home pose to the box_6 prepress; move_group rejects those paths
  // (box_6_button_1 clips box_5_knob, box_6_button_2 clips box_5_button_2).
  // The prepress TOOL pose is fixed in odom regardless of the dock, so the
  // same seeds drive the IK branch at every dock; sweep the dock and the
  // prepress standoff and report which combination survives move_group's
  // plan VALIDATION across the operator's motion_planning_attempts_ retries.
  struct ButtonSpec { const char * name; Eigen::Vector3d face; };
  const std::vector<ButtonSpec> buttons = {
    {"box_6_button_1", {1.9885, -0.119, 1.218}},
    {"box_6_button_2", {1.9885, -0.169, 1.218}},
  };
  const std::map<std::string, std::vector<double>> new_seeds = {
    {"box_6_button_1", {1.6780, 1.3430, -0.0230, 1.9800, -1.4740, -0.0400, 1.7490}},
    {"box_6_button_2", {1.6760, 1.4880, -0.0200, 1.8580, -1.4730, -0.0400, 1.7720}},
  };
  // Focused reliability: the full sweep found these (dock, standoff) combos
  // with a collision-free prepress goal for BOTH buttons.  Measure P(success)
  // across 10 attempts (the operator retries motion_planning_attempts_=3 times
  // and then EXECUTES the first valid path), so we pick a combo that succeeds
  // reliably rather than once.
  //
  // check_cabinet_model pins per-button docks at control_position.x + 0.35
  // (button_1 -> dock y=-0.469, button_2 -> dock y=-0.519).  The committed
  // shared dock y=-0.37 violates that contract; re-sweep the check-compliant
  // dock Y values across dock X to see whether per-button docks are reliable
  // with the corrected axial mount.  Standoff (prepress) held at 0.12.
  struct Combo { double x, y, sd; };
  const std::vector<Combo> combos = {
    {1.00, -0.469, 0.12}, {1.10, -0.469, 0.12}, {1.20, -0.469, 0.12},
    {0.90, -0.469, 0.12},
    {1.00, -0.519, 0.12}, {1.10, -0.519, 0.12}, {1.20, -0.519, 0.12},
    {0.90, -0.519, 0.12},
    {1.00, -0.37, 0.12}};
  constexpr int attempts = 10;
  const double yaw = -1.57079632679;
  for (const auto & b : buttons) {
    for (const auto & c : combos) {
      auto start = state_from_dock(c.x, c.y, yaw);
      moveit_msgs::msg::RobotState start_msg;
      moveit::core::robotStateToRobotStateMsg(*start, start_msg);
      const Eigen::Vector3d tip = b.face - Eigen::Vector3d(c.sd, 0.0, 0.0);
      const geometry_msgs::msg::Pose p = tool_pose(base_q, 0.0, tip);
      moveit::core::RobotState seed = *start;
      set_arm(seed, new_seeds.at(b.name));
      seed.update(true);
      const bool ik_ok = seed.setFromIK(rgroup, p, "r_three_cyl_base", 1.0);
      std::vector<double> gv;
      seed.copyJointGroupPositions(rgroup, gv);
      const std::string why = is_free(seed);
      mgi.setJointValueTarget(gv);
      int first_success = 0, successes_in_3 = 0, successes = 0;
      for (int attempt = 1; attempt <= attempts; ++attempt) {
        mgi.setStartState(start_msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          if (first_success == 0) first_success = attempt;
          if (attempt <= 3) ++successes_in_3;
          ++successes;
        }
      }
      RCLCPP_INFO(node->get_logger(),
                  "RELI %s dock=(%.2f,%.2f) standoff=%.2f ik=%d goal=%s "
                  "first=%d in3=%d/3 tot=%d/%d",
                  b.name, c.x, c.y, c.sd, ik_ok ? 1 : 0,
                  why.empty() ? "free" : why.c_str(),
                  first_success, successes_in_3, successes, attempts);
    }
  }

  return 0;
}
