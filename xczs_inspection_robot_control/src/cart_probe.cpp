// cart_probe.cpp — diagnostic mirroring the operator's button press geometry
// for box_6_button_1, sweeping tool_roll_offset to find a wrist configuration
// where the prepress->contact Cartesian approach is feasible:
//   for each roll r: tool_pose = base_quat * R_z(r)
//     1. prepress tool pose + ready_joint_seed -> setFromIK (1.0s) -> joints
//     2. contact pose seeded from prepress joints -> setFromIK -> contact joints
//     3. computeCartesianPath(prepress joints -> contact pose), fraction ON/OFF
// Tool poses derived from the odom->cabinet_a_frame contract (confirmed
// numerically: face at (1.9885,-0.119,1.218), prepress link at
// (1.540,-0.0573,1.1915), contact at (1.599,-0.0573,1.1915)).
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

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

static std::string join_joints(const moveit::core::RobotState & s,
                               const moveit::core::JointModelGroup * g)
{
  std::vector<double> v;
  s.copyJointGroupPositions(g, v);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3);
  for (double x : v) ss << x << " ";
  return ss.str();
}

// Replicate the operator's make_tool_pose: the tool's business-point tip sits
// at the given odom tip position, and the tool LINK is offset by the roll-rotated
// tool_tip_position so the approach/contact frames stay on the press line.
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
  mgi.setPlanningTime(5.0);

  const moveit::core::JointModelGroup * rgroup =
    mgi.getRobotModel()->getJointModelGroup("right_arm");

  // tool orientation at roll=0: RPY(90,0,-90) -> quat (-0.7071,0,0.7071,0)
  const Eigen::Quaterniond base_q(-0.70710678, 0.0, 0.70710678, 0.0);

  // TIP (business-point) positions in odom, per the box_6_button_1 contract:
  // face (1.9885,-0.119,1.218), inward=+X, prepress_distance 0.06,
  // contact_clearance 0.001.  tool_pose() turns these into link poses exactly
  // like the operator's make_tool_pose().
  const Eigen::Vector3d button_face(1.9885, -0.119, 1.218);
  const Eigen::Vector3d inward(1.0, 0.0, 0.0);
  const Eigen::Vector3d prepress_pos = button_face - inward * 0.06;
  const Eigen::Vector3d contact_pos = button_face - inward * 0.001;

  const std::vector<double> ready_joint_seed = {
    1.6929, 0.9137, -0.0395, 2.1282, -1.4704, -0.0216, 1.4698};

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
    {"r_rotbtn_rotate_joint", 0.0},
    {"r_rotbtn_jaw1_joint", 0.0}, {"r_rotbtn_jaw2_joint", 0.0},
    {"l_two_cyl_finger1_joint", 0.0}, {"l_two_cyl_finger2_joint", 0.0},
    {"l_rocker_rotor_joint", 0.0},
    {"r_wheel_0_joint", 0.0}, {"r_wheel_1_joint", 0.0},
    {"l_wheel_0_joint", 0.0}, {"l_wheel_1_joint", 0.0},
  };

  moveit::core::RobotState base(mgi.getRobotModel());
  base.setToDefaultValues();
  const auto * root_joint = mgi.getRobotModel()->getRootJoint();
  if (root_joint != nullptr && root_joint->getVariableCount() > 0U) {
    // Docked base for box_6_button_1, confirmed empirically in the prior
    // session (odom->body at the box_6 staging dock).  The robot's live
    // odom->body is meaningless until the base is physically docked, and Nav2
    // AMCL can be mislocalized on a fresh boot, so the sweep tests the exact
    // docked geometry directly.
    const double px = 1.227, py = -0.462, pz = 0.58, yaw = 0.002;
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    root.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    root.translation() = Eigen::Vector3d(px, py, pz);
    base.setJointPositions(root_joint, root);
    RCLCPP_INFO(node->get_logger(),
                "base odom->body (fixed dock): %.4f %.4f %.4f  yaw=%.4f",
                px, py, pz, yaw);
  }
  for (const auto & [name, value] : home_poses) base.setVariablePosition(name, value);
  base.update(true);

  const double travel = (contact_pos - prepress_pos).norm();
  RCLCPP_INFO(node->get_logger(), "tip travel total=%.4f m", travel);

  // The LMA IK-only sweep cannot reach the box_6 prepress pose from the real
  // dock, but LMA gets stuck on folded configurations.  The decisive test is the
  // full MoveIt planner (OMPL), which explores joint space globally: plan from
  // the docked HOME posture to the operator's roll=0 prepress pose.  If OMPL
  // finds a plan, the pose is reachable and the operator just needs a better
  // seed; if not, the box_6 station dock is genuinely out of reach and must
  // move before any seed can help.
  // Second-row buttons (box_3/box_4 at height 1.547) compute to wrist span
  // ~0.7 m from their configured docks — comfortably inside the arm's envelope.
  // OMPL (collision-aware) decides whether each is truly pressable; on success
  // the plan's goal joints are the candidate ready_joint_seed.
  const double base_z = 0.58, base_yaw = 0.002;
  struct ButtonProbe {
    const char * name;
    Eigen::Vector3d face;   // odom
    double dock_x, dock_y;  // odom body X/Y for this button's station
  };
  const std::vector<ButtonProbe> probes = {
    // face = cabinet->odom(0.120/0.449, 1.547, 0.0115) ; dock from station anchor+standoff
    {"box_3_button_1", {1.9885, 0.210, 1.547}, 1.07, -0.19},
    {"box_3_button_2", {1.9885, 0.160, 1.547}, 1.07, -0.19},
    {"box_4_button_1", {1.9885, -0.119, 1.547}, 1.07, -0.469},
    {"box_4_button_2", {1.9885, -0.169, 1.547}, 1.07, -0.469},
    {"box_5_button_1", {1.9885, 0.210, 1.218}, 1.231, -0.14},
    {"box_6_button_1", {1.9885, -0.119, 1.218}, 1.231, -0.469},
    {"box_7_button_1", {1.9885, 0.210, 0.889}, 1.231, -0.14},
    {"box_8_button_1", {1.9885, -0.119, 0.889}, 1.231, -0.469},
  };
  // The HOME transport posture at the dock may poke the tool into the cabinet.
  // Print where the right tool tip/wrist sit at each dock to see how close the
  // fixed posture comes to the cabinet face (odom x = 2.000).
  for (const auto & probe : probes) {
    moveit::core::RobotState cell(base);
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    root.linear() = Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    root.translation() = Eigen::Vector3d(probe.dock_x, probe.dock_y, base_z);
    cell.setJointPositions(root_joint, root);
    cell.update(true);
    const Eigen::Isometry3d wrist = cell.getGlobalLinkTransform("r_arm_6");
    const Eigen::Isometry3d tb = cell.getGlobalLinkTransform("r_three_cyl_base");
    const Eigen::Isometry3d finger = cell.getGlobalLinkTransform("r_three_cyl_finger1");
    RCLCPP_INFO(node->get_logger(),
                "%s dock=(%.2f,%.2f) home wrist=(%.3f,%.3f,%.3f) toolbase=(%.3f,%.3f,%.3f) f1=(%.3f,%.3f,%.3f)",
                probe.name, probe.dock_x, probe.dock_y,
                wrist.translation().x(), wrist.translation().y(), wrist.translation().z(),
                tb.translation().x(), tb.translation().y(), tb.translation().z(),
                finger.translation().x(), finger.translation().y(), finger.translation().z());
  }

  return 0;
}
