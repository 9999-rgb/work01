// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "Eigen/Cholesky"
#include "Eigen/Geometry"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/utils/moveit_error_code.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "xczs_inspection_robot_interfaces/action/operate_cabinet_control.hpp"
#include "xczs_inspection_robot_interfaces/action/press_cabinet_button.hpp"
#include "xczs_inspection_robot_control/action_terminal_policy.hpp"
#include "xczs_inspection_robot_control/cabinet_grasp_safety_policy.hpp"
#include "xczs_inspection_robot_control/drawer_hook_seat_projection.hpp"
#include "xczs_inspection_robot_control/drawer_seat_confirm.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control_catalog.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control_state.hpp"
#include "xczs_inspection_robot_control/operation_validation_policy.hpp"
#include "xczs_inspection_robot_control/rotary_operation_policy.hpp"
#include "xczs_inspection_robot_control/router_utils.hpp"
#include "xczs_inspection_robot_control/staging_safety_policy.hpp"
#include "xczs_inspection_robot_control/structured_control_state_policy.hpp"
#include "xczs_inspection_robot_interfaces/srv/manage_operation_lease.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_bimanual_grasp.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_grasp.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_unlock.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using namespace std::chrono_literals;
using PressCabinetButton =
  xczs_inspection_robot_interfaces::action::PressCabinetButton;
using PressGoalHandle = rclcpp_action::ServerGoalHandle<PressCabinetButton>;
using OperateCabinetControl =
  xczs_inspection_robot_interfaces::action::OperateCabinetControl;
using OperateGoalHandle =
  rclcpp_action::ServerGoalHandle<OperateCabinetControl>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using MoveGroupInterface =
  moveit::planning_interface::MoveGroupInterface;
using ManageOperationLease =
  xczs_inspection_robot_interfaces::srv::ManageOperationLease;

constexpr char kActionName[] = "press_cabinet_button";
constexpr char kOperateActionName[] = "operate_cabinet_control";
constexpr char kControlCatalogTopic[] = "control_catalog";
constexpr char kActiveControlTopic[] = "active_control";
constexpr char kOperationHeartbeatTopic[] = "operation_heartbeat";
constexpr char kOperationFaultTopic[] = "operation_fault";
constexpr char kResetControlsService[] = "reset_controls";
constexpr char kEmbeddedNavigationDisabledMessage[] =
  "Embedded cabinet navigation is disabled because navigation is executed "
  "independently by the task layer.";

class OperationError : public std::runtime_error
{
public:
  OperationError(std::uint8_t error_code, const std::string & message)
  : std::runtime_error(message), error_code(error_code)
  {
  }

  std::uint8_t error_code;
};

class GenericOperationError : public std::runtime_error
{
public:
  GenericOperationError(std::uint8_t error_code, const std::string & message)
  : std::runtime_error(message), error_code(error_code)
  {
  }

  std::uint8_t error_code;
};

// 2026-09-03 AGENT §7.2: 分级封顶调试的成功标记，不是失败。debug_stage_cap>0
// 的打开流程执行到指定阶段边界后，在分支内完成与正常尾段一致的安全序列
// （脱离/电缸回位/退让收起/轨道闩重锁），再抛本异常；execute_operate 的专用
// catch 在调用失败回收之前截获它，把结果按成功交付并把封顶证据写入
// result->message。映射（closing 忽略，仅打开）：1 = 单工作位姿就位（§8.3）；
// 2 = 双侧钩爪闭合（§8.4）；3 = 双侧支撑入缝（§8.5）；4 = 解锁按压达标未放行
// （§8.6）；5 = 解锁服务放行（§8.7）；6 = attach 耦合保持 3 s（§8.8）；
// 7 = 拉距封顶 0.03 m（§8.9）；8 = 拉距封顶 0.10 m（§8.10）。
class DebugStageCapReached : public std::runtime_error
{
public:
  DebugStageCapReached(int stage, const std::string & message)
  : std::runtime_error(message), stage(stage)
  {
  }

  int stage{0};
};

struct OperationPoses
{
  geometry_msgs::msg::Pose prepress_pose;
  geometry_msgs::msg::Pose contact_pose;
  geometry_msgs::msg::Pose pressed_pose;
};

struct RotaryOperationPoses
{
  geometry_msgs::msg::Pose ready_pose;
  // Near-grasp standoff used by both knobs and doors.  A pose-only OMPL plan
  // to the far ready pose can land on an IK branch whose long straight inward
  // Cartesian approach self-collides (r_arm_0 <-> r_arm_2).  Planning to a
  // pose only a few mm from the grasp point first keeps the final approach
  // short enough that any branch completes it.
  geometry_msgs::msg::Pose pregrasp_pose;
  geometry_msgs::msg::Pose grasp_pose;
};

struct DoorArcProgress
{
  bool in_progress{false};
  double initial_position{0.0};
  std::size_t completed_waypoints{0U};
  std::size_t total_waypoints{0U};
};

struct ControlNavigationStation
{
  tf2::Vector3 local_anchor{0.0, 0.0, 0.0};
  tf2::Vector3 outward_axis{0.0, 0.0, 1.0};
  double standoff{0.0};
  double base_yaw_offset{0.0};
  std::string frame_id;
};

struct ControlStagingPoses
{
  geometry_msgs::msg::PoseStamped navigation_pose;
  geometry_msgs::msg::PoseStamped planning_pose;
};

// One control type's robot-side tool binding.  Every cabinet control type is
// operated by exactly one arm (MoveIt group) whose contact link
// presses/grasps/rocks the control; ``transport_named_target`` is that arm's
// safe retreat pose in the SRDF.
struct ToolProfile
{
  enum class ToolAxisOrientation
  {
    // Tool local +Z points along the control's outward (retraction) axis.
    // Matches the legacy three-cylinder design whose business point sits at
    // the opposite (-Z) end, so +Z is the empty, non-contacting side.
    ALONG_OUTWARD,
    // Tool local +Z points toward the control (opposite the outward axis).
    // Required by pincer tools (rotate_button, rocker) whose jaws / rotor are
    // at the +Z end: keeping +Z == outward would push the wrist past the
    // control into the cabinet, making the grasp pose unreachable.
    TOWARD_CONTROL,
  };

  std::string move_group;
  std::string contact_tool_link;
  // Link carrying the cabinet grasp-distance probe.  Defaults to
  // contact_tool_link.  Pincer tools
  // (rotate_button) carry their jaws far from the contact-tool origin, so the
  // body origin can sit well outside the grasp distance threshold even when
  // the jaws are exactly on the control; the jaw-carrier link is a better
  // distance probe.
  std::string grasp_link;
  // Exact grasp-distance probe in grasp_link coordinates.  Keeping the point
  // separate from the MoveIt business point also supports tools whose planned
  // contact link and Gazebo grasp link differ.
  tf2::Vector3 grasp_point_position{0.0, 0.0, 0.0};
  std::string transport_named_target;
  ToolAxisOrientation tool_axis_orientation{ToolAxisOrientation::ALONG_OUTWARD};
  // Full 3-D position of the physical business point in contact_tool_link.
  // A scalar axial offset cannot represent an off-axis finger and can align
  // the empty tool centre with a button while a real finger hits the panel.
  tf2::Vector3 tool_tip_position{0.0, 0.0, 0.0};
  // A business point on a movable tool link is valid only at its calibrated
  // tool-joint state.  Reject operation instead of silently using a stale
  // geometric offset after manual tool motion.
  std::vector<std::string> calibration_joint_names;
  std::vector<double> calibration_joint_positions;
};

// Per-arm tool profile for a bimanual drawer operation.  The right
// three-cylinder (right_arm) unlocks and grasps the right handle; the left
// two-cylinder (left_arm) grasps the left handle.  Two separate MoveIt move
// groups and two separate trajectory controllers let both arms move in
// synchrony during the pull/push.
struct BimanualToolProfile
{
  std::string move_group;
  std::string contact_tool_link;
  tf2::Vector3 tool_tip_position{0.0, 0.0, 0.0};
  std::vector<std::string> calibration_joint_names;
  std::vector<double> calibration_joint_positions;
  std::string transport_named_target{"home"};
  double tool_roll_offset{0.0};
  // ---- 2026-09-03 AGENT §4.2: per-rod role contract (P1 末端电机职责) ----
  // 支撑与夹爪不再是「机械臂位姿」的隐含语义，而是各自命名的伸缩电缸
  // （移动关节 + 移动连杆 + 连杆局部系内的杆端接触偏移 + 行程位置）。角色
  // 字段缺失/为空表示该侧该角色未启用（例如只做位姿支撑的旧行为）。接触
  // 数值按 §7.2 阶段 4/5 现场封定后写入 adapter.yaml。
  bool has_support_role{false};
  std::string support_joint;                 // 支撑电缸移动关节（在 calibration 内）
  std::string support_contact_link;          // 支撑电缸移动连杆（杆端载体）
  tf2::Vector3 support_contact_point_local{0.0, 0.0, 0.0};  // 杆端在该连杆局部系
  // 杆端在 contact_tool_link 局部系、对应移动关节为零位时的位置。机械臂
  // 位姿必须按真实杆列而不是抽象 tool_tip_position 计算；伸出量沿工具 -Z。
  tf2::Vector3 support_contact_point_base_at_zero{0.0, 0.0, 0.0};
  double support_retracted_position{0.0};
  double support_contact_position{0.0};      // >0 启用支撑伸出阶段（§7.2 封定）
  // 2026-09-05 cap5 fix#16: 支撑杆端在缝口平面内的名义停留深度（世界 −X，
  // 沿杆伸入缝内）。support_contact_position 是「固定 q 目标」，由一次 cap-1
  // 探针的 q0 悬停 x0 反推（q = x0 − 0.097）；但 x0 随停靠/工作位姿 run 间
  // 漂移（实测 cap5 run 左 0.1295 vs 封定 0.1376，~9mm 更近），固定 q 在无
  // 碰撞的缝内空腔深插过缝口 → support 距离门必挂。>0 时 drive_drawer_
  // support_stage 改用活体 FK 自标定 q = x0.x − 缝口x + inset，杆端无论 x0
  // 落点都停在缝口内侧 inset 处；默认 0 = 固定 q 旧语义（其余抽屉零回归）。
  double support_contact_inset{0.0};
  bool has_gripper_role{false};
  std::string gripper_joint;                 // 夹爪电缸移动关节
  std::string gripper_contact_link;          // 夹爪电缸移动连杆
  tf2::Vector3 gripper_contact_point_local{0.0, 0.0, 0.0};
  tf2::Vector3 gripper_contact_point_base_at_zero{0.0, 0.0, 0.0};
  double gripper_open_position{0.0};
  double gripper_grasp_position{0.0};        // >0 启用夹爪闭合阶段（§7.2 封定）
  // ---- 2026-09-04 AGENT §8.4 fix#3 v3 (cap2 几何根因): 工作位姿 aim 补偿 ----
  // 深压探针取证（seal_geom_diag.csv, rod≈0 工作位姿): 到达后钩爪逻辑触点在
  // cabinet 局部系相对 handle 目标存在稳定横向偏差 —— 左 dy+6/dz+7mm、右
  // dy+13/dz−3mm; 右杆 y=4.7065 越过立板面北缘（实测面前缘 y≤4.701, b1.STL
  // front-face 实测 y∈[4.099,4.701]）→ 从不上板, 压贴变成"滑穿自由区"假证据。
  // 修复 = 把整臂工作位姿瞄准目标按实测反向平移, 令真实杆端落在把手中心,
  // 几何门恢复真校验（handle/support 合同点与全部阈值不动）。值为 cabinet 局部
  // 系 3-向量, 加到 drawer_side_point 瞄准目标上; 只影响 db1 双臂抽拉这组工具
  // （adapter drawer_tools.left/right），其余抽屉/场景默认 [0,0,0] 零回归。
  tf2::Vector3 drawer_pose_aim_offset{0.0, 0.0, 0.0};
  // ---- 2026-09-04 AGENT §8.4 fix #3 v2: 钩爪封定探针参数（cap2 run1/run2
  // 取证，把旧 position-penetration 判据改成力确认压贴，见
  // seal_drawer_hooks_by_probe 头注释）----
  // 旧「深压穿透证据 = 实测杆位 ≥ grasp+0.0015」物理上不可达：把手是贴前脸
  // 的空心立板（rigid），杆端压东侧立板面后无法再向前；run1/run2 两侧杆
  // ~0.0047-0.006 处 ~68N 顶死即物理「贴住」早成立，但杆位永远到不了
  // 0.0115。右杆偶发滑过立板缘进自由区（0.023，假阳性）。→ 啮合证据改用
  // 实测关节 effort（压贴时高、悬空/滑脱时崩落）+ 一个伸出底。ref 与判据按
  // cap2 实测封定于此（press_overshoot 与 press_force_floor 全部 > 0 才启用
  // seal_drawer_hooks_by_probe；默认 0 = 旧到达路径，其他抽屉/场景零回归）：
  double gripper_hook_press_overshoot{0.0};  // 压贴 ref = grasp + 此量（0.015）
  double gripper_hook_press_force_floor{0.0}; // 压贴证据：gripper 关节实测
                                              // effort ≥ 此量(N) 持续 1s
  bool has_unlock_role{false};
  std::string unlock_joint;
  std::string unlock_contact_link;
  tf2::Vector3 unlock_contact_point_local{0.0, 0.0, 0.0};
  tf2::Vector3 unlock_contact_point_base_at_zero{0.0, 0.0, 0.0};
};

// Which of the two drawer side tools a pose belongs to.  Every drawer-side
// target is computed against that side's tool profile (business-point offset,
// contact link, calibration joints and transport target).
enum class DrawerSide
{
  LEFT,
  RIGHT
};

// 2026-09-05 AGENT doc §4.2 (P1): 单次抽屉任务内的钩爪座封保持参考。
// 只在 execute_operate 的 drawer 分支声明（Action 任务级局部变量），不做
// 成员/全局缓存 —— 每次 open/close 任务独立计算。seal_drawer_hooks_by_probe
// 双侧原位座封成功后填入 left/right_joint_ref（= home_joint + 有符号投影
// extension，clamp 到杆行程 [0, ceiling]），valid 置 true。随后 support/
// unlock/attach/pull 阶段发送电缸全关节目标时必须显式携带（经
// drawer_rod_stage_desired 的 gripper_position_override 传 seat ref），防止
// 静态 gripper_grasp_position 把钩爪命令从座封位拽开。unlock/attach/pull
// 不发 hook 目标、不重算，保持 ref 自然贯穿（doc §4.4 保持矩阵）。
// valid=false 表示未做原位接管（legacy 到达路径 / physics 不可用回退）：
// 下游按旧静态 grasp 语义，零回归。
struct DrawerHookHoldState
{
  double left_joint_ref{0.0};
  double right_joint_ref{0.0};
  bool valid{false};
};

struct ButtonSnapshot
{
  bool received{false};
  bool structured_received{false};
  bool pressed_received{false};
  bool pressed{false};
  std::uint64_t pressed_transition_sequence{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double peak_position{0.0};
  bool valid{false};
  bool in_motion{false};
  std::string state_id;
  std::chrono::steady_clock::time_point received_at{};
  std::chrono::steady_clock::time_point structured_received_at{};
  std::chrono::steady_clock::time_point pressed_received_at{};
};

struct ButtonRuntime
{
  mutable std::mutex mutex;
  std::condition_variable condition;
  ButtonSnapshot state;
};

struct ButtonSpec
{
  std::uint8_t control_type{
    xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON};
  std::string id;
  std::string display_name;
  std::string joint_name;
  std::string joint_state_topic;
  std::string pressed_topic;
  std::string state_topic;
  std::string unit;
  std::uint8_t supported_commands{0};
  bool requires_grasp{false};
  double grasp_outward_offset{0.0};
  // Physical execution is granted only by the explicit robot-adapter
  // operable_control_ids allowlist populated during configure_controls().
  bool operable{false};
  // Keep the two policy dimensions explicit in the catalog.  ``operable`` is
  // their conjunction; clients use these fields to explain a mismatch before
  // they request navigation or motion.
  bool adapter_validated{false};
  bool toolset_compatible{false};
  std::string required_toolset;
  std::string unavailable_reason;
  std::optional<ControlNavigationStation> navigation_station;
  std::string parent_control_id;
  double local_x{0.0};
  double local_y{0.0};
  double local_z{0.0};
  double pivot_x{0.0};
  double pivot_y{0.0};
  double pivot_z{0.0};
  tf2::Vector3 axis{0.0, 0.0, -1.0};
  tf2::Vector3 approach_normal{0.0, 0.0, 1.0};
  double min_position{0.0};
  double max_position{0.008};
  double spring_stiffness{0.0};
  double press_threshold{0.0};
  double default_force{0.0};
  std::vector<std::string> state_ids;
  std::vector<std::string> state_labels;
  std::vector<double> state_positions;
  // Optional fixed roll about the contact tool's local +Z axis.  Buttons can
  // use it to keep the passive sibling tool and forearm away from the panel
  // while the measured off-axis finger remains exactly on the target.
  double tool_roll_offset{0.0};
  // Row-major [source detent][target detent] rotation about the contact
  // tool's local +Z axis.  It reorients passive sibling tools in the cabinet
  // plane without changing the calibrated contact direction.
  std::vector<double> tool_roll_offsets;
  // Over-center controls may be released after safely crossing the next
  // detent midpoint so their physical spring finishes the motion.
  double detent_release_fraction{1.0};
  // Slider close position: pushing the open panel PAST its detent to this
  // position trips the plugin's push-push release, and the detent spring then
  // returns the panel to the closed detent.  The robot can only push a slider
  // face (no fixed grasp / no pull), so closing is a second push, not a drag.
  double slider_release_position{std::numeric_limits<double>::quiet_NaN()};
  // Open press compensation: the tool's effective flat-panel contact extends
  // ~0.025-0.031 m past the calibrated business point (varying with press
  // impulse), so a press aimed at the open detent physically carries the panel
  // further.  On the OPEN push only, subtract this offset from the press
  // target so the panel lands AT the open detent (peak <= ~0.312) instead of
  // riding up against the push-push release (0.34).
  double slider_open_press_offset{0.0};
  // Close press compensation (additive): the plugin's release trips only when
  // raw_position >= slider_release_position, so commanding the close press at
  // EXACTLY the release position is a coin-flip -- the 80 kg panel settles
  // right on the threshold and reads ~0.3399x (float just below 0.34), the
  // release never trips, the weak open-detent spring (0.48 N at 0.34) cannot
  // drag the panel past the tool, and the close hangs until the 90 s settle
  // times out (boot18).  Add this offset so the close press target is strictly
  // ABOVE the release (release + offset), guaranteeing the panel crosses the
  // threshold mid-press; once the release trips the detent target flips to
  // closed and the ~4 N closed spring drags the panel (and the tool) back to
  // the closed detent (boots 14/17).
  double slider_close_press_offset{0.0};
  // Front-drawer operation: the robot docks on the panel's OUTWARD side and
  // grasps the panel front face (plugin fixed grasp joint) instead of pushing
  // the west push-push latch.  The operator then drags the panel linearly
  // along its slide axis -- pull to open, push to close.  graspable=true in
  // the scene SDF is the physical precondition; this flag switches the
  // execution branch from the push-push flow to the grasp-drag flow.
  bool grasp_operated{false};
  // ---- Bimanual pull-out drawer contract (TYPE_DRAWER) ----
  // All points live in the cabinet/local fixture frame (== world/map for a
  // unit-pose scene spawn) and are frozen by the P0 geometry contract.  The
  // drawer slides along drawer_axis (P0 measured +X = east, toward the robot).
  // 2026-09-03 AGENT §3: the right unlock MOTOR (unlock_motor_joint,
  // r_three_cyl_finger3_joint) extends so its rod engages the right-handle
  // LOGICAL unlock zone and releases the rail latch (no button press — b1p is a
  // fixed lamp); both tools then press their tips INTO the two handle plates
  // and the two arms pull/push the drawer in synchrony along the axis.  No
  // proximity-only unlock, no hovering "pull in empty air".
  bool drawer_bimanual{false};
  tf2::Vector3 drawer_axis{0.0, 0.0, 0.0};
  bool has_left_handle_point{false};
  tf2::Vector3 left_handle_point{0.0, 0.0, 0.0};
  bool has_right_handle_point{false};
  tf2::Vector3 right_handle_point{0.0, 0.0, 0.0};
  bool has_left_support_point{false};
  tf2::Vector3 left_support_point{0.0, 0.0, 0.0};
  bool has_right_support_point{false};
  tf2::Vector3 right_support_point{0.0, 0.0, 0.0};
  bool has_unlock_press_point{false};
  tf2::Vector3 unlock_press_point{0.0, 0.0, 0.0};
  // Per-drawer approach extension: how far each tool's business point rides
  // OUTWARD (along the drawer outward normal) from its handle when grasping.
  // Two- and three-cylinder effective contact reach differs, so the extension
  // is read from each drawer's contract rather than hard-coded globally.
  double drawer_grasp_outward_offset{0.020};
  // 2026-09-02: the grasp pose and the pull both press each tool tip INTO its
  // handle plate by this depth (west of the riser front face).  The plugin's
  // attach gate and coupling keepalive require both tips within
  // grasp_contact_threshold (~0.02 m) of their handles, so a firm press-in
  // replaces the old 5 cm hover — the tools genuinely contact the drawer.
  double drawer_grasp_press_depth{0.002};
  // The right tool first hovers this far OUTWARD (east) of the unlock-zone
  // centre (unlock_press_point), then drives IN by drawer_unlock_press_depth
  // to align its tip at / just west of the zone centre.  The zone sits 16 mm
  // proud of the panel face (x 0.115 vs 0.099), so the alignment tip never
  // touches the panel.  2026-09-03 AGENT §3: no visible button is pressed — the
  // latch is released by the unlock MOTOR extending (unlock_motor_joint).
  double drawer_unlock_approach_offset{0.040};
  double drawer_unlock_press_depth{0.003};
  // Right-tool distance from the unlock press point accepted by the plugin
  // latch release; mirrors the scene <unlock_distance_threshold>.
  double drawer_unlock_distance_threshold{0.030};
  // ---- 2026-09-03 AGENT §3: unlock motor contract ----
  // Unlock evidence is the P1-identified right unlock motor
  // (r_three_cyl_finger3_joint) ACTUALLY extending so its rod engages the
  // right-handle LOGICAL unlock zone (unlock_press_point = zone centre, world
  // (0.115, 4.693, 0.952)).  No b1p indicator joint is read, subscribed or
  // operated any more — b1p is a fixed lamp on the drawer front.  These values
  // mirror the scene <control> fields the Gazebo plugin gates on; the motor is
  // commanded to unlock_pressed_position and the REAL joint position is read
  // back (bounded steps, ceiling-guarded) before the unlock service is called.
  std::string unlock_motor_joint;          // e.g. r_three_cyl_finger3_joint
  double unlock_retracted_position{0.0};   // motor home (rod clear of the zone)
  double unlock_pressed_position{0.0};     // rod-in-zone engagement target (§7.2)
  double unlock_extension_floor{0.001};    // min extension the plugin counts
  double unlock_extension_ceiling{0.0254}; // hard travel cap — never exceed
  // Maximum left/right tool-tip translation mismatch tolerated during the
  // synchronized pull/push before the operation aborts and detaches.
  double drawer_sync_tolerance{0.050};
  double drawer_open_position{0.30};
  double drawer_closed_position{0.0};
  // Optional named MoveIt joint seed for the ready-pose IK.  A redundant arm
  // can otherwise reach the same pose through a branch that cannot continue
  // through the subsequent Cartesian manipulation.
  std::vector<std::string> ready_joint_seed_names;
  std::vector<double> ready_joint_seed_positions;
  std::shared_ptr<ButtonRuntime> runtime{std::make_shared<ButtonRuntime>()};
};

struct ControlStabilityReference
{
  const ButtonSpec * control{nullptr};
  std::string state_id;
  double position{0.0};
};

struct ResolvedControlGeometry
{
  tf2::Vector3 grasp_zero;
  tf2::Vector3 pivot;
  tf2::Vector3 axis;
  tf2::Vector3 approach_normal;
};

constexpr double clamp_button_press_depth(
  double requested_depth, double max_position) noexcept
{
  return requested_depth < max_position ? requested_depth : max_position;
}

constexpr double button_force_tracking_limit(
  double target_depth, double max_compensation,
  double max_position) noexcept
{
  const double bounded_target = clamp_button_press_depth(
    target_depth, max_position);
  const double remaining_travel = max_position - bounded_target;
  const double bounded_compensation =
    max_compensation < remaining_travel ?
    max_compensation : remaining_travel;
  return bounded_target + bounded_compensation;
}

static_assert(
  button_force_tracking_limit(0.008, 0.001, 0.008) == 0.008,
  "Force compensation must stop at a button's maximum travel.");
static_assert(
  button_force_tracking_limit(0.0075, 0.001, 0.008) == 0.008,
  "Force compensation must clamp partial remaining travel.");
static_assert(
  button_force_tracking_limit(0.006, 0.001, 0.008) == 0.007,
  "Force compensation below the limit must preserve its configured range.");

geometry_msgs::msg::Quaternion to_message(const tf2::Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

}  // namespace

std::optional<std::string> tool_profile_kinematics_validation_error(
  const moveit::core::RobotModelConstPtr & robot_model,
  std::vector<std::string> move_group_names)
{
  std::sort(move_group_names.begin(), move_group_names.end());
  move_group_names.erase(
    std::unique(move_group_names.begin(), move_group_names.end()),
    move_group_names.end());

  std::vector<std::string> errors;
  errors.reserve(move_group_names.size());
  for (const auto & move_group_name : move_group_names) {
    if (!robot_model) {
      errors.emplace_back(
        "MoveIt RobotModel is unavailable while validating tool profile "
        "JointModelGroup '" + move_group_name + "'.");
      continue;
    }

    const auto * joint_model_group =
      robot_model->getJointModelGroup(move_group_name);
    if (!joint_model_group) {
      errors.emplace_back(
        "The robot adapter tool profile references missing MoveIt "
        "JointModelGroup '" + move_group_name + "'.");
      continue;
    }
    if (!joint_model_group->getSolverInstance()) {
      errors.emplace_back(
        "MoveIt JointModelGroup '" + move_group_name +
        "' has no configured kinematics solver.");
    }
  }

  if (errors.empty()) {
    return std::nullopt;
  }

  std::ostringstream message;
  for (std::size_t index = 0; index < errors.size(); ++index) {
    if (index != 0U) {
      message << ' ';
    }
    message << errors[index];
  }
  return message.str();
}

class CabinetButtonOperator final : public rclcpp::Node
{
public:
  CabinetButtonOperator()
  : Node("xczs_cabinet_button_operator")
  {
    planning_frame_ = declare_parameter<std::string>(
      "planning_frame", "odom");
    navigation_frame_ = declare_parameter<std::string>(
      "navigation_frame", "map");
    // 柜体位姿 watchdog 的校验系：必须取夹具物理锚定系（与 pose authority 的
    // pose_parent_frame 合同一致）。odom 锚定的共享柜体沿用旧语义（odom→cabinet
    // 恒为常量，AMCL 修正不会污染）；map 锚定的夹具实例（电气夹层）若仍按 odom
    // 校验，AMCL 的常规 map→odom 重锚（机器人快速行走时可达 20mm+ 的单步修正）
    // 会被误判为柜体位移并中止任务 —— 定位修正不是柜体运动，map→cabinet 为静态
    // 恒等/测量真值，才是不变量。缺省空 = 沿用 planning_frame（旧行为）。
    cabinet_verify_frame_ = declare_parameter<std::string>(
      "cabinet_verify_frame", "");
    robot_model_name_ = required_string_parameter(
      "robot_model_name", "");
    move_group_namespace_ = declare_parameter<std::string>(
      "move_group_namespace", "/");
    // 2026-09-04 AGENT §8.4 fix#3 root-cause (v11): 真实关节状态话题，adapter
    // 契约字段 joint_state_topic（绝对名 /xczs/joint_states），launch 注入。
    // operator 直接订阅它作为 rod measured 的唯一信源——move_group 的
    // planning-scene current state 从未收到真实 joint_states（getCurrentState
    // 停留在启动初始位形：杆实测 ≈0 m 而物理杆已伸出数十毫米），seal 证据读
    // 它必败。缺省相对名 "joint_states"（经 rcl remap）时行为与旧路径一致。
    real_joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "joint_states");
    // 双臂 drawer 轨迹直发控制器（绕过 move_group 串行 execute 的并发路径）。
    // 控制器一律部署在 /xczs 命名空间（moveit_controllers 映射同名）。
    controller_namespace_ = declare_parameter<std::string>(
      "controller_namespace", "xczs");
    toolset_ = declare_parameter<std::string>("toolset", "A");
    if (toolset_ != "A" && toolset_ != "B") {
      throw std::invalid_argument(
              "Parameter 'toolset' must be 'A' or 'B'.");
    }
    load_tool_profiles();
    tool_tip_calibration_joint_tolerance_ = positive_parameter(
      "tool_tip_calibration_joint_tolerance", 0.001);
    tool_calibration_settle_timeout_ = positive_parameter(
      "tool_calibration_settle_timeout", 6.0);
    planning_time_ = positive_parameter("planning_time", 10.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 10);
    if (planning_attempts_ < 1 || planning_attempts_ > 100) {
      throw std::invalid_argument(
              "Parameter 'planning_attempts' must be in [1, 100].");
    }
    // 2026-09-03 AGENT §7.2: 现场分级封顶调试参数（0 = 全流程）。驱动脚本在
    // 两次任务间 ros2 param set debug_stage_cap N 即把下一次抽屉打开执行封到
    // 阶段 N（映射见 DebugStageCapReached 头注释；只读打开方向，closing 忽略）；
    // execute_operate 每次执行时重新读取，无需重启 operator。不改 msg/action
    // 合同，Web 无感知。
    debug_stage_cap_ = declare_parameter<int>("debug_stage_cap", 0);
    planning_velocity_scale_ = unit_interval_parameter(
      "planning_velocity_scale", 0.20);
    planning_acceleration_scale_ = unit_interval_parameter(
      "planning_acceleration_scale", 0.20);
    goal_position_tolerance_ = positive_parameter(
      "goal_position_tolerance", 0.005);
    goal_orientation_tolerance_ = positive_parameter(
      "goal_orientation_tolerance", 0.01);
    goal_joint_tolerance_ = positive_parameter(
      "goal_joint_tolerance", 0.001);
    allow_replanning_ = declare_parameter<bool>("allow_replanning", true);
    navigation_base_frame_ = required_string_parameter(
      "navigation_base_frame", "");
    const auto navigation_action = required_string_parameter(
      "navigation_action", "/navigate_to_pose");
    require_absolute_ros_name(
      navigation_action, "navigation_action", false);
    const auto navigation_mode_service = required_string_parameter(
      "navigation_mode_service", "/xczs/set_navigation_mode");
    require_absolute_ros_name(
      navigation_mode_service, "navigation_mode_service", true);
    allow_embedded_navigation_ = declare_parameter<bool>(
      "allow_embedded_navigation", true);
    docking_base_frame_ = required_string_parameter(
      "docking_base_frame", "");
    grasp_brake_link_ = required_string_parameter(
      "grasp_brake_link", "");
    const auto control_catalog_topic = declare_parameter<std::string>(
      "control_catalog_topic", kControlCatalogTopic);
    cabinet_frame_ = declare_parameter<std::string>(
      "cabinet_frame", "control_cabinet_frame");
    require_cabinet_pose_valid_ = declare_parameter<bool>(
      "require_cabinet_pose_valid", true);
    const auto cabinet_pose_valid_topic = required_string_parameter(
      "cabinet_pose_valid_topic", "pose_valid");
    cabinet_pose_translation_tolerance_ = positive_parameter(
      "cabinet_pose_translation_tolerance", 0.020);
    cabinet_pose_rotation_tolerance_ = positive_parameter(
      "cabinet_pose_rotation_tolerance", 0.035);
    // 2026-09-05 fix#cap2 (物理锚定): 夹具几何真值实体 + 其声明局部原点（都位于
    // cabinet 系）。latch 后若测得该实体在规划系(odom/world)的真位姿，就把
    // odom→cabinet 原点覆写为 (实测 − R·声明局部)，使抽屉轨/工作位姿直接落在
    // 物理板面真值上，与 AMCL 信念 δ / 遥移落地误差无关。无配置或测不到实体时
    // 保持既有 latch 行为（真机/无夹具实体场景零回归）。
    physics_anchor_entity_ = declare_parameter<std::string>(
      "physics_anchor_entity", "");
    {
      const auto flat_origin = declare_parameter<std::vector<double>>(
        "physics_anchor_local_origin", std::vector<double>{});
      if (flat_origin.size() == 3U &&
        std::all_of(
          flat_origin.begin(), flat_origin.end(),
          [](double value) {return std::isfinite(value);}))
      {
        physics_anchor_local_origin_ =
          tf2::Vector3(flat_origin[0], flat_origin[1], flat_origin[2]);
        physics_anchor_configured_ = true;
      } else if (!flat_origin.empty()) {
        throw std::invalid_argument(
                "Parameter 'physics_anchor_local_origin' must contain exactly "
                "three finite [x, y, z] values.");
      }
    }
    const auto grasp_service = declare_parameter<std::string>(
      "grasp_service", "grasp");
    const auto bimanual_grasp_service = declare_parameter<std::string>(
      "bimanual_grasp_service", "bimanual_grasp");
    const auto unlock_service = declare_parameter<std::string>(
      "unlock_service", "unlock");
    const auto reset_physics_service = declare_parameter<std::string>(
      "reset_physics_service", "reset_physics");
    // gazebo 实体位姿服务（物理真值测量，见 drawer_physics_link_point）。
    const auto drawer_entity_service = declare_parameter<std::string>(
      "drawer_entity_state_service", "/get_entity_state");
    const auto manual_cmd_vel_topic = required_string_parameter(
      "manual_cmd_vel_topic", "/xczs/manual_cmd_vel");
    require_absolute_ros_name(
      manual_cmd_vel_topic, "manual_cmd_vel_topic", false);
    const auto operation_lease_service = required_string_parameter(
      "operation_lease_service", "/xczs/operation_lease");
    operation_lease_duration_ = positive_parameter(
      "operation_lease_duration", 3.0);
    operation_lease_renew_period_ = positive_parameter(
      "operation_lease_renew_period", 0.75);
    operation_lease_request_timeout_ = positive_parameter(
      "operation_lease_request_timeout", 0.50);
    if (operation_lease_renew_period_ + operation_lease_request_timeout_ >=
      operation_lease_duration_)
    {
      throw std::invalid_argument(
              "The lease renew period plus request timeout must be shorter "
              "than the lease duration.");
    }

    prepress_distance_ = positive_parameter("prepress_distance", 0.060);
    grasp_outward_offset_ = positive_parameter(
      "grasp_outward_offset", 0.020);
    contact_clearance_ = positive_parameter("contact_clearance", 0.001);
    press_depth_ = positive_parameter("press_depth", 0.007);
    button_state_timeout_ = positive_parameter("state_timeout", 1.0);
    system_wait_timeout_ = positive_parameter("system_wait_timeout", 15.0);
    navigation_timeout_ = positive_parameter(
      "navigation_timeout", 120.0);
    navigation_takeover_distance_ = positive_parameter(
      "navigation_takeover_distance", 0.15);
    navigation_takeover_yaw_tolerance_ = positive_parameter(
      "navigation_takeover_yaw_tolerance", 0.35);
    navigation_takeover_stall_timeout_ = positive_parameter(
      "navigation_takeover_stall_timeout", 4.0);
    docking_timeout_ = positive_parameter("docking_timeout", 45.0);
    docking_position_tolerance_ = positive_parameter(
      "docking_position_tolerance", 0.015);
    docking_yaw_tolerance_ = positive_parameter(
      "docking_yaw_tolerance", 0.10);
    const auto flat_docking_base_footprint =
      declare_parameter<std::vector<double>>(
      "docking_base_footprint", std::vector<double>{});
    if ((!flat_docking_base_footprint.empty() &&
      flat_docking_base_footprint.size() < 6U) ||
      flat_docking_base_footprint.size() % 2U != 0U ||
      std::any_of(
        flat_docking_base_footprint.begin(),
        flat_docking_base_footprint.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Parameter 'docking_base_footprint' must contain at least "
              "three finite [x, y] points in the navigation-base frame.");
    }
    for (std::size_t index = 0U;
      index < flat_docking_base_footprint.size(); index += 2U)
    {
      docking_base_footprint_.push_back(
        {flat_docking_base_footprint[index],
          flat_docking_base_footprint[index + 1U]});
    }
    docking_base_footprint_padding_ = positive_parameter(
      "docking_base_footprint_padding", 0.03);
    docking_max_linear_speed_ = positive_parameter(
      "docking_max_linear_speed", 0.15);
    docking_max_angular_speed_ = positive_parameter(
      "docking_max_angular_speed", 0.45);
    docking_linear_gain_ = positive_parameter(
      "docking_linear_gain", 0.8);
    docking_angular_gain_ = positive_parameter(
      "docking_angular_gain", 1.2);
    // P3-8 grab-and-drive drawer pull: the base translates along the drawer
    // axis at a speed proportional to the remaining drawer travel, capped at
    // drawer_base_drive_max_speed (kept slow so the 80 kg drawer tracks the
    // linear-drag coupling without overshoot).
    drawer_base_drive_max_speed_ = positive_parameter(
      "drawer_base_drive_max_speed", 0.05);
    drawer_base_drive_gain_ = positive_parameter(
      "drawer_base_drive_gain", 1.0);
    drawer_base_drive_timeout_ = positive_parameter(
      "drawer_base_drive_timeout", 60.0);
    const auto & parameter_overrides =
      get_node_parameters_interface()->get_parameter_overrides();
    const bool navigation_yaw_offset_overridden =
      parameter_overrides.count("navigation_velocity_yaw_offset") != 0U;
    const bool legacy_yaw_offset_overridden =
      parameter_overrides.count("base_link_yaw_offset") != 0U;
    const double configured_navigation_yaw_offset = finite_parameter(
      "navigation_velocity_yaw_offset", -1.57079632679);
    const double legacy_yaw_offset = finite_parameter(
      "base_link_yaw_offset", configured_navigation_yaw_offset);
    navigation_velocity_yaw_offset_ =
      !navigation_yaw_offset_overridden && legacy_yaw_offset_overridden ?
      legacy_yaw_offset : configured_navigation_yaw_offset;
    if (!navigation_yaw_offset_overridden && legacy_yaw_offset_overridden) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'base_link_yaw_offset' is deprecated; use "
        "'navigation_velocity_yaw_offset'.");
    }
    press_detection_timeout_ = positive_parameter(
      "press_detection_timeout", 3.0);
    release_detection_timeout_ = positive_parameter(
      "release_detection_timeout", 3.0);
    press_hold_seconds_ = positive_parameter("press_hold_seconds", 0.5);
    force_tracking_tolerance_ = positive_parameter(
      "force_tracking_tolerance", 0.00005);
    force_tracking_max_compensation_ = positive_parameter(
      "force_tracking_max_compensation", 0.0010);
    force_tracking_settle_seconds_ = positive_parameter(
      "force_tracking_settle_seconds", 0.15);
    force_tracking_attempts_ = declare_parameter<int>(
      "force_tracking_attempts", 3);
    if (force_tracking_attempts_ < 1 || force_tracking_attempts_ > 5) {
      throw std::invalid_argument(
              "Parameter 'force_tracking_attempts' must be in [1, 5].");
    }
    button_press_minimum_cartesian_fraction_ = unit_interval_parameter(
      "button_press_minimum_cartesian_fraction", 0.95);
    if (button_press_minimum_cartesian_fraction_ < 0.90 ||
      button_press_minimum_cartesian_fraction_ > 0.99)
    {
      throw std::invalid_argument(
              "Parameter 'button_press_minimum_cartesian_fraction' must be "
              "in [0.90, 0.99].");
    }
    target_tolerance_ = positive_parameter("target_tolerance", 0.035);
    // A slider's panel settles against friction, so its detent verification
    // tolerates a small rest offset as long as the state has already flipped
    // (the state check is the primary signal; this is a secondary bound).
    slider_position_tolerance_ = positive_parameter(
      "slider_position_tolerance", 0.03);
    // AGENT §7.2 (cap7 run2 fix): 封顶拉距的紧取证带。cap7 拉距 0.03 恰好落在
    // slider_position_tolerance (0.03) 内 —— 曾导致拉取被跳过 + 位置判据空转
    // 通过的假 PASS。此容差与拉距同量级：必须明显小于最短封顶拉距 0.03
    // （否则 0.03 拉距又可被静止位置满足），又须容纳耦合真实落点偏差。
    debug_capped_pull_tolerance_ = positive_parameter(
      "debug_capped_pull_tolerance", 0.010);
    if (debug_capped_pull_tolerance_ < 0.002 ||
      debug_capped_pull_tolerance_ > 0.02)
    {
      throw std::invalid_argument(
              "Parameter 'debug_capped_pull_tolerance' must be in "
              "[0.002, 0.02].");
    }
    stable_velocity_tolerance_ = positive_parameter(
      "stable_velocity_tolerance", 0.03);
    stable_state_duration_ = positive_parameter(
      "stable_state_duration", 0.30);
    drawer_support_contact_tolerance_ = positive_parameter(
      "drawer_support_contact_tolerance", 0.008);
    drawer_gripper_contact_tolerance_ = positive_parameter(
      "drawer_gripper_contact_tolerance", 0.008);
    drawer_arm_pose_tolerance_ = positive_parameter(
      "drawer_arm_pose_tolerance", 0.008);
    drawer_arm_orientation_tolerance_ = positive_parameter(
      "drawer_arm_orientation_tolerance", 0.02);
    drawer_arm_stability_tolerance_ = positive_parameter(
      "drawer_arm_stability_tolerance", 0.003);
    if (drawer_support_contact_tolerance_ > 0.02 ||
      drawer_gripper_contact_tolerance_ > 0.02 ||
      drawer_arm_pose_tolerance_ > 0.02 ||
      drawer_arm_orientation_tolerance_ > 0.05 ||
      drawer_arm_stability_tolerance_ > 0.01)
    {
      throw std::invalid_argument(
              "Drawer contact/arm verification tolerances exceed their "
              "safety bounds.");
    }
    grasp_attach_settle_duration_ = positive_parameter(
      "grasp_attach_settle_duration", 0.15);
    grasp_release_settle_duration_ = positive_parameter(
      "grasp_release_settle_duration", 0.30);
    door_release_fraction_ = declare_parameter<double>(
      "door_release_fraction", 0.60);
    if (!std::isfinite(door_release_fraction_) ||
      door_release_fraction_ <= 0.5 || door_release_fraction_ >= 0.75)
    {
      throw std::invalid_argument(
              "Parameter 'door_release_fraction' must be in (0.5, 0.75).");
    }
    door_settle_timeout_ = positive_parameter("door_settle_timeout", 90.0);
    door_release_position_timeout_ = positive_parameter(
      "door_release_position_timeout", 10.0);
    door_detent_hysteresis_ = positive_parameter(
      "door_detent_hysteresis", 0.02);
    door_release_position_margin_ = positive_parameter(
      "door_release_position_margin", 0.01);
    rotary_pregrasp_clearance_ = positive_parameter(
      "rotary_pregrasp_clearance", 0.010);
    if (rotary_pregrasp_clearance_ > 0.015) {
      throw std::invalid_argument(
              "Parameter 'rotary_pregrasp_clearance' must be no greater "
              "than 0.015 m.");
    }
    // Knob blades sit inside the cabinet face plane while the tool clamps
    // them, so the shared door clearance (<= 15 mm) would put the unconstrained
    // OMPL ready->pregrasp path within jaw reach of the blade and actuate it
    // before grasp.  Knobs instead stop far enough out that the whole tool
    // stays beyond the blade near face during the pose-planned pregrasp, then
    // commit the final straight push as a short validated Cartesian approach.
    // 0.050 m gives a 50 mm final approach (CLEAR on every IK branch) with a
    // >50 mm blade gap; the OMPL pregrasp plan cannot physically clip it.
    knob_pregrasp_clearance_ = positive_parameter(
      "knob_pregrasp_clearance", 0.050);
    if (knob_pregrasp_clearance_ < 0.030 ||
      knob_pregrasp_clearance_ > 0.054)
    {
      throw std::invalid_argument(
              "Parameter 'knob_pregrasp_clearance' must be in "
              "[0.030, 0.054] m.");
    }
    door_release_clearance_ = positive_parameter(
      "door_release_clearance", 0.30);
    if (door_release_clearance_ < 0.28 ||
      door_release_clearance_ > 0.35)
    {
      throw std::invalid_argument(
              "Parameter 'door_release_clearance' must be in "
              "[0.28, 0.35] m.");
    }
    planning_scene_settle_seconds_ = positive_parameter(
      "planning_scene_settle_seconds", 0.50);
    rotation_waypoint_step_ = positive_parameter(
      "rotation_waypoint_step", 0.03490658504);
    drawer_waypoint_step_ = positive_parameter(
      "drawer_waypoint_step", 0.03);
    cartesian_velocity_scale_ = unit_interval_parameter(
      "cartesian_velocity_scale", 0.08);
    cartesian_acceleration_scale_ = unit_interval_parameter(
      "cartesian_acceleration_scale", 0.08);
    cartesian_jump_threshold_ = positive_parameter(
      "cartesian_jump_threshold", 2.0);
    if (cartesian_jump_threshold_ < 1.0 ||
      cartesian_jump_threshold_ > 10.0)
    {
      throw std::invalid_argument(
              "Parameter 'cartesian_jump_threshold' must be in [1, 10].");
    }
    cartesian_planning_attempts_ = declare_parameter<int>(
      "cartesian_planning_attempts", 5);
    if (cartesian_planning_attempts_ < 1 || cartesian_planning_attempts_ > 10) {
      throw std::invalid_argument(
              "Parameter 'cartesian_planning_attempts' must be in [1, 10].");
    }
    door_cartesian_segment_waypoints_ = declare_parameter<int>(
      "door_cartesian_segment_waypoints", 2);
    if (door_cartesian_segment_waypoints_ != 2) {
      throw std::invalid_argument(
              "Parameter 'door_cartesian_segment_waypoints' must be 2.");
    }
    motion_planning_attempts_ = declare_parameter<int>(
      "motion_planning_attempts", 3);
    if (motion_planning_attempts_ < 2 || motion_planning_attempts_ > 10) {
      throw std::invalid_argument(
              "Parameter 'motion_planning_attempts' must be in "
              "[2, 10].");
    }

    configure_controls();
    subscribe_to_button_states();
    subscribe_to_real_joint_states();
    control_catalog_publisher_ =
      create_publisher<
      xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>(
      control_catalog_topic,
      rclcpp::QoS(1).reliable().transient_local());
    active_control_publisher_ = create_publisher<std_msgs::msg::String>(
      kActiveControlTopic,
      rclcpp::QoS(1).reliable().transient_local());
    operation_heartbeat_publisher_ =
      create_publisher<std_msgs::msg::String>(
      kOperationHeartbeatTopic, rclcpp::QoS(1).reliable());
    operation_fault_subscription_ =
      create_subscription<std_msgs::msg::String>(
      kOperationFaultTopic, rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (message->data.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring a cabinet physics watchdog fault without a lease ID.");
          return;
        }
        mark_operation_lease_lost_for_exact_lease(
          "The cabinet physics watchdog reported this operation lease as "
          "abandoned.", message->data);
      });

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, navigation_action);
    // 双臂 drawer 的关节轨迹直发各自 follow_joint_trajectory 控制器，让两条
    // 执行真正并行（move_group 的 execute_trajectory action 串行处理目标，
    // 会锁死先动的那条臂——抽屉被另一条臂焊接固定，物理上无法前进）。
    left_fjt_client_ = rclcpp_action::create_client<
      control_msgs::action::FollowJointTrajectory>(
      this, controller_namespace_ + "/left_arm_controller/follow_joint_trajectory");
    right_fjt_client_ = rclcpp_action::create_client<
      control_msgs::action::FollowJointTrajectory>(
      this, controller_namespace_ + "/right_arm_controller/follow_joint_trajectory");
    // 2026-09-03 AGENT §3: right unlock motor (finger3) lives on the
    // three_cylinder controller; the unlock extension is driven here directly
    // (drive_unlock_motor), bypassing MoveIt so the intended contact is not
    // collision-checked away.
    unlock_motor_fjt_client_ = rclcpp_action::create_client<
      control_msgs::action::FollowJointTrajectory>(
      this, controller_namespace_ + "/three_cylinder_controller/follow_joint_trajectory");
    // 2026-09-03 AGENT §4.2: the LEFT support/gripper rods live on the
    // two_cylinder controller (l_two_cyl_finger{1,2}_joint); their extension is
    // driven through this direct full-joint FJT client, symmetric with the
    // right three_cylinder unlock/support/gripper client above.
    two_cylinder_fjt_client_ = rclcpp_action::create_client<
      control_msgs::action::FollowJointTrajectory>(
      this, controller_namespace_ + "/two_cylinder_controller/follow_joint_trajectory");
    // 2026-09-05 AGENT §8.4 fix#4 (hook seat release): controller_manager
    // service clients used to discharge the rod drives' effort-PID integrator
    // charge (deactivate -> configure -> activate the tool controllers; see
    // seal_drawer_hooks_by_probe phase 3).  A dedicated reentrant group keeps
    // the reset callable from the action worker thread without serializing
    // against the node's default callbacks.
    rod_controller_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    controller_switch_client_ = create_client<
      controller_manager_msgs::srv::SwitchController>(
      controller_namespace_ + "/controller_manager/switch_controller",
      rmw_qos_profile_services_default, rod_controller_callback_group_);
    controller_configure_client_ = create_client<
      controller_manager_msgs::srv::ConfigureController>(
      controller_namespace_ + "/controller_manager/configure_controller",
      rmw_qos_profile_services_default, rod_controller_callback_group_);
    subscribe_to_arm_controller_references();
    navigation_mode_client_ = create_client<std_srvs::srv::SetBool>(
      navigation_mode_service);
    grasp_client_ =
      create_client<xczs_inspection_robot_interfaces::srv::SetCabinetGrasp>(
      grasp_service);
    bimanual_grasp_client_ = create_client<
      xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp>(
      bimanual_grasp_service);
    unlock_client_ =
      create_client<xczs_inspection_robot_interfaces::srv::SetCabinetUnlock>(
      unlock_service);
    reset_client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    reset_physics_client_ = create_client<std_srvs::srv::Trigger>(
      reset_physics_service, rmw_qos_profile_services_default,
      reset_client_callback_group_);
    drawer_entity_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    drawer_entity_client_ = create_client<gazebo_msgs::srv::GetEntityState>(
      drawer_entity_service, rmw_qos_profile_services_default,
      drawer_entity_callback_group_);
    operation_lease_client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    operation_lease_client_ = create_client<ManageOperationLease>(
      operation_lease_service, rmw_qos_profile_services_default,
      operation_lease_client_callback_group_);
    manual_base_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      manual_cmd_vel_topic, 10);
    transform_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*transform_buffer_);
    cabinet_pose_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      cabinet_pose_valid_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        cabinet_pose_valid_.store(message->data);
        cabinet_pose_valid_received_.store(true);
      });

    action_server_ = rclcpp_action::create_server<PressCabinetButton>(
      this,
      kActionName,
      [this](
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<const PressCabinetButton::Goal> goal)
      {
        return handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<PressGoalHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<PressGoalHandle> goal_handle) {
        handle_accepted(goal_handle);
      });
    operate_action_server_ =
      rclcpp_action::create_server<OperateCabinetControl>(
      this,
      kOperateActionName,
      [this](
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<const OperateCabinetControl::Goal> goal)
      {
        return handle_operate_goal(uuid, goal);
      },
      [this](const std::shared_ptr<OperateGoalHandle> goal_handle) {
        return handle_operate_cancel(goal_handle);
      },
      [this](const std::shared_ptr<OperateGoalHandle> goal_handle) {
        handle_operate_accepted(goal_handle);
      });
    // reset_controls 在 handler 内同步 wait_for_service/wait_for 可达数秒；
    // 放在独立的 Reentrant 组，避免阻塞默认组的 action server 与其它服务。
    reset_service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    reset_controls_service_ = create_service<std_srvs::srv::Trigger>(
      kResetControlsService,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        reset_controls(response);
      },
      rmw_qos_profile_services_default,
      reset_service_callback_group_);
    publish_control_catalog();
    publish_active_control("");

    RCLCPP_INFO(
      get_logger(),
      "Cabinet action %s controls %zu configured cabinet controls.",
      kActionName, buttons_in_order_.size());
  }

  ~CabinetButtonOperator() override
  {
    shutdown_requested_.store(true);
    stop_active_motion();
    cancel_active_navigation();
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
    }
    release_operation_lease_noexcept();
  }

private:
  enum class ActiveGoalType
  {
    NONE,
    PRESS,
    OPERATE,
  };

  enum class PendingGoalDisposition
  {
    EXECUTE,
    INVALID_CONTROL,
    INVALID_BUTTON,
    RESOURCE_BUSY,
  };

  static std::string goal_uuid_key(
    const rclcpp_action::GoalUUID & goal_id)
  {
    return std::string(
      reinterpret_cast<const char *>(goal_id.data()), goal_id.size());
  }

  void remember_pending_goal(
    const rclcpp_action::GoalUUID & goal_id,
    PendingGoalDisposition disposition)
  {
    std::lock_guard<std::mutex> lock(pending_goal_mutex_);
    pending_goal_dispositions_[goal_uuid_key(goal_id)] = disposition;
  }

  PendingGoalDisposition take_pending_goal(
    const rclcpp_action::GoalUUID & goal_id)
  {
    std::lock_guard<std::mutex> lock(pending_goal_mutex_);
    const auto key = goal_uuid_key(goal_id);
    const auto iterator = pending_goal_dispositions_.find(key);
    if (iterator == pending_goal_dispositions_.end()) {
      return PendingGoalDisposition::INVALID_CONTROL;
    }
    const auto disposition = iterator->second;
    pending_goal_dispositions_.erase(iterator);
    return disposition;
  }

  std::string required_string_parameter(
    const std::string & name,
    const std::string & default_value)
  {
    const auto value = declare_parameter<std::string>(name, default_value);
    if (value.empty()) {
      throw std::invalid_argument("Parameter '" + name + "' must not be empty.");
    }
    return value;
  }

  ToolProfile read_tool_profile(const std::string & type_name)
  {
    const std::string prefix = "tool_profiles." + type_name + ".";
    ToolProfile profile;
    profile.move_group = declare_parameter<std::string>(
      prefix + "move_group", "");
    profile.contact_tool_link = declare_parameter<std::string>(
      prefix + "contact_tool_link", "");
    profile.grasp_link = declare_parameter<std::string>(
      prefix + "grasp_link", profile.contact_tool_link);
    const auto grasp_point_position = declare_parameter<std::vector<double>>(
      prefix + "grasp_point_position", {0.0, 0.0, 0.0});
    if (grasp_point_position.size() != 3U ||
      std::any_of(
        grasp_point_position.begin(), grasp_point_position.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Parameter '" + prefix +
              "grasp_point_position' must contain three finite values.");
    }
    profile.grasp_point_position = tf2::Vector3(
      grasp_point_position[0], grasp_point_position[1],
      grasp_point_position[2]);
    profile.transport_named_target = declare_parameter<std::string>(
      prefix + "transport_named_target", "");
    const auto tool_axis_orientation = declare_parameter<std::string>(
      prefix + "tool_axis_orientation", "along_outward");
    if (tool_axis_orientation == "along_outward") {
      profile.tool_axis_orientation =
        ToolProfile::ToolAxisOrientation::ALONG_OUTWARD;
    } else if (tool_axis_orientation == "toward_control") {
      profile.tool_axis_orientation =
        ToolProfile::ToolAxisOrientation::TOWARD_CONTROL;
    } else {
      throw std::invalid_argument(
              "Parameter '" + prefix + "tool_axis_orientation' must be "
              "'along_outward' or 'toward_control', got '" +
              tool_axis_orientation + "'.");
    }
    const auto tool_tip_position = declare_parameter<std::vector<double>>(
      prefix + "tool_tip_position", std::vector<double>{});
    const double legacy_tool_tip_offset = declare_parameter<double>(
      prefix + "tool_tip_offset", 0.0);
    if (tool_tip_position.empty()) {
      if (!std::isfinite(legacy_tool_tip_offset) ||
        legacy_tool_tip_offset < 0.0)
      {
        throw std::invalid_argument(
                "Parameter '" + prefix +
                "tool_tip_offset' must be finite and non-negative.");
      }
      profile.tool_tip_position = tf2::Vector3(
        0.0, 0.0, -legacy_tool_tip_offset);
    } else {
      if (tool_tip_position.size() != 3U ||
        std::any_of(
          tool_tip_position.begin(), tool_tip_position.end(),
          [](double value) {return !std::isfinite(value);}))
      {
        throw std::invalid_argument(
                "Parameter '" + prefix +
                "tool_tip_position' must contain three finite values.");
      }
      if (std::abs(legacy_tool_tip_offset) > 1.0e-12) {
        throw std::invalid_argument(
                "Tool profile '" + type_name +
                "' must not declare both tool_tip_position and the legacy "
                "non-zero tool_tip_offset.");
      }
      profile.tool_tip_position = tf2::Vector3(
        tool_tip_position[0], tool_tip_position[1], tool_tip_position[2]);
    }
    profile.calibration_joint_names =
      declare_parameter<std::vector<std::string>>(
      prefix + "calibration_joint_names", std::vector<std::string>{});
    profile.calibration_joint_positions =
      declare_parameter<std::vector<double>>(
      prefix + "calibration_joint_positions", std::vector<double>{});
    const std::unordered_set<std::string> unique_calibration_joints(
      profile.calibration_joint_names.begin(),
      profile.calibration_joint_names.end());
    if (profile.calibration_joint_names.size() !=
      profile.calibration_joint_positions.size() ||
      unique_calibration_joints.size() !=
      profile.calibration_joint_names.size() ||
      std::any_of(
        profile.calibration_joint_names.begin(),
        profile.calibration_joint_names.end(),
        [](const std::string & value) {return value.empty();}) ||
      std::any_of(
        profile.calibration_joint_positions.begin(),
        profile.calibration_joint_positions.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Tool profile '" + type_name +
              "' calibration joint names/positions must be finite, unique "
              "and have equal length.");
    }
    return profile;
  }

  BimanualToolProfile read_bimanual_tool_profile(const std::string & side)
  {
    const std::string prefix = "drawer_tools." + side + ".";
    BimanualToolProfile profile;
    profile.move_group = declare_parameter<std::string>(
      prefix + "move_group", "");
    profile.contact_tool_link = declare_parameter<std::string>(
      prefix + "contact_tool_link", "");
    const auto tool_tip_position = declare_parameter<std::vector<double>>(
      prefix + "tool_tip_position", std::vector<double>{0.0, 0.0, 0.0});
    if (tool_tip_position.size() != 3U ||
      std::any_of(
        tool_tip_position.begin(), tool_tip_position.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Parameter '" + prefix +
              "tool_tip_position' must contain three finite values.");
    }
    profile.tool_tip_position = tf2::Vector3(
      tool_tip_position[0], tool_tip_position[1], tool_tip_position[2]);
    profile.calibration_joint_names =
      declare_parameter<std::vector<std::string>>(
      prefix + "calibration_joint_names", std::vector<std::string>{});
    profile.calibration_joint_positions =
      declare_parameter<std::vector<double>>(
      prefix + "calibration_joint_positions", std::vector<double>{});
    const std::unordered_set<std::string> unique_calibration_joints(
      profile.calibration_joint_names.begin(),
      profile.calibration_joint_names.end());
    if (profile.calibration_joint_names.size() !=
      profile.calibration_joint_positions.size() ||
      unique_calibration_joints.size() !=
      profile.calibration_joint_names.size() ||
      std::any_of(
        profile.calibration_joint_names.begin(),
        profile.calibration_joint_names.end(),
        [](const std::string & value) {return value.empty();}) ||
      std::any_of(
        profile.calibration_joint_positions.begin(),
        profile.calibration_joint_positions.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Drawer tool '" + side +
              "' calibration joint names/positions must be finite, unique "
              "and have equal length.");
    }
    profile.transport_named_target = declare_parameter<std::string>(
      prefix + "transport_named_target", profile.transport_named_target);
    if (profile.transport_named_target.empty()) {
      throw std::invalid_argument(
              "Drawer tool '" + side +
              "' transport_named_target must be non-empty.");
    }
    profile.tool_roll_offset = declare_parameter<double>(
      prefix + "tool_roll_offset", 0.0);
    if (!std::isfinite(profile.tool_roll_offset)) {
      throw std::invalid_argument(
              "Drawer tool '" + side +
              "' tool_roll_offset must be finite.");
    }

    // ---- 2026-09-03 AGENT §4.2: per-side rod ROLE fields.  A role is enabled
    // only when its joint, contact link and a real (z<0) rod-end local offset
    // are all declared AND its moving joint is one of the side's calibrated
    // joints; the contact/grasp WORK position additionally gates the stage
    // (see drive_drawer_support_stage / drive_drawer_hook_stage).  This is
    // the read-side half of the drawer_tools role contract in the adapter.
    auto parse_role = [&] (
      std::string & joint_out, std::string & link_out,
      tf2::Vector3 & local_out, tf2::Vector3 & base_zero_out,
      const std::string & role_name) {
      const std::string role_prefix = prefix + role_name;
      const std::string joint = declare_parameter<std::string>(
        role_prefix + "_joint", "");
      const std::string contact_link = declare_parameter<std::string>(
        role_prefix + "_contact_link", "");
      const auto local_point = declare_parameter<std::vector<double>>(
        role_prefix + "_contact_point_local",
        std::vector<double>{0.0, 0.0, 0.0});
      const auto base_zero_point = declare_parameter<std::vector<double>>(
        role_prefix + "_contact_point_base_at_zero",
        std::vector<double>{});
      if (joint.empty() && contact_link.empty()) {
        return false;  // 角色未声明：保持 has_*_role=false（旧行为）。
      }
      const auto in_calibration = std::find(
        profile.calibration_joint_names.begin(),
        profile.calibration_joint_names.end(), joint);
      if (joint.empty() || contact_link.empty() ||
        local_point.size() != 3U ||
        base_zero_point.size() != 3U ||
        std::any_of(
          local_point.begin(), local_point.end(),
          [](double value) {return !std::isfinite(value);}) ||
        std::any_of(
          base_zero_point.begin(), base_zero_point.end(),
          [](double value) {return !std::isfinite(value);}) ||
        !(local_point[2] < 0.0) ||
        in_calibration == profile.calibration_joint_names.end())
      {
        throw std::invalid_argument(
                "Drawer tool '" + side + "' " + role_name +
                " role requires a non-empty moving joint present in "
                "calibration_joint_names, a non-empty contact link and a "
                "finite rod-end local point with negative z, plus a finite "
                "contact_point_base_at_zero (rod motion is along the tool "
                "-Z axis).");
      }
      joint_out = joint;
      link_out = contact_link;
      local_out = tf2::Vector3(
        local_point[0], local_point[1], local_point[2]);
      base_zero_out = tf2::Vector3(
        base_zero_point[0], base_zero_point[1], base_zero_point[2]);
      return true;
    };
    profile.has_support_role = parse_role(
      profile.support_joint, profile.support_contact_link,
      profile.support_contact_point_local,
      profile.support_contact_point_base_at_zero, "support");
    profile.support_retracted_position = declare_parameter<double>(
      prefix + "support_retracted_position", 0.0);
    profile.support_contact_position = declare_parameter<double>(
      prefix + "support_contact_position", 0.0);
    profile.support_contact_inset = declare_parameter<double>(
      prefix + "support_contact_inset", 0.0);
    profile.has_gripper_role = parse_role(
      profile.gripper_joint, profile.gripper_contact_link,
      profile.gripper_contact_point_local,
      profile.gripper_contact_point_base_at_zero, "gripper");
    profile.gripper_open_position = declare_parameter<double>(
      prefix + "gripper_open_position", 0.0);
    profile.gripper_grasp_position = declare_parameter<double>(
      prefix + "gripper_grasp_position", 0.0);
    // 2026-09-04 AGENT §8.4 fix #3 v2 封定探针参数（见结构体注释）。默认 0 =
    // 旧到达路径：只对写入 adapter.yaml 的抽屉（db1）启用压贴+保持探针。
    profile.gripper_hook_press_overshoot = declare_parameter<double>(
      prefix + "gripper_hook_press_overshoot", 0.0);
    profile.gripper_hook_press_force_floor = declare_parameter<double>(
      prefix + "gripper_hook_press_force_floor", 0.0);
    if (!std::isfinite(profile.gripper_hook_press_overshoot) ||
      !std::isfinite(profile.gripper_hook_press_force_floor))
    {
      throw std::invalid_argument(
              "Drawer tool '" + side +
              "' gripper_hook_* probe parameters must be finite.");
    }
    // cap2 几何根因 fix#3 v3: 工作位姿 aim 补偿（见结构体注释）。默认 [0,0,0]。
    const auto drawer_pose_aim = declare_parameter<std::vector<double>>(
      prefix + "drawer_pose_aim_offset",
      std::vector<double>{0.0, 0.0, 0.0});
    if (drawer_pose_aim.size() != 3U ||
      !std::all_of(
        drawer_pose_aim.begin(), drawer_pose_aim.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Drawer tool '" + side +
              "' drawer_pose_aim_offset must be a finite 3-vector.");
    }
    profile.drawer_pose_aim_offset = tf2::Vector3(
      drawer_pose_aim[0], drawer_pose_aim[1], drawer_pose_aim[2]);
    profile.has_unlock_role = parse_role(
      profile.unlock_joint, profile.unlock_contact_link,
      profile.unlock_contact_point_local,
      profile.unlock_contact_point_base_at_zero, "unlock");
    return profile;
  }

  void require_drawer_tools_configured() const
  {
    if (!drawer_tools_configured_ ||
      drawer_left_tool_.move_group.empty() ||
      drawer_right_tool_.move_group.empty() ||
      drawer_left_tool_.contact_tool_link.empty() ||
      drawer_right_tool_.contact_tool_link.empty())
    {
      throw std::invalid_argument(
              "Bimanual drawer control requires 'drawer_tools.left' and "
              "'drawer_tools.right' tool profiles (move_group and "
              "contact_tool_link each).");
    }
  }

  void load_tool_profiles()
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    tool_profiles_[Control::TYPE_BUTTON] = read_tool_profile("button");
    tool_profiles_[Control::TYPE_KNOB] = read_tool_profile("knob");
    tool_profiles_[Control::TYPE_SWITCH] = read_tool_profile("switch");
    tool_profiles_[Control::TYPE_DOOR] = read_tool_profile("door");
    // A slider is driven with the same three-cylinder press tool as a button:
    // the tool tip pushes the panel face along the prismatic axis.  Copy the
    // already-declared button profile (read_tool_profile declares parameters,
    // so calling it again for the same type would throw
    // ParameterAlreadyDeclaredException).
    tool_profiles_[Control::TYPE_SLIDER] = tool_profiles_[Control::TYPE_BUTTON];

    // A legacy single-arm adapter declares one scalar move_group / tip link /
    // contact tool link / transport target; apply it to every control type so
    // historical adapter files keep loading unchanged.
    if (tool_profiles_[Control::TYPE_BUTTON].move_group.empty()) {
      ToolProfile legacy;
      legacy.move_group = required_string_parameter("move_group", "");
      legacy.contact_tool_link =
        required_string_parameter("contact_tool_link", "");
      legacy.transport_named_target =
        required_string_parameter("transport_named_target", "");
      for (auto & entry : tool_profiles_) {
        entry.second = legacy;
      }
    } else {
      for (const auto & entry : tool_profiles_) {
        const auto & profile = entry.second;
        if (profile.move_group.empty() ||
          profile.contact_tool_link.empty() ||
          profile.transport_named_target.empty())
        {
          throw std::invalid_argument(
                  "Every tool_profiles entry must declare move_group, "
                  "contact_tool_link and transport_named_target.");
        }
      }
    }
    apply_tool_profile(Control::TYPE_BUTTON);

    // Bimanual drawer tool profiles are optional at the parameter level (a
    // legacy adapter without drawers declares neither), but required by
    // require_drawer_tools_configured() the moment any drawer control exists.
    drawer_left_tool_ = read_bimanual_tool_profile("left");
    drawer_right_tool_ = read_bimanual_tool_profile("right");
    drawer_tools_configured_ =
      !drawer_left_tool_.move_group.empty() &&
      !drawer_right_tool_.move_group.empty() &&
      !drawer_left_tool_.contact_tool_link.empty() &&
      !drawer_right_tool_.contact_tool_link.empty();
  }

  void apply_tool_profile(std::uint8_t control_type)
  {
    const auto iterator = tool_profiles_.find(control_type);
    if (iterator == tool_profiles_.end()) {
      throw std::invalid_argument(
              "No tool profile is configured for control type " +
              std::to_string(static_cast<int>(control_type)) + ".");
    }
    const auto & profile = iterator->second;
    move_group_name_ = profile.move_group;
    contact_tool_link_ = profile.contact_tool_link;
    grasp_link_ = profile.grasp_link.empty() ?
      profile.contact_tool_link : profile.grasp_link;
    grasp_point_position_ = profile.grasp_point_position;
    transport_named_target_ = profile.transport_named_target;
    tool_axis_orientation_ = profile.tool_axis_orientation;
    tool_tip_position_ = profile.tool_tip_position;
    tool_tip_calibration_joint_names_ = profile.calibration_joint_names;
    tool_tip_calibration_joint_positions_ =
      profile.calibration_joint_positions;
  }

  // Bind the legacy single-arm members to the RIGHT drawer tool so the shared
  // transport / docking / calibration code drives the right arm for a drawer
  // operation.  The left arm is driven only through the bimanual helpers
  // (plan_and_execute_bimanual_poses / execute_bimanual_segmented_cartesian
  // _path), which always name their own group and contact link.  The tool-axis
  // orientation is intentionally left untouched: the bimanual drawer uses the
  // same ALONG_OUTWARD convention as the mounted Set-A right tool.
  void apply_drawer_tool_profiles()
  {
    require_drawer_tools_configured();
    move_group_name_ = drawer_right_tool_.move_group;
    contact_tool_link_ = drawer_right_tool_.contact_tool_link;
    grasp_link_ = drawer_right_tool_.contact_tool_link;
    transport_named_target_ = drawer_right_tool_.transport_named_target;
    tool_tip_position_ = drawer_right_tool_.tool_tip_position;
    tool_tip_calibration_joint_names_ =
      drawer_right_tool_.calibration_joint_names;
    tool_tip_calibration_joint_positions_ =
      drawer_right_tool_.calibration_joint_positions;
  }

  void verify_tool_tip_calibration_state(
    const moveit::core::RobotState & robot_state) const
  {
    for (std::size_t index = 0U;
      index < tool_tip_calibration_joint_names_.size(); ++index)
    {
      const auto & joint_name = tool_tip_calibration_joint_names_[index];
      double measured_position;
      try {
        measured_position = robot_state.getVariablePosition(joint_name);
      } catch (const std::exception & error) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Tool business-point calibration references unknown joint '" +
                joint_name + "': " + error.what());
      }
      const double expected_position =
        tool_tip_calibration_joint_positions_[index];
      if (!std::isfinite(measured_position) ||
        std::abs(measured_position - expected_position) >
        tool_tip_calibration_joint_tolerance_)
      {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Tool joint '" + joint_name + "' is at " +
                std::to_string(measured_position) +
                " rad/m, outside its calibrated business-point position " +
                std::to_string(expected_position) + " +/- " +
                std::to_string(tool_tip_calibration_joint_tolerance_) +
                "; arm motion was blocked. Reset the tool first.");
      }
    }
  }

  // 在 arm 运动前把工具标定关节（如三缸手指）主动合拢到标定业务位姿。
  // transport/导航/停靠只规划臂组，未受令的末端关节会自然回落到静息位
  // （约 3mm，恰在校验容差边界），导致 verify_tool_tip_calibration_state
  // 悬空失败。此函数在标定关节所在的 MoveIt 组上规划一段关节空间轨迹，
  // 把它们可靠带回标定位姿；已处于容差内时直接跳过，避免多余运动。
  // execute_motion_bounded 带宽限提前返回，真实轨迹可能仍在飞行中，因此
  // 返回前轮询标定关节直到实际回到业务位姿（或超时），并把合拢后的最新
  // 状态返回给调用方立即校验，避免校验读到 execute 之前的陈旧快照。
  template<typename GoalHandleT>
  moveit::core::RobotStatePtr ensure_calibration_position_for_arm(
    MoveGroupInterface & arm_move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<std::string> & calibration_joint_names,
    const std::vector<double> & calibration_joint_positions)
  {
    if (calibration_joint_names.empty()) {
      // 该工具 profile 无标定关节（knob/switch），无需合拢
      return synchronized_current_robot_state(arm_move_group);
    }
    const auto current_state =
      synchronized_current_robot_state(arm_move_group);
    bool already_calibrated = true;
    for (std::size_t index = 0U;
      index < calibration_joint_names.size(); ++index)
    {
      double measured;
      try {
        measured = current_state->getVariablePosition(
          calibration_joint_names[index]);
      } catch (const std::exception & error) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Tool calibration references unknown joint '" +
                calibration_joint_names[index] + "': " +
                error.what());
      }
      const double expected = calibration_joint_positions[index];
      if (!std::isfinite(measured) ||
        std::abs(measured - expected) > tool_tip_calibration_joint_tolerance_)
      {
        already_calibrated = false;
        break;
      }
    }
    if (already_calibrated) {
      return current_state;
    }
    const auto robot_model = arm_move_group.getRobotModel();
    std::string calibration_group;
    for (const auto * group : robot_model->getJointModelGroups()) {
      bool contains_all = true;
      for (const auto & joint_name : calibration_joint_names) {
        if (group->getJointModel(joint_name) == nullptr) {
          contains_all = false;
          break;
        }
      }
      if (contains_all) {
        calibration_group = group->getName();
        break;
      }
    }
    if (calibration_group.empty()) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Tool calibration joints are not contained in a single MoveIt "
              "joint-model group; cannot close the tool before arm motion.");
    }
    RCLCPP_INFO(
      get_logger(),
      "Closing tool calibration joints via MoveIt group '%s' to their "
      "calibrated business-point positions before arm motion.",
      calibration_group.c_str());
    check_cancel(goal_handle);
    MoveGroupInterface tool_group(
      shared_from_this(),
      MoveGroupInterface::Options(
        calibration_group, "robot_description", move_group_namespace_),
      transform_buffer_,
      rclcpp::Duration::from_seconds(system_wait_timeout_));
    tool_group.setPoseReferenceFrame(planning_frame_);
    tool_group.setPlanningTime(planning_time_);
    tool_group.setNumPlanningAttempts(planning_attempts_);
    tool_group.setMaxVelocityScalingFactor(planning_velocity_scale_);
    tool_group.setMaxAccelerationScalingFactor(planning_acceleration_scale_);
    tool_group.setGoalJointTolerance(goal_joint_tolerance_);
    tool_group.allowReplanning(allow_replanning_);
    tool_group.setJointValueTarget(
      calibration_joint_names,
      calibration_joint_positions);
    MoveGroupInterface::Plan plan;
    const auto planning_result = tool_group.plan(plan);
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt could not plan to close tool calibration joints on "
              "group '" + calibration_group + "' to their calibrated "
              "business-point positions.");
    }
    check_cancel(goal_handle);
    const auto close_code = execute_motion_bounded(
      [&tool_group, &plan]() { return tool_group.execute(plan); },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(30), "tool calibration joints");
    if (close_code != moveit::core::MoveItErrorCode::SUCCESS) {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt failed to execute the tool calibration joint-close "
              "trajectory on group '" + calibration_group + "'.");
    }
    // 合拢轨迹执行成功不等同于手指已实际到位：execute_motion_bounded 会
    // 提前返回，真实轨迹可能仍在飞行中。轮询标定关节直到它们真实回到
    // 业务位姿（或超时），期间关节状态话题持续刷新。到位后返回最新状态。
    const auto settle_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(tool_calibration_settle_timeout_);
    while (std::chrono::steady_clock::now() < settle_deadline) {
      check_cancel(goal_handle);
      const auto settled_state =
        synchronized_current_robot_state(arm_move_group);
      bool within_tolerance = true;
      for (std::size_t index = 0U;
        index < calibration_joint_names.size(); ++index)
      {
        double measured;
        try {
          measured = settled_state->getVariablePosition(
            calibration_joint_names[index]);
        } catch (const std::exception & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Tool calibration references unknown joint '" +
                  calibration_joint_names[index] + "': " +
                  error.what());
        }
        const double expected = calibration_joint_positions[index];
        if (!std::isfinite(measured) ||
          std::abs(measured - expected) > tool_tip_calibration_joint_tolerance_)
        {
          within_tolerance = false;
          break;
        }
      }
      if (within_tolerance) {
        return settled_state;
      }
      std::this_thread::sleep_for(50ms);
    }
    check_cancel(goal_handle);
    // 超时未到位：给出逐关节实测 vs 期望，便于排查控制器静差/质量等。
    std::string detail;
    const auto final_state = synchronized_current_robot_state(arm_move_group);
    for (std::size_t index = 0U;
      index < calibration_joint_names.size(); ++index)
    {
      double measured = std::numeric_limits<double>::quiet_NaN();
      try {
        measured = final_state->getVariablePosition(
          calibration_joint_names[index]);
      } catch (const std::exception &) {
      }
      const double expected = calibration_joint_positions[index];
      if (!detail.empty()) {
        detail += "; ";
      }
      detail += calibration_joint_names[index] + "=" +
        (std::isfinite(measured) ? std::to_string(measured) : "nan") +
        " (expected " + std::to_string(expected) + ")";
    }
    throw OperationError(
            PressCabinetButton::Result::NOT_READY,
            "Tool calibration joints did not settle to their calibrated "
            "business-point positions within " +
            std::to_string(tool_calibration_settle_timeout_) + " s after "
            "the joint-close trajectory: " + detail);
  }
  // 单臂调用点：以当前已应用的工具 profile 的标定关节执行合拢。
  template<typename GoalHandleT>
  moveit::core::RobotStatePtr ensure_tool_calibration_position(
    MoveGroupInterface & arm_move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle)
  {
    return ensure_calibration_position_for_arm(
      arm_move_group, goal_handle,
      tool_tip_calibration_joint_names_, tool_tip_calibration_joint_positions_);
  }

  // Close BOTH drawer tool profiles' calibration joints before any bimanual
  // arm motion.  The right arm reuses the single-arm path (the members are
  // bound to the right drawer profile); the left arm closes its own joints.
  // Returns the settled states in (left, right) order for immediate verify.
  template<typename GoalHandleT>
  std::pair<moveit::core::RobotStatePtr, moveit::core::RobotStatePtr>
  ensure_bimanual_calibration_position(
    MoveGroupInterface & left_group,
    MoveGroupInterface & right_group,
    const std::shared_ptr<GoalHandleT> & goal_handle)
  {
    const auto right_state =
      ensure_tool_calibration_position(right_group, goal_handle);
    const auto left_state = ensure_calibration_position_for_arm(
      left_group, goal_handle,
      drawer_left_tool_.calibration_joint_names,
      drawer_left_tool_.calibration_joint_positions);
    return {left_state, right_state};
  }

  void require_absolute_ros_name(
    const std::string & value,
    const std::string & parameter_name,
    bool is_service) const
  {
    if (value.empty() || value.front() != '/') {
      throw std::invalid_argument(
              "Parameter '" + parameter_name +
              "' must be an absolute ROS name.");
    }
    try {
      (void)rclcpp::expand_topic_or_service_name(
        value, get_name(), get_namespace(), is_service);
    } catch (const std::exception & error) {
      throw std::invalid_argument(
              "Parameter '" + parameter_name +
              "' must be a valid absolute ROS name: " + error.what());
    }
  }

  void activate_goal(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id)
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_goal_type_ = type;
    active_goal_id_ = goal_id;
    // An accepted goal does not own shared MoveIt/Nav2/base resources.  The
    // worker promotes this flag only after it has acquired the global lease.
    active_goal_owns_physical_motion_resources_ = false;
    active_goal_physical_outcome_committed_ = false;
    cancel_requested_.store(false);
  }

  void claim_active_goal_physical_motion_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id,
    bool resources_required)
  {
    if (!resources_required) {
      return;
    }
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    const bool goal_is_active =
      active_goal_matches_locked(type, goal_id) &&
      !cancel_requested_.load() && !shutdown_requested_.load();
    const bool lease_is_active =
      operation_lease_held_.load() && !operation_lease_lost_.load();
    if (!physical_motion_resources_are_owned(
        resources_required, lease_is_active, goal_is_active))
    {
      throw OperationError(
              lease_is_active ? PressCabinetButton::Result::CANCELED :
              PressCabinetButton::Result::LEASE_LOST,
              lease_is_active ?
              "Cabinet button operation was canceled before it acquired "
              "physical motion resources." :
              "The global robot operation lease was lost before physical "
              "motion resources were acquired.");
    }
    active_goal_owns_physical_motion_resources_ = true;
  }

  bool active_goal_owns_physical_motion_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) const
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    return active_goal_matches_locked(type, goal_id) &&
           active_goal_owns_physical_motion_resources_;
  }

  void relinquish_active_goal_physical_motion_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_matches_locked(type, goal_id)) {
      active_goal_owns_physical_motion_resources_ = false;
    }
  }

  bool commit_active_goal_physical_outcome(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (!active_goal_matches_locked(type, goal_id) ||
      cancel_requested_.load() || shutdown_requested_.load() ||
      operation_lease_lost_.load() || !rclcpp::ok())
    {
      return false;
    }
    active_goal_physical_outcome_committed_ = true;
    return true;
  }

  bool active_goal_matches_locked(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) const noexcept
  {
    return active_goal_type_ == type && active_goal_id_ == goal_id;
  }

  bool cancel_requested_for(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) const
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    return !active_goal_matches_locked(type, goal_id) ||
           cancel_requested_.load();
  }

  bool cancel_requested_for(
    const std::shared_ptr<PressGoalHandle> & goal_handle) const
  {
    return !goal_handle || cancel_requested_for(
      ActiveGoalType::PRESS, goal_handle->get_goal_id());
  }

  bool cancel_requested_for(
    const std::shared_ptr<OperateGoalHandle> & goal_handle) const
  {
    return !goal_handle || cancel_requested_for(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id());
  }

  template<typename GoalHandleT>
  bool goal_should_stop(
    const std::shared_ptr<GoalHandleT> & goal_handle) const noexcept
  {
    try {
      return cancel_requested_for(goal_handle) ||
             goal_handle->is_canceling() || shutdown_requested_.load() ||
             operation_lease_lost_.load() || !rclcpp::ok();
    } catch (...) {
      return true;
    }
  }

  template<typename GoalHandleT>
  bool goal_canceling_after_request(
    const std::shared_ptr<GoalHandleT> & goal_handle) const noexcept
  {
    try {
      if (!goal_handle) {
        return false;
      }
      if (!cancel_requested_for(goal_handle)) {
        return goal_handle->is_canceling();
      }

      // rclcpp_action changes the handle to CANCELING only after the user
      // cancel callback returns ACCEPT. The worker can observe our bound flag
      // during that small interval, so wait for the action state transition
      // before selecting the terminal state.
      const auto deadline = std::chrono::steady_clock::now() + 1s;
      while (!goal_handle->is_canceling() &&
        std::chrono::steady_clock::now() < deadline && rclcpp::ok())
      {
        std::this_thread::sleep_for(1ms);
      }
      return goal_handle->is_canceling();
    } catch (...) {
      return true;
    }
  }

  void clear_active_goal(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_matches_locked(type, goal_id)) {
      active_goal_type_ = ActiveGoalType::NONE;
      active_goal_id_ = {};
      active_goal_owns_physical_motion_resources_ = false;
      active_goal_physical_outcome_committed_ = false;
    }
  }

  void release_failed_goal_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    clear_latched_cabinet_transform();
    if (active_goal_owns_physical_motion_resources(type, goal_id)) {
      stop_active_motion();
      cancel_active_navigation();
      request_navigation_mode_without_wait(false);
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    relinquish_active_goal_physical_motion_resources(type, goal_id);
    release_operation_lease_noexcept();
    publish_active_control("");
  }

  void abort_pending_press_goal(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    std::uint8_t error_code,
    const std::string & message,
    bool release_operation_slot) noexcept
  {
    auto result = std::make_shared<PressCabinetButton::Result>();
    result->success = false;
    result->error_code = error_code;
    result->message = message;
    result->failure_reason = message;
    try {
      if (goal_handle && goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to abort pending button goal: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to abort pending button goal.");
    }
    if (release_operation_slot && goal_handle) {
      release_failed_goal_resources(
        ActiveGoalType::PRESS, goal_handle->get_goal_id());
      clear_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id());
      operation_active_.store(false);
    }
  }

  void abort_pending_operate_goal(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    std::uint8_t error_code,
    const std::string & message,
    bool release_operation_slot) noexcept
  {
    auto result = std::make_shared<OperateCabinetControl::Result>();
    result->success = false;
    result->error_code = error_code;
    result->message = message;
    result->failure_reason = message;
    result->diagnostic_stage = "accepted_callback";
    try {
      if (goal_handle && goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to abort pending cabinet goal: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to abort pending cabinet goal.");
    }
    if (release_operation_slot && goal_handle) {
      release_failed_goal_resources(
        ActiveGoalType::OPERATE, goal_handle->get_goal_id());
      clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
      operation_active_.store(false);
    }
  }

  rclcpp_action::CancelResponse cancel_active_goal(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id)
  {
    // Keep the binding locked until all global motion resources have been
    // stopped. This prevents a completed goal from releasing the operation
    // slot and a new goal from becoming active midway through a late cancel.
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (!active_goal_matches_locked(type, goal_id)) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    if (client_cancel_disposition(
        active_goal_physical_outcome_committed_) ==
      ClientCancelDisposition::REJECT_AFTER_PHYSICAL_COMMIT)
    {
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_requested_.store(true);
    if (active_goal_owns_physical_motion_resources_) {
      stop_active_motion();
      cancel_active_navigation();
      publish_manual_base_stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // The mounted end-effector toolset only serves the control types whose
  // contact tool link it carries: Set A (three-cylinder + two-cylinder)
  // operates buttons and doors, Set B (rotate-button + rocker) operates
  // knobs and switches.  The shared adapter allowlist may span both
  // toolsets, so the catalog must report only the mounted set as
  // physically operable; execute_operate() rejects the unmounted set before
  // changing a MoveIt profile or looking up TF.
  bool tool_serves_control(std::uint8_t control_type) const
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    if (toolset_ == "B") {
      return control_type == Control::TYPE_KNOB ||
             control_type == Control::TYPE_SWITCH;
    }
    return control_type == Control::TYPE_BUTTON ||
           control_type == Control::TYPE_DOOR ||
           control_type == Control::TYPE_SLIDER ||
           control_type == Control::TYPE_DRAWER;
  }

  std::string required_toolset_for_control(std::uint8_t control_type) const
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    if (control_type == Control::TYPE_KNOB ||
      control_type == Control::TYPE_SWITCH)
    {
      return "B";
    }
    return "A";
  }

  static bool is_slider_type(std::uint8_t control_type)
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    return control_type == Control::TYPE_SLIDER;
  }

  static bool is_drawer_type(std::uint8_t control_type)
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    return control_type == Control::TYPE_DRAWER;
  }

  static bool is_grasp_free_type(std::uint8_t control_type)
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    return control_type == Control::TYPE_BUTTON ||
           control_type == Control::TYPE_SLIDER;
  }

  void configure_controls()
  {
    const auto control_ids = declare_parameter<std::vector<std::string>>(
      "control_ids", std::vector<std::string>{});
    if (control_ids.empty()) {
      throw std::invalid_argument(
              "Parameter 'control_ids' must contain at least one control.");
    }
    const auto operable_control_ids =
      declare_parameter<std::vector<std::string>>(
      "operable_control_ids", std::vector<std::string>{});
    const auto inoperable_control_reason = declare_parameter<std::string>(
      "inoperable_control_reason",
      "The control has not passed this robot adapter's complete physical "
      "operation and recovery validation.");
    // A control whose operating tool is not mounted in the current
    // end-effector toolset (e.g. Set A cannot rotate a knob) must report the
    // toolset mismatch before the generic planning-only policy.  This gives
    // the Web client a concrete corrective action even when the control is
    // not part of the physical-operation allowlist.
    const auto toolset_mismatch_reason = declare_parameter<std::string>(
      "toolset_mismatch_reason",
      "The control's operating tool is not mounted in the current end-effector "
      "toolset; status display remains available, but valid MoveIt planning "
      "and physical execution require switching to the matching toolset.");
    const std::unordered_set<std::string> operable_controls(
      operable_control_ids.begin(), operable_control_ids.end());
    if (operable_controls.size() != operable_control_ids.size() ||
      inoperable_control_reason.empty())
    {
      throw std::invalid_argument(
              "Robot operable_control_ids must be unique and the shared "
              "inoperable_control_reason must be non-empty.");
    }

    const auto button_default_axis = declare_parameter<std::vector<double>>(
      "button_defaults.axis", {0.0, 0.0, -1.0});
    const auto button_default_approach =
      declare_parameter<std::vector<double>>(
      "button_defaults.approach_normal", {0.0, 0.0, 1.0});
    const double button_default_min = declare_parameter<double>(
      "button_defaults.min_position", 0.0);
    const double button_default_max = declare_parameter<double>(
      "button_defaults.max_position", 0.008);
    const auto button_default_state_ids =
      declare_parameter<std::vector<std::string>>(
      "button_defaults.state_ids", {"released", "pressed"});
    const auto button_default_state_labels =
      declare_parameter<std::vector<std::string>>(
      "button_defaults.state_labels", {"释放", "按下"});
    const auto button_default_state_positions =
      declare_parameter<std::vector<double>>(
      "button_defaults.state_positions", {0.0, 0.006});
    const double button_default_spring_stiffness =
      declare_parameter<double>(
      "button_defaults.spring_stiffness", 800.0);
    const double button_default_press_threshold =
      declare_parameter<double>(
      "button_defaults.press_threshold", 0.006);
    const double button_default_force = declare_parameter<double>(
      "button_defaults.default_force", 5.0);

    const auto knob_default_axis = declare_parameter<std::vector<double>>(
      "knob_defaults.axis", {0.0, 0.0, 1.0});
    const auto knob_default_approach =
      declare_parameter<std::vector<double>>(
      "knob_defaults.approach_normal", {0.0, 0.0, 1.0});
    const double knob_default_min = declare_parameter<double>(
      "knob_defaults.min_position", -0.78539816339);
    const double knob_default_max = declare_parameter<double>(
      "knob_defaults.max_position", 0.78539816339);
    const auto knob_default_state_ids =
      declare_parameter<std::vector<std::string>>(
      "knob_defaults.state_ids", {"left", "center", "right"});
    const auto knob_default_state_labels =
      declare_parameter<std::vector<std::string>>(
      "knob_defaults.state_labels", {"左档", "中档", "右档"});
    const auto knob_default_state_positions =
      declare_parameter<std::vector<double>>(
      "knob_defaults.state_positions",
      {-0.78539816339, 0.0, 0.78539816339});

    const auto checked_vector3 = [](const std::vector<double> & values,
        const std::string & name) {
        if (values.size() != 3U ||
          !std::all_of(
            values.begin(), values.end(),
            [](double value) {return std::isfinite(value);}))
        {
          throw std::invalid_argument(
                  "Parameter '" + name +
                  "' must contain three finite values.");
        }
        return tf2::Vector3(values[0], values[1], values[2]);
      };
    checked_vector3(button_default_axis, "button_defaults.axis");
    checked_vector3(
      button_default_approach, "button_defaults.approach_normal");
    checked_vector3(knob_default_axis, "knob_defaults.axis");
    checked_vector3(knob_default_approach, "knob_defaults.approach_normal");
    const auto & parameter_overrides =
      get_node_parameters_interface()->get_parameter_overrides();

    std::unordered_set<std::string> seen_button_ids;
    for (const auto & control_id : control_ids) {
      if (control_id.empty()) {
        throw std::invalid_argument(
                "Parameter 'control_ids' must not contain an empty ID.");
      }
      if (!seen_button_ids.insert(control_id).second) {
        throw std::invalid_argument(
                "Duplicate cabinet control ID '" + control_id + "'.");
      }

      const std::string prefix = "controls." + control_id + ".";
      auto button = std::make_shared<ButtonSpec>();
      button->id = control_id;
      const auto type_name = declare_parameter<std::string>(
        prefix + "type", "");
      if (type_name == "button") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
      } else if (type_name == "knob") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB;
      } else if (type_name == "switch") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SWITCH;
      } else if (type_name == "door") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
      } else if (type_name == "slider") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SLIDER;
      } else if (type_name == "drawer") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DRAWER;
      } else {
        throw std::invalid_argument(
                "Unsupported control type '" + type_name + "' for '" +
                control_id + "'.");
      }
      const bool is_button = button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
      const bool is_knob = button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB;
      const bool is_switch = button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SWITCH;
      const bool is_slider = is_slider_type(button->control_type);
      button->display_name = declare_parameter<std::string>(
        prefix + "display_name", control_id);
      button->joint_name = declare_parameter<std::string>(
        prefix + "joint_name", "");
      button->joint_state_topic = declare_parameter<std::string>(
        prefix + "joint_state_topic",
        control_id + "/joint_states");
      button->pressed_topic = declare_parameter<std::string>(
        prefix + "pressed_topic",
        is_button ? control_id + "/pressed" : "");
      button->state_topic = declare_parameter<std::string>(
        prefix + "state_topic",
        control_id + "/state");
      const auto local_position = declare_parameter<std::vector<double>>(
        prefix + "local_position", std::vector<double>{});
      const auto pivot_position = declare_parameter<std::vector<double>>(
        prefix + "pivot_position", local_position);
      const auto axis = declare_parameter<std::vector<double>>(
        prefix + "axis", is_knob ? knob_default_axis : button_default_axis);
      const auto approach_normal = declare_parameter<std::vector<double>>(
        prefix + "approach_normal",
        is_knob ? knob_default_approach : button_default_approach);
      button->min_position = declare_parameter<double>(
        prefix + "min_position",
        is_knob ? knob_default_min : button_default_min);
      button->max_position = declare_parameter<double>(
        prefix + "max_position",
        is_knob ? knob_default_max : button_default_max);
      button->state_ids = declare_parameter<std::vector<std::string>>(
        prefix + "state_ids",
        is_knob ? knob_default_state_ids : button_default_state_ids);
      button->state_labels = declare_parameter<std::vector<std::string>>(
        prefix + "state_labels",
        is_knob ? knob_default_state_labels : button_default_state_labels);
      button->state_positions = declare_parameter<std::vector<double>>(
        prefix + "state_positions",
        is_knob ? knob_default_state_positions :
        button_default_state_positions);
      if (is_knob) {
        if (button->state_ids.empty()) {
          throw std::invalid_argument(
                  "Knob '" + control_id +
                  "' must define at least one detent before tool calibration.");
        }
        const std::size_t transition_count =
          button->state_ids.size() * button->state_ids.size();
        button->tool_roll_offsets = declare_parameter<std::vector<double>>(
          prefix + "tool_roll_offsets",
          std::vector<double>(transition_count, 0.0));
        button->detent_release_fraction = declare_parameter<double>(
          prefix + "detent_release_fraction", 1.0);
        if (button->tool_roll_offsets.size() != transition_count ||
          std::any_of(
            button->tool_roll_offsets.begin(),
            button->tool_roll_offsets.end(),
            [](double value) {
              return !std::isfinite(value) ||
              std::abs(value) > std::acos(-1.0);
            }))
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' tool_roll_offsets must contain one finite [-pi, pi] "
                  "entry for every source/target detent pair.");
        }
        if (!std::isfinite(button->detent_release_fraction) ||
          button->detent_release_fraction <= 0.5 ||
          button->detent_release_fraction > 1.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' detent_release_fraction must be in (0.5, 1.0].");
        }
      }
      // ready_joint_seed 对按钮与旋钮均有效：七轴臂对同一夹具位姿存在多组
      // IK，把自由空间目标（如 prepress）固定到 r_arm_4<0 的连续分支，可让
      // 接近、按压、撤回全程避开腕部翻转奇点。set_pose_target_with_
      // calibrated_ik_seed 在规划时按 move_group 变量名严格校验种子。
      button->ready_joint_seed_names =
        declare_parameter<std::vector<std::string>>(
        prefix + "ready_joint_seed.joint_names",
        std::vector<std::string>{});
      button->ready_joint_seed_positions =
        declare_parameter<std::vector<double>>(
        prefix + "ready_joint_seed.positions",
        std::vector<double>{});
      {
        const bool has_ready_joint_seed =
          !button->ready_joint_seed_names.empty() ||
          !button->ready_joint_seed_positions.empty();
        std::unordered_set<std::string> unique_seed_names(
          button->ready_joint_seed_names.begin(),
          button->ready_joint_seed_names.end());
        if (has_ready_joint_seed &&
          (button->ready_joint_seed_names.empty() ||
          button->ready_joint_seed_names.size() !=
          button->ready_joint_seed_positions.size() ||
          unique_seed_names.size() != button->ready_joint_seed_names.size() ||
          std::any_of(
            button->ready_joint_seed_names.begin(),
            button->ready_joint_seed_names.end(),
            [](const std::string & name) {return name.empty();}) ||
          std::any_of(
            button->ready_joint_seed_positions.begin(),
            button->ready_joint_seed_positions.end(),
            [](double value) {return !std::isfinite(value);})))
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' ready_joint_seed must contain equal-length unique "
                  "joint_names and finite positions.");
        }
      }
      if (is_button) {
        button->tool_roll_offset = declare_parameter<double>(
          prefix + "tool_roll_offset", 0.0);
        if (!std::isfinite(button->tool_roll_offset) ||
          std::abs(button->tool_roll_offset) > std::acos(-1.0))
        {
          throw std::invalid_argument(
                  "Button '" + control_id +
                  "' tool_roll_offset must be finite and within [-pi, pi].");
        }
        button->spring_stiffness = declare_parameter<double>(
          prefix + "spring_stiffness", button_default_spring_stiffness);
        button->press_threshold = declare_parameter<double>(
          prefix + "press_threshold", button_default_press_threshold);
        button->default_force = declare_parameter<double>(
          prefix + "default_force", button_default_force);
      }
      const bool in_allowlist = operable_controls.count(control_id) != 0U;
      const bool tool_serves = tool_serves_control(button->control_type);
      button->adapter_validated = in_allowlist;
      button->toolset_compatible = tool_serves;
      button->required_toolset = required_toolset_for_control(
        button->control_type);
      button->operable = in_allowlist && tool_serves;
      button->unavailable_reason = declare_parameter<std::string>(
        prefix + "unavailable_reason", "");
      if (button->operable && !button->unavailable_reason.empty()) {
        throw std::invalid_argument(
                "Operable control '" + control_id +
                "' must not declare an unavailable_reason.");
      }
      if (!button->operable && button->unavailable_reason.empty()) {
        // 当前套装未挂载其操作工具时，先报告“套装不匹配”，而非笼统地报告
        // 未列入物理操作清单；切换到正确套装后才可能进入规划验证路径。
        button->unavailable_reason = !tool_serves ?
          toolset_mismatch_reason :
          inoperable_control_reason;
      }
      const std::string station_prefix = prefix + "navigation_station.";
      const std::string station_anchor_parameter =
        station_prefix + "local_anchor";
      const std::string station_axis_parameter =
        station_prefix + "outward_axis";
      const std::string station_standoff_parameter =
        station_prefix + "standoff";
      const bool station_configured =
        parameter_overrides.count(station_anchor_parameter) != 0U ||
        parameter_overrides.count(station_axis_parameter) != 0U ||
        parameter_overrides.count(station_standoff_parameter) != 0U ||
        parameter_overrides.count(station_prefix + "base_yaw_offset") != 0U ||
        parameter_overrides.count(station_prefix + "frame_id") != 0U;
      if (station_configured) {
        if (parameter_overrides.count(station_anchor_parameter) == 0U ||
          parameter_overrides.count(station_axis_parameter) == 0U ||
          parameter_overrides.count(station_standoff_parameter) == 0U)
        {
          throw std::invalid_argument(
                  "Control '" + control_id + "' navigation_station must "
                  "define local_anchor, outward_axis and standoff.");
        }
        ControlNavigationStation station;
        station.local_anchor = checked_vector3(
          declare_parameter<std::vector<double>>(
            station_anchor_parameter, std::vector<double>{}),
          station_anchor_parameter);
        station.outward_axis = checked_vector3(
          declare_parameter<std::vector<double>>(
            station_axis_parameter, std::vector<double>{}),
          station_axis_parameter);
        if (station.outward_axis.length2() <= 1.0e-12) {
          throw std::invalid_argument(
                  "Parameter '" + station_axis_parameter +
                  "' must not be zero.");
        }
        station.outward_axis.normalize();
        station.standoff = declare_parameter<double>(
          station_standoff_parameter, 0.0);
        if (!std::isfinite(station.standoff) || station.standoff <= 0.0) {
          throw std::invalid_argument(
                  "Parameter '" + station_standoff_parameter +
                  "' must be positive and finite.");
        }
        station.base_yaw_offset = declare_parameter<double>(
          station_prefix + "base_yaw_offset", 0.0);
        if (!std::isfinite(station.base_yaw_offset)) {
          throw std::invalid_argument(
                  "Parameter '" + station_prefix +
                  "base_yaw_offset' must be finite.");
        }
        station.frame_id = declare_parameter<std::string>(
          station_prefix + "frame_id", navigation_frame_);
        if (station.frame_id != navigation_frame_) {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' navigation_station frame must match navigation_frame '" +
                  navigation_frame_ + "'.");
        }
        if (button->operable && !station_standoff_is_safe(
            station.standoff, docking_base_footprint_,
            docking_base_footprint_padding_, docking_position_tolerance_,
            docking_yaw_tolerance_, station.base_yaw_offset))
        {
          const double minimum_standoff = minimum_safe_station_standoff(
            docking_base_footprint_, docking_base_footprint_padding_,
            docking_position_tolerance_, docking_yaw_tolerance_,
            station.base_yaw_offset);
          throw std::invalid_argument(
                  "Operable control '" + control_id +
                  "' navigation_station.standoff=" +
                  std::to_string(station.standoff) +
                  " m is unsafe for the configured docking-base footprint; "
                  "it must be at least " +
                  std::to_string(minimum_standoff) +
                  " m including footprint padding and docking tolerance.");
        }
        button->navigation_station = std::move(station);
      }
      if (button->operable && !button->navigation_station) {
        throw std::invalid_argument(
                "Operable control '" + control_id +
                "' requires an explicit robot-adapter navigation_station.");
      }
      button->parent_control_id = declare_parameter<std::string>(
        prefix + "parent_control_id", "");
      button->grasp_operated = declare_parameter<bool>(
        prefix + "grasp_operated", false);
      const bool is_drawer = is_drawer_type(button->control_type);
      if (button->grasp_operated && !is_slider && !is_drawer) {
        throw std::invalid_argument(
                "Control '" + control_id +
                "' grasp_operated is only supported for slider and drawer "
                "controls.");
      }
      // A front-operated drawer (grasp_operated) is no longer a grasp-free
      // control: its execution branch attaches the probe to the panel and drags
      // it.  Derive the effective grasp-free flag so requires_grasp, the grasp
      // outward offset and the unit all follow the intended branch.
      const bool is_grasp_free =
        is_grasp_free_type(button->control_type) && !button->grasp_operated;
      button->requires_grasp = declare_parameter<bool>(
        prefix + "requires_grasp", !is_grasp_free);
      if (!grasp_requirement_matches_control_kind(
          is_grasp_free, button->requires_grasp))
      {
        throw std::invalid_argument(
                "Control '" + control_id + "' requires_grasp must be " +
                (is_grasp_free ?
                "false for a button or slider." :
                "true for a knob, switch or door."));
      }
      if (!is_grasp_free) {
        const double default_grasp_offset = is_knob ?
          grasp_outward_offset_ : (is_switch ? 0.012 : 0.015);
        button->grasp_outward_offset = declare_parameter<double>(
          prefix + "grasp_outward_offset", default_grasp_offset);
        if (!std::isfinite(button->grasp_outward_offset) ||
          button->grasp_outward_offset <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' has an invalid grasp outward offset.");
        }
      }
      // A drawer travels linearly along its rail (m); a grasped rotary control
      // reports angular travel (rad).
      button->unit = (is_grasp_free || is_drawer) ? "m" : "rad";
      if (is_drawer) {
        // Bimanual pull-out drawer contract (P0-frozen geometry).  The drawer
        // branch drives two arms, so it requires the full dual-side contract
        // (left/right handles, supports, the right-hand unlock zone and the
        // slide axis).  Missing fields fail loudly at startup rather than at
        // operation time.
        require_drawer_tools_configured();
        button->drawer_bimanual = true;
        button->drawer_axis = checked_vector3(
          declare_parameter<std::vector<double>>(
            prefix + "drawer_axis", std::vector<double>{}),
          prefix + "drawer_axis");
        if (button->drawer_axis.length2() <= 1.0e-12) {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_axis must be a non-zero vector.");
        }
        button->drawer_axis.normalize();
        const auto require_point = [&](const char * key,
            tf2::Vector3 & target) {
          target = checked_vector3(
            declare_parameter<std::vector<double>>(
              prefix + key, std::vector<double>{}),
            prefix + key);
        };
        require_point("left_handle_point", button->left_handle_point);
        button->has_left_handle_point = true;
        require_point("right_handle_point", button->right_handle_point);
        button->has_right_handle_point = true;
        require_point("left_support_point", button->left_support_point);
        button->has_left_support_point = true;
        require_point("right_support_point", button->right_support_point);
        button->has_right_support_point = true;
        require_point("unlock_press_point", button->unlock_press_point);
        button->has_unlock_press_point = true;
        button->drawer_grasp_outward_offset = declare_parameter<double>(
          prefix + "drawer_grasp_outward_offset",
          button->drawer_grasp_outward_offset);
        if (!std::isfinite(button->drawer_grasp_outward_offset) ||
          button->drawer_grasp_outward_offset < 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_grasp_outward_offset must be finite and "
                  "non-negative.");
        }
        button->drawer_grasp_press_depth = declare_parameter<double>(
          prefix + "drawer_grasp_press_depth",
          button->drawer_grasp_press_depth);
        if (!std::isfinite(button->drawer_grasp_press_depth) ||
          button->drawer_grasp_press_depth < 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_grasp_press_depth must be finite and "
                  "non-negative.");
        }
        button->drawer_unlock_approach_offset = declare_parameter<double>(
          prefix + "drawer_unlock_approach_offset",
          button->drawer_unlock_approach_offset);
        if (!std::isfinite(button->drawer_unlock_approach_offset) ||
          button->drawer_unlock_approach_offset <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_unlock_approach_offset must be positive and "
                  "finite.");
        }
        button->drawer_unlock_press_depth = declare_parameter<double>(
          prefix + "drawer_unlock_press_depth",
          button->drawer_unlock_press_depth);
        if (!std::isfinite(button->drawer_unlock_press_depth) ||
          button->drawer_unlock_press_depth <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_unlock_press_depth must be positive and finite.");
        }
        button->drawer_unlock_distance_threshold = declare_parameter<double>(
          prefix + "drawer_unlock_distance_threshold",
          button->drawer_unlock_distance_threshold);
        if (!std::isfinite(button->drawer_unlock_distance_threshold) ||
          button->drawer_unlock_distance_threshold <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_unlock_distance_threshold must be positive and "
                  "finite.");
        }
        button->drawer_sync_tolerance = declare_parameter<double>(
          prefix + "drawer_sync_tolerance", button->drawer_sync_tolerance);
        if (!std::isfinite(button->drawer_sync_tolerance) ||
          button->drawer_sync_tolerance <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_sync_tolerance must be positive and finite.");
        }
        // 2026-09-03 AGENT §3: the drawer's unlock is now a real motor
        // extension contract (mirrors the scene <control> fields and the
        // P1-identified unlock motor, e.g. r_three_cyl_finger3_joint).  A
        // drawer control must declare its unlock motor joint; the retracted /
        // pressed / floor / ceiling positions are validated like the plugin's
        // gate so the operator can never command a motor outside the plugin's
        // accepted band.  unlock_pressed_position is a first-pass target whose
        // exact value is field-sealed at §7.2.
        button->unlock_motor_joint = declare_parameter<std::string>(
          prefix + "unlock_motor_joint", std::string{});
        if (button->unlock_motor_joint.empty()) {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' is a drawer with an unlock zone but declares no "
                  "unlock_motor_joint (the P1 right unlock motor that must "
                  "extend to release the rail latch).");
        }
        button->unlock_retracted_position = declare_parameter<double>(
          prefix + "unlock_retracted_position",
          button->unlock_retracted_position);
        button->unlock_pressed_position = declare_parameter<double>(
          prefix + "unlock_pressed_position",
          button->unlock_pressed_position);
        button->unlock_extension_floor = declare_parameter<double>(
          prefix + "unlock_extension_floor",
          button->unlock_extension_floor);
        button->unlock_extension_ceiling = declare_parameter<double>(
          prefix + "unlock_extension_ceiling",
          button->unlock_extension_ceiling);
        if (!std::isfinite(button->unlock_retracted_position) ||
          !std::isfinite(button->unlock_pressed_position) ||
          !std::isfinite(button->unlock_extension_floor) ||
          !std::isfinite(button->unlock_extension_ceiling) ||
          button->unlock_retracted_position < 0.0 ||
          button->unlock_extension_floor < 0.0 ||
          button->unlock_pressed_position <
            button->unlock_retracted_position +
            button->unlock_extension_floor ||
          button->unlock_pressed_position >=
            button->unlock_extension_ceiling)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' unlock-motor positions must satisfy "
                  "retracted >= 0, floor >= 0 and retracted + floor <= "
                  "pressed < ceiling (all finite).");
        }
        if (button->state_ids.size() >= 2U &&
          button->state_positions.size() >= 2U)
        {
          // closed/open detents are the operation endpoints; the remaining
          // fields fall back to the frozen 0.0 / 0.30 defaults.
          button->drawer_closed_position = button->state_positions.front();
          button->drawer_open_position = button->state_positions.back();
        }
        if (button->drawer_open_position <= button->drawer_closed_position) {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' open detent must be strictly greater than the closed "
                  "detent.");
        }
      }
      if (is_slider) {
        button->slider_release_position = declare_parameter<double>(
          prefix + "slider_release_position",
          std::numeric_limits<double>::quiet_NaN());
        button->slider_open_press_offset = declare_parameter<double>(
          prefix + "slider_open_press_offset", 0.0);
        if (!std::isfinite(button->slider_open_press_offset) ||
          button->slider_open_press_offset < 0.0)
        {
          throw std::invalid_argument(
                  "Slider control '" + control_id +
                  "' has an invalid slider_open_press_offset (must be "
                  "finite and non-negative).");
        }
        button->slider_close_press_offset = declare_parameter<double>(
          prefix + "slider_close_press_offset", 0.0);
        if (!std::isfinite(button->slider_close_press_offset) ||
          button->slider_close_press_offset < 0.0)
        {
          throw std::invalid_argument(
                  "Slider control '" + control_id +
                  "' has an invalid slider_close_press_offset (must be "
                  "finite and non-negative).");
        }
        if (std::isfinite(button->slider_release_position) &&
          std::isfinite(button->slider_close_press_offset) &&
          button->slider_release_position + button->slider_close_press_offset >
            button->max_position + 1e-6)
        {
          throw std::invalid_argument(
                  "Slider control '" + control_id +
                  "' requires slider_release_position + "
                  "slider_close_press_offset <= max_position so the close "
                  "press target stays within the joint travel.");
        }
      }
      // A knob is deliberately single-detent only: wrapping TOGGLE from the
      // right detent to the left detent would cross an intermediate detent
      // and is rejected by the physical transition guard.  Switches and
      // doors have two-state semantics and may expose TOGGLE.
      if (is_button) {
        button->supported_commands =
          xczs_inspection_robot_interfaces::msg::CabinetControl::SUPPORT_PRESS;
      } else {
        button->supported_commands = static_cast<std::uint8_t>(
          xczs_inspection_robot_interfaces::msg::CabinetControl::SUPPORT_SET_STATE |
          xczs_inspection_robot_interfaces::msg::CabinetControl::
          SUPPORT_SET_POSITION |
          ((is_knob || is_drawer) ? 0U :
          xczs_inspection_robot_interfaces::msg::CabinetControl::SUPPORT_TOGGLE));
      }
      if (button->display_name.empty() || button->joint_name.empty() ||
        button->joint_state_topic.empty() || button->state_topic.empty() ||
        (is_button && button->pressed_topic.empty()))
      {
        throw std::invalid_argument(
                "Cabinet control '" + control_id +
                "' has an empty catalog field.");
      }
      const auto local = checked_vector3(
        local_position, prefix + "local_position");
      const auto pivot = checked_vector3(
        pivot_position, prefix + "pivot_position");
      button->axis = checked_vector3(axis, prefix + "axis");
      button->approach_normal = checked_vector3(
        approach_normal, prefix + "approach_normal");
      if (button->axis.length2() < 1.0e-12 ||
        button->approach_normal.length2() < 1.0e-12)
      {
        throw std::invalid_argument(
                "Control '" + control_id + "' has a zero axis or normal.");
      }
      button->axis.normalize();
      button->approach_normal.normalize();
      button->local_x = local.x();
      button->local_y = local.y();
      button->local_z = local.z();
      button->pivot_x = pivot.x();
      button->pivot_y = pivot.y();
      button->pivot_z = pivot.z();
      if (!std::isfinite(button->min_position) ||
        !std::isfinite(button->max_position) ||
        button->min_position >= button->max_position ||
        button->state_ids.size() != button->state_labels.size() ||
        button->state_ids.size() != button->state_positions.size() ||
        button->state_ids.empty())
      {
        throw std::invalid_argument(
                "Control '" + control_id +
                "' has invalid limits or state presets.");
      }
      // A front-operated drawer (grasp_operated) has no push-push release
      // mechanism: the panel is dragged along the slide axis and there is no
      // over-travel trip point, so slider_release_position is intentionally
      // absent for such controls.
      if (is_slider && !button->grasp_operated &&
        (!std::isfinite(button->slider_release_position) ||
        button->state_positions.empty() ||
        button->slider_release_position <= button->state_positions.back() ||
        button->slider_release_position > button->max_position))
      {
        throw std::invalid_argument(
                "Slider control '" + control_id +
                "' requires a slider_release_position strictly above the "
                "open detent and no further than max_position, so the robot "
                "can trip the push-push release.");
      }
      if (is_button &&
        (!std::isfinite(button->spring_stiffness) ||
        button->spring_stiffness <= 0.0 ||
        !std::isfinite(button->press_threshold) ||
        button->press_threshold <= 0.0 ||
        button->press_threshold > button->max_position ||
        !std::isfinite(button->default_force) ||
        button->default_force <= 0.0 ||
        button->default_force >
        button->spring_stiffness * button->max_position))
      {
        throw std::invalid_argument(
                "Button '" + control_id +
                "' has an invalid force profile.");
      }
      for (const double position : button->state_positions) {
        if (!std::isfinite(position) || position < button->min_position - 1e-9 ||
          position > button->max_position + 1e-9)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' has a preset outside its limits.");
        }
      }
      buttons_by_id_.emplace(control_id, button);
      buttons_in_order_.push_back(std::move(button));
    }

    for (const auto & control : buttons_in_order_) {
      std::unordered_set<std::string> ancestors;
      std::string parent_id = control->parent_control_id;
      while (!parent_id.empty()) {
        const auto parent = buttons_by_id_.find(parent_id);
        if (parent == buttons_by_id_.end()) {
          throw std::invalid_argument(
                  "Control '" + control->id + "' references unknown parent '" +
                  parent_id + "'.");
        }
        if (!ancestors.insert(parent_id).second || parent_id == control->id) {
          throw std::invalid_argument(
                  "Control '" + control->id +
                  "' has a cyclic parent_control_id chain.");
        }
        if (parent->second->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
        {
          throw std::invalid_argument(
                  "Control '" + control->id +
                  "' cannot use a prismatic button as its parent.");
        }
        parent_id = parent->second->parent_control_id;
      }
    }
    for (const auto & operable_control : operable_controls) {
      if (buttons_by_id_.count(operable_control) == 0U) {
        throw std::invalid_argument(
                "Unknown operable_control_ids entry '" +
                operable_control + "'.");
      }
    }
  }

  void subscribe_to_button_states()
  {
    for (const auto & button : buttons_in_order_) {
      button_joint_state_subscriptions_.push_back(
        create_subscription<sensor_msgs::msg::JointState>(
          button->joint_state_topic,
          rclcpp::SensorDataQoS(),
          [this, button](
            const sensor_msgs::msg::JointState::SharedPtr message)
          {
            receive_button_joint_state(*button, *message);
          }));
      if (!button->pressed_topic.empty()) {
        button_pressed_subscriptions_.push_back(
          create_subscription<std_msgs::msg::Bool>(
            button->pressed_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this, button](const std_msgs::msg::Bool::SharedPtr message) {
              receive_button_pressed(*button, *message);
            }));
      }
      control_state_subscriptions_.push_back(
        create_subscription<
          xczs_inspection_robot_interfaces::msg::CabinetControlState>(
          button->state_topic,
          rclcpp::QoS(1).reliable().transient_local(),
          [this, button](
            const xczs_inspection_robot_interfaces::msg::CabinetControlState::
            SharedPtr message)
          {
            receive_control_state(*button, *message);
          }));
    }
  }

  // ---------------------------------------------------------------------------
  // 2026-09-04 AGENT §8.4 fix#3 root-cause (v11): 真实关节状态直读。
  //
  // v11 的 deep-press evidence 判据读 move_group.getCurrentState() 的杆位置，
  // 而该 scene 从未收到真实 joint_states：GetPlanningScene 实测 rods ≈0 m
  // （l_two_cyl_finger2 = +0.001776 vs 物理 0.052）、arm 关节也卡在启动位形
  // （r_arm_0 = −0.0035 vs 物理 −1.69）——current state 从启动起从未更新，
  // 任何 "measured" 都读垃圾（失败消息 left measured 0.003274 vs 物理脉冲
  // 52 mm）。本订阅是 rod measured 的唯一信源：read_real_joint_position 与
  // FJT start_positions 优先读此缓存（新鲜度门槛内），缓存缺样本才退回
  // move_group scene（旧行为，仅栈外环境/断流时兜底）。
  void subscribe_to_real_joint_states()
  {
    if (real_joint_state_topic_.empty()) {
      throw std::invalid_argument(
              "Parameter 'joint_state_topic' must not be empty.");
    }
    // Reentrant group: a long operate (execute 全程占着默认 MutuallyExclusive
    // group 数百秒) 会把同组订阅回调饿死——首次实测即 WARN cache 不新鲜、
    // measured 退回 stale scene。100 Hz 的 real JS 必须在操作中持续刷新。
    real_joint_states_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = real_joint_states_callback_group_;
    real_joint_states_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
      real_joint_state_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(real_joint_states_mutex_);
        real_joint_states_stamp_ = message->header.stamp;
        // 新鲜度判定以接收时刻（steady clock）为准，而不是拿 header.stamp
        // 与本节点 ros 时间相减：消息 stamp 由发布侧按仿真钟盖上，若本节点
        // 的 use_sim_time 时钟与仿真钟脱钩（/clock QoS 中断等），两种时钟
        // 混合相减会恒判超龄/恒判负龄 → 缓存恒 stale（clean2 实测 18/18）。
        // 100 Hz 的真实订阅下，接收时刻在 1.0 s 门限内即等价于样本新鲜。
        real_joint_states_receipt_ = std::chrono::steady_clock::now();
        real_joint_positions_.clear();
        real_joint_positions_.reserve(message->name.size());
        real_joint_efforts_.clear();
        real_joint_efforts_.reserve(message->name.size());
        for (std::size_t i = 0; i < message->name.size(); ++i) {
          real_joint_positions_[message->name[i]] = message->position[i];
          real_joint_efforts_[message->name[i]] = message->effort[i];
        }
      },
      sub_options);
    RCLCPP_INFO(
      get_logger(),
      "Subscribed to REAL robot joint states on '%s' (rod measured source).",
      real_joint_state_topic_.c_str());
  }

  // ---------------------------------------------------------------------------
  // 2026-09-05 fix#13 (v28 取证): 双臂 arm 控制器指令参考（des）直读。
  //
  // 控制器当前正在执行的指令参考不是 move_group current state 的实测值：负载
  // 下物理沉降 O = des − act 可达 ±7 mrad/关节（hook 咬合抽屉把手后 R arm3
  // +7.0 / L arm4 −3.5 mrad 实测）。controller_state.reference 正是“现在命令
  // 各关节停在哪”（active 驻留轨迹的终态 X_arr），self-center 微修正的轨迹
  // 必须把终态表达为 X_arr + dj（而非 act + dj）——否则控制器换轨时 t=0 从
  // X_arr 阶跃到 act，物理立刻再沉降一个 O，净位移变成 dj + O，修正自毁
  // （v28 实锤：L 侧修正后残差 2.5 → 5.3 mm，物理位移 ≈ FK(dj) + FK(O)）。
  // 本订阅缓存 left/right_arm_controller 的 reference（按关节名），新鲜度门
  // 与 real joint states 同规则（接收时刻 steady clock ≤ 1 s；参考在驻留下
  // 冻结，50 Hz 发布下门内样本即等价于当前命令）。
  void subscribe_to_arm_controller_references()
  {
    // Reentrant：一次 operate 执行全程占着默认 MutuallyExclusive group，参考
    // 缓存必须独立刷新，否则长任务中订阅被饿死 → 缓存恒 stale → 锚点回退。
    arm_controller_reference_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = arm_controller_reference_callback_group_;
    const auto subscribe_one = [this, &sub_options](bool left_side) {
      const std::string topic = controller_namespace_ +
        (left_side ? "/left_arm_controller/controller_state" :
        "/right_arm_controller/controller_state");
      return create_subscription<
        control_msgs::msg::JointTrajectoryControllerState>(
        topic, rclcpp::QoS(10),
        [this, left_side](
          const control_msgs::msg::JointTrajectoryControllerState::SharedPtr
          message) {
          std::lock_guard<std::mutex> lock(arm_controller_reference_mutex_);
          auto & receipt = left_side ?
            left_arm_reference_receipt_ : right_arm_reference_receipt_;
          auto & positions = left_side ?
            left_arm_reference_positions_ : right_arm_reference_positions_;
          receipt = std::chrono::steady_clock::now();
          positions.clear();
          auto & joint_order = left_side ?
            left_arm_controller_joint_order_ : right_arm_controller_joint_order_;
          if (joint_order.empty()) {
            joint_order = message->joint_names;
          }
          const std::size_t sample_count = std::min(
            message->joint_names.size(),
            static_cast<std::size_t>(message->reference.positions.size()));
          for (std::size_t i = 0; i < sample_count; ++i) {
            positions[message->joint_names[i]] =
              message->reference.positions[i];
          }
        },
        sub_options);
    };
    left_arm_controller_state_subscription_ = subscribe_one(true);
    right_arm_controller_state_subscription_ = subscribe_one(false);
    RCLCPP_INFO(
      get_logger(),
      "Subscribed to left/right arm controller command references (fix#13 "
      "self-center anchor source).");
  }

  // 读取某侧 arm 控制器当前指令参考，返回向量按查询关节序。全部关节命中且
  // 接收新鲜（steady-clock ≤ 1 s）才返回 true；否则清空输出并返回 false，
  // 调用方回退“以实测构型为锚”（fix#13 前语义，re-anchor 偏置仍在——应打
  // 日志照实报告，下一轮物理测量兜底）。
  bool arm_controller_reference(
    bool left_side, const std::vector<std::string> & joint_names,
    std::vector<double> & reference_out)
  {
    constexpr double kArmControllerReferenceMaxAgeSeconds = 1.0;
    std::lock_guard<std::mutex> lock(arm_controller_reference_mutex_);
    const auto & receipt = left_side ?
      left_arm_reference_receipt_ : right_arm_reference_receipt_;
    const auto & positions = left_side ?
      left_arm_reference_positions_ : right_arm_reference_positions_;
    if (positions.empty()) {
      return false;
    }
    const double age_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - receipt).count();
    if (age_seconds < 0.0 || age_seconds > kArmControllerReferenceMaxAgeSeconds) {
      return false;
    }
    reference_out.clear();
    reference_out.reserve(joint_names.size());
    for (const auto & name : joint_names) {
      const auto it = positions.find(name);
      if (it == positions.end()) {
        reference_out.clear();
        return false;
      }
      reference_out.push_back(it->second);
    }
    return true;
  }

  // Freshness-gated read of the direct joint-state cache.  Returns false when
  // the cache is empty, the sample's receipt is older than the age budget
  // (steady-clock seconds; the message stamp is authored by the publisher's
  // sim clock and must not be differenced against a possibly-desynced local
  // ros time — the mixed-clock subtraction was the stale-18/18 root cause),
  // or the joint is absent.
  bool cached_real_joint_position(
    const std::string & joint, double & position_out)
  {
    constexpr double kRealJointStateMaxAgeSeconds = 1.0;
    std::lock_guard<std::mutex> lock(real_joint_states_mutex_);
    if (real_joint_positions_.empty()) {
      return false;
    }
    const double age_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - real_joint_states_receipt_).count();
    if (age_seconds < 0.0 || age_seconds > kRealJointStateMaxAgeSeconds) {
      return false;
    }
    const auto it = real_joint_positions_.find(joint);
    if (it == real_joint_positions_.end()) {
      return false;
    }
    position_out = it->second;
    return true;
  }

  // Freshness-gated read of the direct joint-state effort cache, mirroring
  // cached_real_joint_position (same mutex, same age rule).  Effort is the
  // hook-seal press signal (fix#3 v2).
  bool cached_real_joint_effort(
    const std::string & joint, double & effort_out)
  {
    constexpr double kRealJointStateMaxAgeSeconds = 1.0;
    std::lock_guard<std::mutex> lock(real_joint_states_mutex_);
    if (real_joint_efforts_.empty()) {
      return false;
    }
    const double age_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - real_joint_states_receipt_).count();
    if (age_seconds < 0.0 || age_seconds > kRealJointStateMaxAgeSeconds) {
      return false;
    }
    const auto it = real_joint_efforts_.find(joint);
    if (it == real_joint_efforts_.end()) {
      return false;
    }
    effort_out = it->second;
    return true;
  }

  void publish_control_catalog()
  {
    xczs_inspection_robot_interfaces::msg::CabinetControlCatalog catalog;
    catalog.controls.reserve(buttons_in_order_.size());
    for (const auto & button : buttons_in_order_) {
      xczs_inspection_robot_interfaces::msg::CabinetControl control;
      control.control_id = button->id;
      control.display_name = button->display_name;
      control.control_type =
        button->control_type;
      control.joint_name = button->joint_name;
      control.joint_state_topic = button->joint_state_topic;
      control.pressed_topic = button->pressed_topic;
      control.state_topic = button->state_topic;
      control.supported_commands = button->supported_commands;
      control.unit = button->unit;
      control.min_position = button->min_position;
      control.max_position = button->max_position;
      control.state_ids = button->state_ids;
      control.state_labels = button->state_labels;
      control.state_positions = button->state_positions;
      control.requires_grasp = button->requires_grasp;
      control.operable = button->operable;
      control.unavailable_reason = button->unavailable_reason;
      control.required_toolset = button->required_toolset;
      control.toolset_compatible = button->toolset_compatible;
      control.adapter_validated = button->adapter_validated;
      if (button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
      {
        control.default_force = button->default_force;
        control.min_trigger_force =
          button->spring_stiffness * button->press_threshold;
        control.max_force =
          button->spring_stiffness * button->max_position;
      }
      catalog.controls.push_back(std::move(control));
    }
    control_catalog_publisher_->publish(catalog);
  }

  std::shared_ptr<ButtonSpec> find_button(
    const std::string & button_id) const
  {
    const auto iterator = buttons_by_id_.find(button_id);
    return iterator == buttons_by_id_.end() ? nullptr : iterator->second;
  }

  void publish_active_control(const std::string & control_id) noexcept
  {
    std::string operation_lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_heartbeat_mutex_);
      operation_heartbeat_control_id_ = control_id;
    }
    if (!control_id.empty()) {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (operation_lease_held_.load() && !operation_lease_lost_.load()) {
        operation_lease_id = operation_lease_id_;
      }
    }
    publish_active_control_message(control_id, operation_lease_id);
    if (!control_id.empty()) {
      // Acquiring the lease is also a valid initial liveness event; periodic
      // heartbeats thereafter are emitted only after successful renewals.
      publish_operation_heartbeat();
    }
  }

  void publish_operation_heartbeat() noexcept
  {
    std::string control_id;
    std::string lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_heartbeat_mutex_);
      control_id = operation_heartbeat_control_id_;
    }
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (operation_lease_held_.load() && !operation_lease_lost_.load()) {
        lease_id = operation_lease_id_;
      }
    }
    if (control_id.empty() || lease_id.empty()) {
      return;
    }
    try {
      std_msgs::msg::String message;
      // The active-control topic already carries the control identity.  The
      // volatile heartbeat carries the globally unique lease identity so a
      // delayed request/heartbeat from an older same-control operation cannot
      // authorize a new physical grasp.
      message.data = lease_id;
      operation_heartbeat_publisher_->publish(message);
    } catch (const std::exception & error) {
      mark_operation_lease_lost_for_exact_lease(
        std::string("Failed to publish the cabinet operation heartbeat: ") +
        error.what(), lease_id);
    } catch (...) {
      mark_operation_lease_lost_for_exact_lease(
        "Failed to publish the cabinet operation heartbeat unexpectedly.",
        lease_id);
    }
  }

  void publish_active_control_message(
    const std::string & control_id,
    const std::string & operation_lease_id) noexcept
  {
    try {
      std_msgs::msg::String message;
      message.data = control_id;
      active_control_publisher_->publish(message);
    } catch (const std::exception & error) {
      if (!control_id.empty()) {
        mark_operation_lease_lost_for_exact_lease(
          std::string("Failed to publish the active cabinet control: ") +
          error.what(), operation_lease_id);
      } else {
        RCLCPP_ERROR(
          get_logger(), "Failed to publish the idle cabinet control: %s",
          error.what());
      }
    } catch (...) {
      if (!control_id.empty()) {
        mark_operation_lease_lost_for_exact_lease(
          "Failed to publish the active cabinet control unexpectedly.",
          operation_lease_id);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to publish the idle cabinet control unexpectedly.");
      }
    }
  }

  void reset_controls(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    bool expected = false;
    if (!operation_active_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message =
        "Cabinet controls cannot be reset during an active operation.";
      return;
    }

    try {
      if (!reset_physics_client_->wait_for_service(2s)) {
        response->success = false;
        response->message = "Cabinet reset_physics service is unavailable.";
      } else {
        auto future = reset_physics_client_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
        if (future.wait_for(3s) != std::future_status::ready) {
          response->success = false;
          response->message = "Cabinet reset_physics service timed out.";
        } else {
          const auto reset_response = future.get();
          response->success = reset_response->success;
          response->message = reset_response->message;
        }
      }
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("Cabinet reset failed: ") + error.what();
    }
    operation_active_.store(false);
  }

  rclcpp_action::GoalResponse handle_operate_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const OperateCabinetControl::Goal> goal)
  {
    // Always accept at the transport layer.  Unknown controls and resource
    // contention are recorded and converted to an action result in the
    // accepted callback, so clients never lose the failure reason in a bare
    // GoalResponse::REJECT.
    const auto control = find_button(goal->control_id);
    PendingGoalDisposition disposition =
      control ? PendingGoalDisposition::EXECUTE :
      PendingGoalDisposition::INVALID_CONTROL;
    bool expected = false;
    if (disposition == PendingGoalDisposition::EXECUTE &&
      !operation_active_.compare_exchange_strong(expected, true))
    {
      disposition = PendingGoalDisposition::RESOURCE_BUSY;
    }
    if (disposition == PendingGoalDisposition::EXECUTE) {
      activate_goal(ActiveGoalType::OPERATE, uuid);
    }
    remember_pending_goal(uuid, disposition);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_operate_cancel(
    const std::shared_ptr<OperateGoalHandle> goal_handle)
  {
    return goal_handle ?
           cancel_active_goal(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id()) :
           rclcpp_action::CancelResponse::REJECT;
  }

  void handle_operate_accepted(
    const std::shared_ptr<OperateGoalHandle> goal_handle)
  {
    const auto disposition = take_pending_goal(goal_handle->get_goal_id());
    if (disposition != PendingGoalDisposition::EXECUTE) {
      abort_pending_operate_goal(
        goal_handle,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        OperateCabinetControl::Result::RESOURCE_BUSY :
        OperateCabinetControl::Result::INVALID_CONTROL,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        "Cabinet operation rejected because another operation is active." :
        "Unknown cabinet control; no configured control matches control_id.",
        false);
      return;
    }
    std::lock_guard<std::mutex> lock(worker_mutex_);
    try {
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
      worker_thread_ = std::thread(
        [this, goal_handle]() {
          try {
            execute_operate(goal_handle);
          } catch (const std::exception & error) {
            abort_pending_operate_goal(
              goal_handle,
              OperateCabinetControl::Result::INTERNAL_ERROR,
              std::string("Cabinet worker terminated unexpectedly: ") +
              error.what(), true);
          } catch (...) {
            abort_pending_operate_goal(
              goal_handle,
              OperateCabinetControl::Result::INTERNAL_ERROR,
              "Cabinet worker terminated unexpectedly.", true);
          }
        });
    } catch (const std::exception & error) {
      abort_pending_operate_goal(
        goal_handle,
        OperateCabinetControl::Result::INTERNAL_ERROR,
        std::string("Failed to start cabinet worker: ") + error.what(), true);
    } catch (...) {
      abort_pending_operate_goal(
        goal_handle,
        OperateCabinetControl::Result::INTERNAL_ERROR,
        "Failed to start cabinet worker.", true);
    }
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

  double unit_interval_parameter(
    const std::string & name,
    double default_value)
  {
    const double value = positive_parameter(name, default_value);
    if (value > 1.0) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must not exceed 1.0.");
    }
    return value;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const PressCabinetButton::Goal> goal)
  {
    const auto control = find_button(goal->button_id);
    PendingGoalDisposition disposition =
      control && control->control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON ?
      PendingGoalDisposition::EXECUTE : PendingGoalDisposition::INVALID_BUTTON;
    bool expected = false;
    if (disposition == PendingGoalDisposition::EXECUTE &&
      !operation_active_.compare_exchange_strong(expected, true))
    {
      disposition = PendingGoalDisposition::RESOURCE_BUSY;
    }
    if (disposition == PendingGoalDisposition::EXECUTE) {
      activate_goal(ActiveGoalType::PRESS, uuid);
    }
    remember_pending_goal(uuid, disposition);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<PressGoalHandle> goal_handle)
  {
    return goal_handle ?
           cancel_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id()) :
           rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const std::shared_ptr<PressGoalHandle> goal_handle)
  {
    const auto disposition = take_pending_goal(goal_handle->get_goal_id());
    if (disposition != PendingGoalDisposition::EXECUTE) {
      abort_pending_press_goal(
        goal_handle,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        PressCabinetButton::Result::RESOURCE_BUSY :
        PressCabinetButton::Result::INVALID_BUTTON,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        "Cabinet button operation rejected because another operation is active." :
        "Unknown cabinet button; no configured button matches button_id.",
        false);
      return;
    }
    std::lock_guard<std::mutex> lock(worker_mutex_);
    try {
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
      worker_thread_ = std::thread(
        [this, goal_handle]() {
          try {
            execute(goal_handle);
          } catch (const std::exception & error) {
            abort_pending_press_goal(
              goal_handle,
              PressCabinetButton::Result::INTERNAL_ERROR,
              std::string("Cabinet worker terminated unexpectedly: ") +
              error.what(), true);
          } catch (...) {
            abort_pending_press_goal(
              goal_handle,
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Cabinet worker terminated unexpectedly.", true);
          }
        });
    } catch (const std::exception & error) {
      abort_pending_press_goal(
        goal_handle,
        PressCabinetButton::Result::INTERNAL_ERROR,
        std::string("Failed to start cabinet worker: ") + error.what(), true);
    } catch (...) {
      abort_pending_press_goal(
        goal_handle,
        PressCabinetButton::Result::INTERNAL_ERROR,
        "Failed to start cabinet worker.", true);
    }
  }

  void execute(const std::shared_ptr<PressGoalHandle> goal_handle)
  {
    const auto operation_started_at = std::chrono::steady_clock::now();
    auto result = std::make_shared<PressCabinetButton::Result>();
    const auto button = find_button(goal_handle->get_goal()->button_id);
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses poses;
    ControlStagingPoses staging_poses;
    bool should_attempt_retreat = false;
    bool move_group_ready_for_motion = false;
    bool request_success = false;

    if (!embedded_navigation_request_is_supported(
        goal_handle->get_goal()->navigate_to_staging_pose,
        allow_embedded_navigation_))
    {
      result->success = false;
      result->error_code = PressCabinetButton::Result::NAVIGATION_FAILED;
      result->message = kEmbeddedNavigationDisabledMessage;
      result->max_travel = button ?
        button_snapshot(*button).peak_position : 0.0;
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      finish_goal_noexcept(goal_handle, result, false);
      clear_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id());
      operation_active_.store(false);
      return;
    }

    try {
      if (!button) {
        throw OperationError(
                PressCabinetButton::Result::INVALID_BUTTON,
                "The accepted cabinet button is no longer configured.");
      }
      if (!tool_serves_control(button->control_type)) {
        throw OperationError(
                PressCabinetButton::Result::TOOLSET_MISMATCH,
                "Button '" + button->id + "' requires end-effector toolset " +
                button->required_toolset + ", but the mounted toolset is " +
                toolset_ + ".");
      }
      if (!button->adapter_validated) {
        throw OperationError(
                PressCabinetButton::Result::ADAPTER_NOT_VALIDATED,
                button->unavailable_reason.empty() ?
                "This button is not physically validated by the robot "
                "adapter; use the generic cabinet operation for planning-only "
                "validation." : button->unavailable_reason);
      }
      // Bind the button operation's arm group, tip link, contact tool link and
      // transport target to the button type before any MoveIt planning.
      apply_tool_profile(
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON);
      const bool should_navigate_to_staging_pose =
        goal_handle->get_goal()->navigate_to_staging_pose;
      acquire_operation_lease(
        goal_handle, ActiveGoalType::PRESS, true);
      if (!should_navigate_to_staging_pose) {
        publish_active_control(button->id);
      }
      reset_peak_position(*button);
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::WAITING_FOR_SYSTEM,
        0.02F,
        "Waiting for the cabinet button state.");
      wait_for_fresh_button_state(
        goal_handle, *button, operation_started_at);
      check_cancel(goal_handle);
      latch_cabinet_transform();
      interruptible_hold(goal_handle, planning_scene_settle_seconds_);

      poses = calculate_operation_poses(*button, press_depth_);
      staging_poses = calculate_control_staging_poses(*button);

      move_group = std::make_shared<MoveGroupInterface>(
        shared_from_this(),
        MoveGroupInterface::Options(
          move_group_name_, "robot_description", move_group_namespace_),
        transform_buffer_,
        rclcpp::Duration::from_seconds(system_wait_timeout_));
      check_cancel(goal_handle);
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        active_move_group_ = move_group;
      }
      {
        // A fresh MoveGroupInterface replaces any wedged one from a prior
        // goal: abandon the execute serialization a wedged worker is holding
        // so the new goal can move on its own independent instance.  The old
        // worker's in-flight execute() only touches the old instance, which it
        // keeps alive through its own action client.
        std::lock_guard<std::mutex> lock(motion_execute_serial_->guard_mutex);
        motion_execute_serial_->wedged_lock.reset();
      }
      configure_move_group(*move_group);
      move_group_ready_for_motion = true;
      const auto initial_robot_state =
        ensure_tool_calibration_position(*move_group, goal_handle);
      verify_tool_tip_calibration_state(*initial_robot_state);

      if (should_navigate_to_staging_pose) {
        publish_feedback(
          goal_handle,
          PressCabinetButton::Feedback::NAVIGATING,
          0.05F,
          "Returning the arm to its safe transport target.");
        plan_and_execute_named_target(
          *move_group, goal_handle, transport_named_target_);
        publish_feedback(
          goal_handle,
          PressCabinetButton::Feedback::NAVIGATING,
          0.08F,
          "Driving the base to the cabinet staging pose.");
        // map->odom is expected to evolve while AMCL observes a moving base.
        // The pre-navigation latch was only needed to build the map-frame
        // goal and to stow safely; do not treat localization corrections as
        // cabinet motion while Nav2 is active.
        clear_latched_cabinet_transform();
        navigate_to_staging_pose(
          goal_handle, staging_poses.navigation_pose);

        // AMCL may update map->odom while Nav2 is moving.  Refresh every
        // planning-frame target after navigation instead of docking and
        // manipulating against the odom snapshot captured before the goal.
        // Keep active_control empty during this settle interval so the
        // planning-scene node is also allowed to adopt the refreshed TF.
        latch_cabinet_transform();
        poses = calculate_operation_poses(*button, press_depth_);
        staging_poses = calculate_control_staging_poses(*button);
        interruptible_hold(goal_handle, planning_scene_settle_seconds_);
        publish_active_control(button->id);
      }
      set_navigation_mode(goal_handle, false);
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::NAVIGATING,
        0.20F,
        "Refining the configured control station from odometry.");
      dock_to_staging_pose(goal_handle, staging_poses.planning_pose);
      interruptible_hold(goal_handle, planning_scene_settle_seconds_);
      verify_staging_pose_before_arm_motion(
        goal_handle, staging_poses.planning_pose);
      const auto predelivery_robot_state =
        ensure_tool_calibration_position(*move_group, goal_handle);
      verify_tool_tip_calibration_state(*predelivery_robot_state);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::MOVING_TO_PREPRESS,
        0.25F,
        "Planning to the button prepress pose.");
      plan_and_execute_pose(
        *move_group, goal_handle, poses.prepress_pose, contact_tool_link_,
        nullptr, button.get());
      should_attempt_retreat = true;

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::APPROACHING,
        0.48F,
        "Approaching the button along the panel normal.");
      execute_cartesian_path(
        *move_group,
        goal_handle,
        {poses.contact_pose},
        cartesian_velocity_scale_,
        cartesian_acceleration_scale_);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::PRESSING,
        0.67F,
        "Pressing " + button->id + " through its physical travel.");
      const auto press_transition_sequence =
        button_snapshot(*button).pressed_transition_sequence;
      execute_cartesian_path(
        *move_group,
        goal_handle,
        {poses.pressed_pose},
        cartesian_velocity_scale_ * 0.5,
        cartesian_acceleration_scale_ * 0.5,
        button_press_minimum_cartesian_fraction_);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::VERIFYING_PRESS,
        0.78F,
        "Verifying the Gazebo button press state.");
      if (!wait_for_pressed_state(
          goal_handle,
          *button,
          true,
          press_detection_timeout_,
          press_transition_sequence))
      {
        throw OperationError(
                PressCabinetButton::Result::PRESS_NOT_DETECTED,
                "The arm reached the press pose, but the button did not "
                "cross its 6 mm press threshold.");
      }
      interruptible_hold(goal_handle, press_hold_seconds_);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::RETRACTING,
        0.88F,
        "Retracting the probe from the cabinet.");
      const auto release_transition_sequence =
        button_snapshot(*button).pressed_transition_sequence;
      execute_cartesian_path(
        *move_group,
        goal_handle,
        {poses.contact_pose, poses.prepress_pose},
        cartesian_velocity_scale_,
        cartesian_acceleration_scale_);
      should_attempt_retreat = false;

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::VERIFYING_RELEASE,
        0.97F,
        "Verifying that the button spring returned to its released state.");
      if (!wait_for_pressed_state(
          goal_handle,
          *button,
          false,
          release_detection_timeout_,
          release_transition_sequence))
      {
        throw OperationError(
                PressCabinetButton::Result::RELEASE_NOT_DETECTED,
                "The probe retracted, but the button did not return below "
                "its release threshold.");
      }

      // Linearization point: both the press and release side effects have
      // been physically verified.  A later client cancel must not label this
      // completed press as canceled or stop the safety transport trajectory.
      if (!commit_active_goal_physical_outcome(
          ActiveGoalType::PRESS, goal_handle->get_goal_id()))
      {
        check_cancel(goal_handle);
      }
      result->physical_outcome_confirmed = true;
      result->final_state_verified = true;

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::RETRACTING,
        0.99F,
        "Returning the arm to its safe transport target.");
      plan_and_execute_named_target(
        *move_group, goal_handle, transport_named_target_, nullptr, true);
      result->transport_succeeded = true;

      result->success = true;
      result->error_code = PressCabinetButton::Result::SUCCESS;
      result->message =
        "Pressed and released " + button->id + " successfully.";
      result->max_travel = button_snapshot(*button).peak_position;
      result->recovery_succeeded = true;
      result->grasp_released = true;
      request_success = true;
    } catch (const OperationError & error) {
      const bool lease_available = !operation_lease_lost_.load();
      if (lease_available && should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      if (lease_available && move_group_ready_for_motion && move_group &&
        rclcpp::ok())
      {
        best_effort_stow(*move_group);
      }
      result->success = false;
      result->error_code = error.error_code;
      result->message = error.what();
      result->max_travel = button ?
        button_snapshot(*button).peak_position : 0.0;
      const bool client_canceled = is_goal_canceling_noexcept(goal_handle);
      if (client_canceled) {
        result->error_code = PressCabinetButton::Result::CANCELED;
        result->message = "Cabinet button operation was canceled.";
      } else if (error.error_code == PressCabinetButton::Result::CANCELED) {
        result->error_code = PressCabinetButton::Result::INTERNAL_ERROR;
        result->message =
          "Cabinet button operation stopped because ROS is shutting down.";
      }
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    } catch (const std::exception & error) {
      const bool lease_available = !operation_lease_lost_.load();
      if (lease_available && should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      if (lease_available && move_group_ready_for_motion && move_group &&
        rclcpp::ok())
      {
        best_effort_stow(*move_group);
      }
      result->success = false;
      const bool canceled = is_goal_canceling_noexcept(goal_handle);
      result->error_code = operation_lease_lost_.load() ?
        PressCabinetButton::Result::LEASE_LOST : canceled ?
        PressCabinetButton::Result::CANCELED :
        PressCabinetButton::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        "The global robot operation lease was lost; all motion was stopped." :
        canceled ?
        "Cabinet button operation was canceled." :
        std::string("Cabinet operation failed: ") + error.what();
      result->max_travel = button ?
        button_snapshot(*button).peak_position : 0.0;
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    } catch (...) {
      const bool lease_available = !operation_lease_lost_.load();
      if (lease_available && should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      if (lease_available && move_group_ready_for_motion && move_group &&
        rclcpp::ok())
      {
        best_effort_stow(*move_group);
      }
      result->success = false;
      const bool canceled = is_goal_canceling_noexcept(goal_handle);
      result->error_code = operation_lease_lost_.load() ?
        PressCabinetButton::Result::LEASE_LOST : canceled ?
        PressCabinetButton::Result::CANCELED :
        PressCabinetButton::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        "The global robot operation lease was lost; all motion was stopped." :
        canceled ?
        "Cabinet button operation was canceled." :
        "Cabinet operation failed with an unknown exception.";
      result->max_travel = button ?
        button_snapshot(*button).peak_position : 0.0;
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    }

    clear_latched_cabinet_transform();
    const bool owned_physical_motion_resources =
      active_goal_owns_physical_motion_resources(
      ActiveGoalType::PRESS, goal_handle->get_goal_id());
    if (owned_physical_motion_resources) {
      stop_active_motion();
      cancel_active_navigation();
      request_navigation_mode_without_wait(false);
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    // Drop the ownership flag before releasing the lease.  A late cancel may
    // still bind to this goal until its terminal transition, but must not stop
    // a subsequent lease holder.
    relinquish_active_goal_physical_motion_resources(
      ActiveGoalType::PRESS, goal_handle->get_goal_id());
    release_operation_lease_noexcept();
    publish_active_control("");
    const bool goal_succeeded =
      finish_goal_noexcept(goal_handle, result, request_success);
    clear_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id());
    operation_active_.store(false);
    if (goal_succeeded &&
      request_success && button)
    {
      RCLCPP_INFO(
        get_logger(), "Cabinet button %s operation succeeded (max %.2f mm).",
        button->id.c_str(), result->max_travel * 1000.0);
    }
  }

  void execute_operate(
    const std::shared_ptr<OperateGoalHandle> goal_handle)
  {
    const auto operation_started_at = std::chrono::steady_clock::now();
    auto result = std::make_shared<OperateCabinetControl::Result>();
    result->diagnostic_stage = "preflight";
    const auto control = find_button(goal_handle->get_goal()->control_id);
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses button_poses;
    OperationPoses slider_poses;
    RotaryOperationPoses rotary_poses;
    // Grasp-drag staging for a front-operated drawer (grasp_operated slider).
    RotaryOperationPoses drawer_poses;
    std::vector<geometry_msgs::msg::Pose> drawer_waypoints;
    std::optional<ControlStagingPoses> staging_poses;
    bool should_attempt_retreat = false;
    bool grasp_attached = false;
    bool request_success = false;
    DoorArcProgress door_arc_progress;
    double target_position = 0.0;
    double rotary_tool_roll_offset = 0.0;
    // Precomputed upstream so the manipulation block and the ready/pregrasp
    // branch selection share the exact same arc targets and waypoints.
    double rotary_manip_pos = 0.0;
    std::vector<geometry_msgs::msg::Pose> rotary_arc_waypoints;
    double button_press_depth = 0.0;
    bool button_should_trigger = false;
    bool is_button = false;
    bool is_slider = false;
    bool is_drawer = false;
    // Bimanual drawer state: the left arm runs on its own group, the right arm
    // on the shared move_group.  drawer_bimanual_attached tracks the plugin's
    // two-sided fixed constraint so recovery knows whether the drawer is still
    // rigidly held before it retreats either arm.
    std::shared_ptr<MoveGroupInterface> drawer_left_move_group;
    bool drawer_bimanual_attached = false;
    DrawerBimanualPoses drawer_bimanual_poses;
    const bool validation_only = control &&
      requires_planning_only_validation(control->operable);
    const auto preparation_policy = operation_preparation_policy(
      validation_only,
      goal_handle->get_goal()->navigate_to_staging_pose,
      control && control->navigation_station.has_value());
    std::string target_state;
    std::unordered_map<std::string, std::string> expected_parent_states;
    std::vector<ControlStabilityReference> pregrasp_stability_references;
    // 2026-09-03 AGENT §7.2: 封顶参数每次执行读取（驱动脚本两次任务间 param
    // set 即生效）。0 = 全流程；1..8 = 分级阶段边界（映射见 DebugStageCapReached
    // 头注释）。未知值不匹配任何边界，按全流程执行。
    const int debug_stage_cap = static_cast<int>(
      get_parameter("debug_stage_cap").as_int());

    if (!embedded_navigation_request_is_supported(
        goal_handle->get_goal()->navigate_to_staging_pose,
        allow_embedded_navigation_))
    {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::NAVIGATION_FAILED;
      result->message = kEmbeddedNavigationDisabledMessage;
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      finish_operate_goal_noexcept(goal_handle, result, false);
      clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
      operation_active_.store(false);
      return;
    }

    try {
      if (!control) {
        throw GenericOperationError(
                OperateCabinetControl::Result::INVALID_CONTROL,
                "The accepted cabinet control is no longer configured.");
      }
      if (!tool_serves_control(control->control_type)) {
        const std::string required_toolset =
          required_toolset_for_control(control->control_type);
        result->success = false;
        result->error_code =
          OperateCabinetControl::Result::TOOLSET_MISMATCH;
        result->message = "Control '" + control->id +
          "' requires end-effector toolset " + required_toolset +
          ", but the mounted toolset is " + toolset_ + ".";
        result->validation_performed = false;
        result->operation_executed = false;
        result->diagnostic_stage = "toolset_validation";
        result->policy_reason = control->unavailable_reason.empty() ?
          "Switch to end-effector toolset " + required_toolset +
          " before operating this control." :
          control->unavailable_reason;
        result->failure_reason = result->policy_reason;
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        finish_operate_goal_noexcept(goal_handle, result, false);
        clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
        operation_active_.store(false);
        return;
      }
      // Bind this operation's arm group, tip link, contact tool link and
      // transport target to the control's type before any MoveIt planning.
      // A bimanual drawer binds the single-arm members to the RIGHT drawer
      // tool so the shared transport/calibration/docking code drives the right
      // arm; the left arm is driven only through the bimanual helpers.
      if (is_drawer_type(control->control_type)) {
        apply_drawer_tool_profiles();
      } else {
        apply_tool_profile(control->control_type);
      }
      if (validation_only) {
        result->policy_reason = control->unavailable_reason.empty() ?
          "The selected control has not passed this robot adapter's complete "
          "physical operation and recovery validation." :
          control->unavailable_reason;
        result->diagnostic_stage = "preflight";
      }
      is_button = control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
      is_slider = is_slider_type(control->control_type);
      is_drawer = is_drawer_type(control->control_type);
      // Command-level validation: check that the command is compatible with
      // the control type and that its parameters are valid.  This runs inside
      // the accepted goal so that the caller receives a structured failure
      // with diagnostic details instead of a silent goal rejection.
      {
        bool command_valid = false;
        std::string command_reason;
        if (is_button) {
          command_valid = goal_handle->get_goal()->command ==
            OperateCabinetControl::Goal::COMMAND_PRESS;
          if (!command_valid) {
            command_reason = "Buttons only support the 'press' command.";
          }
        } else {
          const auto cmd = goal_handle->get_goal()->command;
          if (cmd == OperateCabinetControl::Goal::COMMAND_SET_STATE) {
            command_valid = (control->supported_commands &
              xczs_inspection_robot_interfaces::msg::CabinetControl::
              SUPPORT_SET_STATE) != 0U;
            if (!command_valid) {
              command_reason = "This control does not support setting a state.";
            }
            if (command_valid) {
              command_valid = std::find(
                control->state_ids.begin(), control->state_ids.end(),
                goal_handle->get_goal()->target_state) !=
                control->state_ids.end();
              if (!command_valid) {
                command_reason = "target_state '" +
                  goal_handle->get_goal()->target_state +
                  "' is not in the control's state list.";
              }
            }
          } else if (cmd ==
            OperateCabinetControl::Goal::COMMAND_SET_POSITION)
          {
            command_valid = (control->supported_commands &
              xczs_inspection_robot_interfaces::msg::CabinetControl::
              SUPPORT_SET_POSITION) != 0U;
            if (!command_valid) {
              command_reason =
                "This control does not support setting a position.";
            }
            if (command_valid) {
              command_valid = goal_handle->get_goal()->use_target_position &&
                std::isfinite(goal_handle->get_goal()->target_position) &&
                goal_handle->get_goal()->target_position >=
                control->min_position &&
                goal_handle->get_goal()->target_position <=
                control->max_position &&
                std::any_of(
                control->state_positions.begin(),
                control->state_positions.end(),
                [this, &goal_handle](double preset) {
                  return std::abs(
                    preset - goal_handle->get_goal()->target_position) <=
                  target_tolerance_;
                });
              if (!command_valid) {
                command_reason = "target_position is out of range or does " \
                  "not match a configured detent.";
              }
            }
          } else if (cmd == OperateCabinetControl::Goal::COMMAND_TOGGLE) {
            command_valid = (control->supported_commands &
              xczs_inspection_robot_interfaces::msg::CabinetControl::
              SUPPORT_TOGGLE) != 0U && control->state_ids.size() >= 2U;
            if (!command_valid) {
              command_reason = (control->supported_commands &
                xczs_inspection_robot_interfaces::msg::CabinetControl::
                SUPPORT_TOGGLE) == 0U ?
                "This control does not support toggle; choose an adjacent "
                "detent with set_state or set_position." :
                "Toggle requires at least 2 states.";
            }
          } else {
            command_reason = "Unknown command code " +
              std::to_string(static_cast<int>(cmd)) + ".";
          }
        }
        if (!command_valid) {
          result->success = false;
          result->error_code =
            OperateCabinetControl::Result::UNSUPPORTED_COMMAND;
          result->message = "Command not valid for control '" +
            control->id + "': " + command_reason;
          result->validation_performed = true;
          result->operation_executed = false;
          result->diagnostic_stage = "command_validation";
          result->policy_reason = command_reason;
          result->failure_reason = command_reason;
          RCLCPP_WARN(
            get_logger(), "%s", result->message.c_str());
          finish_operate_goal_noexcept(goal_handle, result, false);
          clear_active_goal(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id());
          operation_active_.store(false);
          return;
        }
      }
      if (is_button) {
        const double requested_force = goal_handle->get_goal()->force == 0.0 ?
          control->default_force : goal_handle->get_goal()->force;
        result->requested_force = requested_force;
        const double maximum_force =
          control->spring_stiffness * control->max_position;
        if (!std::isfinite(requested_force) || requested_force <= 0.0 ||
          requested_force > maximum_force)
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::INVALID_FORCE,
                  "Button force must be finite and within (0, " +
                  std::to_string(maximum_force) + "] N.");
        }
        button_press_depth = requested_force / control->spring_stiffness;
        button_should_trigger =
          button_press_depth + 1.0e-9 >= control->press_threshold;
      }
      acquire_operation_lease(
        goal_handle,
        ActiveGoalType::OPERATE,
        preparation_policy.requires_physical_motion_resources);
      if (!preparation_policy.execute_embedded_navigation) {
        publish_active_control(control->id);
      }
      if (!validation_only) {
        reset_peak_position(*control);
      }
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::WAITING_FOR_SYSTEM,
        0.02F, 0.0, "Waiting for a fresh physical control state.");
      wait_for_fresh_button_state(
        goal_handle, *control, operation_started_at);
      check_cancel(goal_handle);
      const auto initial_state = button_snapshot(*control);
      result->initial_position = initial_state.position;
      if (!is_button) {
        pregrasp_stability_references.push_back(
          {control.get(), initial_state.state_id, initial_state.position});
      }
      if (control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR)
      {
        door_arc_progress.initial_position = initial_state.position;
      }
      for (const auto * ancestor : control_ancestors(*control)) {
        wait_for_fresh_button_state(
          goal_handle, *ancestor, operation_started_at);
        const auto parent_initial_state = button_snapshot(*ancestor);
        expected_parent_states.emplace(
          ancestor->id, parent_initial_state.state_id);
        if (!is_button) {
          pregrasp_stability_references.push_back(
            {ancestor, parent_initial_state.state_id,
              parent_initial_state.position});
        }
      }
      std::tie(target_position, target_state) = resolve_operation_target(
        *control, *goal_handle->get_goal(), initial_state);
      if (is_button) {
        target_position = button_press_depth;
      } else if (std::abs(target_position - initial_state.position) <=
        target_tolerance_)
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Control '" + control->id + "' is already at requested "
                "state '" + target_state +
                "'; select a different physical detent.");
      } else if (control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
      {
        const auto source_index = control_state_index(*control, initial_state);
        const auto target_index = control_state_index(*control, target_state);
        if (!rotary_transition_is_adjacent(source_index, target_index)) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                  "旋钮 '" + control->id +
                  "' 不支持一次跨越两个档位；请先切换到中档，再执行下一步。");
        }
        rotary_tool_roll_offset = control->tool_roll_offsets.at(
          rotary_transition_matrix_index(
            source_index, target_index, control->state_ids.size()));
      }
      if (!is_button) {
        wait_for_pregrasp_controls_stable(
          goal_handle, *control, pregrasp_stability_references,
          std::chrono::steady_clock::now());
      }
      if (!try_station_anchor_latch(*control)) {
        latch_cabinet_transform();
      }

      if (is_button) {
        button_poses = calculate_operation_poses(
          *control, button_press_depth);
      } else if (is_drawer) {
        // 2026-09-03 AGENT §6.4 (P4): drawer 不再在 settings 预生成整臂位姿族/
        // waypoint 链 —— 单工作位姿由执行分支以到达前实测轨位锚定
        // （calculate_drawer_bimanual_work_poses），抽拉段在 pull 时从抽屉实时
        // 位置分段重算，settings 处无事可做。此分支刻意留空。
      } else if (is_slider && control->grasp_operated) {
        // Front-drawer: grasp-drag flow.  The manipulation waypoints follow
        // the slide axis from the panel's current position to the requested
        // detent -- pull east to open, push west to close -- with the probe
        // rigidly attached to the panel front face.
        drawer_poses = calculate_drawer_operation_poses(
          *control, initial_state.position, rotary_tool_roll_offset);
        drawer_waypoints = calculate_drawer_waypoints(
          *control, initial_state.position, target_position,
          rotary_tool_roll_offset);
      } else if (is_slider) {
        // The press face adapts to the current slide position.  Opening pushes
        // the panel forward to the target detent (target > current).  Closing
        // cannot pull the panel back (the tool only pushes a face), so the
        // robot presses it PAST the open detent, STRICTLY BEYOND the release
        // threshold (release + slider_close_press_offset) -- commanding the
        // press at exactly the release is a float coin-flip that hangs the
        // close (boot18).  Crossing the release mid-press trips the plugin's
        // push-push release, and the detent spring then returns the panel to
        // the closed detent.
        const double slider_press_target =
          target_position < initial_state.position ?
          control->slider_release_position + control->slider_close_press_offset :
          target_position - control->slider_open_press_offset;
        slider_poses = calculate_slider_operation_poses(
          *control, initial_state.position, slider_press_target);
      } else {
        rotary_poses = calculate_rotary_operation_poses(
          *control, initial_state.position, rotary_tool_roll_offset);
        // The manipulation arc is needed both for the runtime branch
        // selection below (validated against the real dock TF before any arm
        // motion) and for the actual arc execution after the grasp attaches.
        rotary_manip_pos = rotary_manipulation_position(
          *control, initial_state.position, target_position);
        rotary_arc_waypoints = calculate_rotation_waypoints(
          *control, initial_state.position, rotary_manip_pos,
          rotary_tool_roll_offset);
      }
      if (control->navigation_station &&
        (preparation_policy.execute_embedded_navigation ||
        preparation_policy.execute_precision_docking))
      {
        staging_poses = calculate_control_staging_poses(*control);
      }
      if (preparation_policy.execute_embedded_navigation && !staging_poses) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NAVIGATION_FAILED,
                "Embedded navigation requires an explicit robot-adapter "
                "navigation_station for control '" + control->id + "'.");
      }

      move_group = std::make_shared<MoveGroupInterface>(
        shared_from_this(),
        MoveGroupInterface::Options(
          move_group_name_, "robot_description", move_group_namespace_),
        transform_buffer_,
        rclcpp::Duration::from_seconds(system_wait_timeout_));
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        active_move_group_ = move_group;
      }
      {
        // See the press-goal equivalent above: a fresh move group replaces any
        // wedged one, so its execute serialization must be released.
        std::lock_guard<std::mutex> lock(motion_execute_serial_->guard_mutex);
        motion_execute_serial_->wedged_lock.reset();
      }
      configure_move_group(*move_group);
      moveit::core::RobotStatePtr calibrated_robot_state;
      if (is_drawer) {
        // The bimanual drawer also owns the left arm: create and configure its
        // group up front so recovery can always retreat it, then close BOTH
        // tool profiles' calibration joints before the first arm motion.
        drawer_left_move_group = std::make_shared<MoveGroupInterface>(
          shared_from_this(),
          MoveGroupInterface::Options(
            drawer_left_tool_.move_group, "robot_description",
            move_group_namespace_),
          transform_buffer_,
          rclcpp::Duration::from_seconds(system_wait_timeout_));
        // The LEFT arm's EEF is the left tool's contact link, NOT the active
        // right tool (contact_tool_link_).  Setting the right-tool link on the
        // left group made move_group resolve the Cartesian path for
        // 'r_three_cyl_base' against a left-arm group whose tip is 'l_arm_6',
        // failing every IK query (P3-8 db1 Cartesian 0%).
        configure_move_group(
          *drawer_left_move_group, drawer_left_tool_.contact_tool_link);
        const auto bimanual_states = ensure_bimanual_calibration_position(
          *drawer_left_move_group, *move_group, goal_handle);
        verify_bimanual_tool_calibration_state(
          *bimanual_states.first, *bimanual_states.second);
        calibrated_robot_state = bimanual_states.second;
      } else {
        calibrated_robot_state =
          ensure_tool_calibration_position(*move_group, goal_handle);
        verify_tool_tip_calibration_state(*calibrated_robot_state);
      }

      if (validation_only) {
        result->operation_executed = false;
        result->validation_performed = true;
        validate_inoperable_control_path(
          *move_group, goal_handle, *control, initial_state,
          target_position, is_slider ? slider_poses : button_poses,
          rotary_poses, rotary_tool_roll_offset, *calibrated_robot_state, result);
        result->diagnostic_stage = "complete";
        result->path_fraction = 1.0;
        result->required_fraction = 1.0;
        result->moveit_error_code =
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        throw GenericOperationError(
                OperateCabinetControl::Result::UNREACHABLE,
                "实时 MoveIt 运动学与碰撞规划验证全部通过，但没有发送任何"
                "机械臂轨迹、导航、精确对接、抓取或柜体写入命令；按钮触发、"
                "档位到达、串扰和安全恢复仍未经过物理闭环验证，因此继续受"
                "机器人适配层安全策略限制：" + result->policy_reason);
      }

      if (!physical_motion_is_permitted(
          control->operable, result->validation_performed))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::INTERNAL_ERROR,
                "Robot-adapter policy blocked physical cabinet motion.");
      }

      if (preparation_policy.execute_embedded_navigation) {
        result->diagnostic_stage = "navigation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::NAVIGATING,
          0.05F, target_position,
          "Returning the arm to its safe transport target.");
        plan_and_execute_named_target(
          *move_group, goal_handle, transport_named_target_,
          &result->operation_executed);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::NAVIGATING,
          0.07F, target_position,
          "Driving the base to the cabinet staging pose.");
        // Nav2 may update map->odom during navigation.  The physical cabinet
        // remains fixed in odom/world, so release the pre-navigation latch and
        // establish a new manipulation latch only after Nav2 has stopped.
        clear_latched_cabinet_transform();
        navigate_to_staging_pose(
          goal_handle, staging_poses->navigation_pose);

        // Embedded Nav2 can change map->odom through localization updates.
        // Rebuild all odom-frame manipulation targets from the post-navigation
        // TF.  active_control is deliberately still empty here so the
        // planning scene can refresh its matching cabinet transform too.
        latch_cabinet_transform();
        if (is_button) {
          button_poses = calculate_operation_poses(
            *control, button_press_depth);
        } else if (is_drawer) {
          // 2026-09-03 AGENT §6.4 (P4): 见本函数上部 is_drawer 分支注释 ——
          // settings 不再预生成 drawer 整臂位姿/waypoint，全部留到执行分支
          // live 锚定。此分支刻意留空。
        } else if (is_slider && control->grasp_operated) {
          drawer_poses = calculate_drawer_operation_poses(
            *control, initial_state.position, rotary_tool_roll_offset);
          drawer_waypoints = calculate_drawer_waypoints(
            *control, initial_state.position, target_position,
            rotary_tool_roll_offset);
        } else if (is_slider) {
          const double slider_press_target =
            target_position < initial_state.position ?
            control->slider_release_position + control->slider_close_press_offset :
            target_position - control->slider_open_press_offset;
          slider_poses = calculate_slider_operation_poses(
            *control, initial_state.position, slider_press_target);
        } else {
          rotary_poses = calculate_rotary_operation_poses(
            *control, initial_state.position, rotary_tool_roll_offset);
        }
        staging_poses = calculate_control_staging_poses(*control);
        interruptible_hold(goal_handle, planning_scene_settle_seconds_);
        publish_active_control(control->id);
      }
      if (preparation_policy.enter_manual_base_mode) {
        result->diagnostic_stage = "docking";
        set_navigation_mode(goal_handle, false);
      }
      if (preparation_policy.execute_precision_docking) {
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::DOCKING,
          0.18F, target_position,
          "Refining the configured control station from odometry.");
        dock_to_staging_pose(goal_handle, staging_poses->planning_pose);
      }
      if (preparation_policy.wait_for_scene_settle) {
        interruptible_hold(goal_handle, planning_scene_settle_seconds_);
      }
      if (preparation_policy.execute_precision_docking) {
        verify_staging_pose_before_arm_motion(
          goal_handle, staging_poses->planning_pose);
      }
      if (is_drawer) {
        const auto bimanual_states = ensure_bimanual_calibration_position(
          *drawer_left_move_group, *move_group, goal_handle);
        verify_bimanual_tool_calibration_state(
          *bimanual_states.first, *bimanual_states.second);
      } else {
        const auto predelivery_robot_state =
          ensure_tool_calibration_position(*move_group, goal_handle);
        verify_tool_tip_calibration_state(*predelivery_robot_state);
      }

      if (is_button) {
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning to the button ready pose.");
        plan_and_execute_pose(
          *move_group, goal_handle, button_poses.prepress_pose,
          contact_tool_link_, &result->operation_executed, control.get());
        should_attempt_retreat = true;
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.47F, target_position,
          "Approaching the button along its travel axis.");
        execute_cartesian_path(
          *move_group, goal_handle, {button_poses.contact_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, &result->operation_executed);
        const auto transition_sequence =
          button_snapshot(*control).pressed_transition_sequence;
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.65F, target_position,
          "Pressing the button through its physical travel.");
        execute_cartesian_path(
          *move_group, goal_handle, {button_poses.pressed_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          button_press_minimum_cartesian_fraction_,
          &result->operation_executed);
        track_button_force(
          *move_group, goal_handle, *control, button_press_depth,
          &result->operation_executed);
        result->diagnostic_stage = "verification";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.77F, target_position,
          button_should_trigger ?
          "Verifying the Gazebo press transition." :
          "Measuring the physical travel produced by the requested force.");
        if (button_should_trigger) {
          if (!wait_for_pressed_state(
              goal_handle, *control, true, press_detection_timeout_,
              transition_sequence))
          {
            throw GenericOperationError(
                    OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                    "The requested force should have triggered the button, "
                    "but no physical press transition was detected.");
          }
          result->button_triggered = true;
        } else {
          interruptible_hold(goal_handle, press_hold_seconds_);
          result->button_triggered = button_snapshot(*control).pressed;
        }
        if (button_should_trigger) {
          interruptible_hold(goal_handle, press_hold_seconds_);
        }
        const auto release_sequence =
          button_snapshot(*control).pressed_transition_sequence;
        result->diagnostic_stage = "retreat";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.88F, 0.0, "Retracting the probe from the button.");
        execute_cartesian_path(
          *move_group, goal_handle,
          {button_poses.contact_pose, button_poses.prepress_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, &result->operation_executed);
        should_attempt_retreat = false;
        result->diagnostic_stage = "verification";
        if (result->button_triggered && !wait_for_pressed_state(
            goal_handle, *control, false, release_detection_timeout_,
            release_sequence))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::RELEASE_FAILED,
                  "The button did not return to its released state.");
        }
        const auto measured_state = button_snapshot(*control);
        result->peak_position = measured_state.peak_position;
        result->estimated_force =
          measured_state.peak_position * control->spring_stiffness;
        if (!button_should_trigger) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::INSUFFICIENT_FORCE,
                  "力度不足：请求 " +
                  std::to_string(result->requested_force) +
                  " N，只能产生约 " +
                  std::to_string(button_press_depth * 1000.0) +
                  " mm 位移；按钮至少需要 " +
                  std::to_string(
                    control->spring_stiffness * control->press_threshold) +
                  " N（" +
                  std::to_string(control->press_threshold * 1000.0) +
                  " mm）才能触发。");
        }
        // The press and spring return are both verified above.  Commit before
        // safety transport so a late client cancel cannot cause a duplicate
        // physical press on retry.
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else if (is_drawer) {
        // 2026-09-03 AGENT §6（P4 重构，取代 P3-9 旧解锁臂舞/双位姿链）：
        // 打开状态机 = 单工作位姿 → 钩爪（hook）→ 支撑（support）→ 按压解锁
        // （unlock motor + service，自按下贯穿 pull）→ 双臂耦合（attach，
        // base_free=false）→ 同步拉出 → 开位确认 → 释放 → 电缸回位 → 撤回/
        // 收起；关闭时同序但跳过解锁（闩定抽屉直接推回，关位 detent 后插件
        // 自动复闩）。§6.4：已删除 unlock 四拍臂舞、unlock→ready 回退、第二
        // 套位姿（support/pregrasp/grasp）、两次 live 重算与 reseat —— 单工作
        // 位姿之后不再有任何整臂 Cartesian 链需要分支钉点，全部 rod 阶段由
        // 电缸全关节 FJT 完成，抽拉段从当前实测状态按 live 轨位分段重规划。
        const bool closing = target_position < initial_state.position;
        // 2026-09-05 AGENT doc §4.2 (P1): 任务级钩爪座封保持参考（hook →
        // support → unlock → attach → pull 贯穿；unlock/attach/pull 不发 hook
        // 目标也不重算，ref 保持自然贯穿）。正常打开路径由 seal 原位接管后
        // valid=true；legacy / physics 回退路径保持 invalid。
        DrawerHookHoldState hook_hold;
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.20F, target_position,
          "Planning both arms to the single drawer work pose (hooks open).");
        // 工作位姿以到达前实测轨位锚定（对关闭同效：抽屉在开位静止时读数
        // 不变，属无害重算）。位姿后无任何整臂链，分支种子留空 → OMPL 位姿
        // 目标 + 并发有界执行 + 内部到达稳定验证（空种子回退路径已由 /007
        // 与旧 cap4 系列实测验证）。
        {
          const double live_drawer_position =
            button_snapshot(*control).position;
          drawer_bimanual_poses = calculate_drawer_bimanual_work_poses(
            *control, live_drawer_position);
          RCLCPP_INFO(
            get_logger(),
            "Anchoring drawer '%s' bimanual work poses at live rail position "
            "%.4f m (op-start %.4f m).",
            control->id.c_str(), live_drawer_position,
            initial_state.position);
          plan_and_execute_bimanual_poses(
            drawer_left_move_group, move_group, goal_handle,
            drawer_left_tool_.move_group, drawer_right_tool_.move_group,
            drawer_bimanual_poses.left_work_pose,
            drawer_bimanual_poses.right_work_pose,
            {}, {}, &result->operation_executed, "drawer work pose",
            true /* leave_arrival_dwell: self-center 首轮测量在驻留保持下进行 */);
        }
        should_attempt_retreat = true;
        // 2026-09-05 AGENT §8.4 cap-2 v5 取证 fix #6: 到达稳定后按实测钩杆端做
        // 一次闭环自对中（仅横向残差超带才整臂再到达，见函数头注释）。
        self_center_drawer_work_pose(
          goal_handle, *control, drawer_left_move_group, move_group,
          drawer_left_tool_.move_group, drawer_right_tool_.move_group,
          button_snapshot(*control).position, drawer_bimanual_poses,
          &result->operation_executed, "drawer work pose");
        // 2026-09-03 AGENT §7.2 cap1: 单工作位姿就位取证（§8.3）——双侧杆端
        // 悬停把手外侧、全部电缸收拢、抽屉未解锁仍闩定关闭。收尾只做退让
        // 收起，不触发任何电缸（未解锁，显式复闩恒无害）。
        if (!closing && debug_stage_cap == 1) {
          finish_capped_drawer_stage(
            goal_handle, *control, drawer_left_move_group, move_group, 1,
            "both arms reached the single drawer work pose with all rod "
            "actuators retracted (drawer untouched, still latched closed).",
            drawer_bimanual_attached, result);
        }
        result->diagnostic_stage = "hook";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.30F, target_position,
          "Closing both hook fingers onto the drawer handle plates.");
        // 2026-09-03 AGENT §4.2/§6.1: 工作位姿下两侧钩爪（gripper 电缸）从
        // 收拢态闭合 —— 真实杆端自面板东侧悬停位沿杆轴向压上把手立板
        // （闩定抽屉 give≈0：杆端贴面板；自由抽屉：咬进 press_depth）。
        // support/unlock 电缸保持收拢（§6.3 hook 矩阵）。
        drive_drawer_hook_stage(
          goal_handle, *control, *move_group, &result->operation_executed,
          &hook_hold);
        // 2026-09-03 AGENT §7.2 cap2: 双侧钩爪闭合取证（§8.4）——钩爪贴合
        // 把手立板、抽屉仍闩定、支撑/解锁电缸收拢。收尾电缸回位→退让→复闩。
        if (!closing && debug_stage_cap == 2) {
          finish_capped_drawer_stage(
            goal_handle, *control, drawer_left_move_group, move_group, 2,
            "both hook fingers closed onto the handle plates at the work pose "
            "(support/unlock rods still retracted; drawer latched closed).",
            drawer_bimanual_attached, result);
        }
        result->diagnostic_stage = "support";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.38F, target_position,
          "Extending both support rods into the cabinet side seams.");
        // 2026-09-03 AGENT §4.3/§6.1: 钩爪保持闭合的同时伸两侧支撑电缸到
        // support_contact_position（杆端入左右侧缝抵缝口内缘，提供拉取反力）。
        // 本段内部复核钩爪尖端仍贴住把手（§6.3：support 矩阵保持 grasp 合同，
        // 不再有旧「support 阶段钩爪张开」的违约时序）。
        drive_drawer_support_stage(
          goal_handle, *control, *move_group, hook_hold,
          &result->operation_executed);
        // 2026-09-03 AGENT §7.2 cap3: 双侧支撑取证（§8.5）——支撑杆端入缝
        // 抵住、钩爪保持贴合、抽屉纹丝不动。收尾电缸回位→退让→显式复闩。
        if (!closing && debug_stage_cap == 3) {
          finish_capped_drawer_stage(
            goal_handle, *control, drawer_left_move_group, move_group, 3,
            "both support rods extended into the cabinet side seams while the "
            "hook fingers kept holding the handles; drawer immobile.",
            drawer_bimanual_attached, result);
        }
        if (!closing) {
          result->diagnostic_stage = "unlock";
          // Mode-neutral until the unlock service returns the plugin's
          // authoritative evidence (strict vs simulated_linkage).
          last_unlock_acceptance_mode_.clear();
          last_unlock_acceptance_message_.clear();
          last_unlock_simulated_ = false;
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::APPROACHING,
            0.44F, target_position,
            "Releasing the drawer rail latch with the right unlock rod.");
          // 2026-09-03 AGENT §3/§6.1: 解锁电机有界伸出到 pressed（经三电缸
          // 控制器全关节 FJT；读回真实电机关节 ∈ [retracted+floor, ceiling)
          // 且稳定后才放行，不达标 helper 先尽力收回再响亮抛错，绝不盲解锁）。
          // 自本段起电机保持按压贯穿整个 PULL（§6.1 PRESS_UNLOCK → 收尾
          // rods-home 之间不回撤、不移动任何整臂）。
          drive_unlock_motor(
            goal_handle, *control, *move_group,
            control->unlock_pressed_position, "extend",
            &result->operation_executed);
          // 2026-09-03 AGENT §7.2 cap4: 解锁按压取证（§8.6）——电机已伸出并
          // 读回达标，但解锁服务未调用、插件仍闩定（按压保持至收尾 teardown
          // 电缸回位时才收回）。收尾显式复闩。
          if (debug_stage_cap == 4) {
            finish_capped_drawer_stage(
              goal_handle, *control, drawer_left_move_group, move_group, 4,
              "finger-3 unlock motor extended and read back within "
              "[retracted+floor, ceiling); unlock service not yet called, "
              "rail still latched.",
              drawer_bimanual_attached, result);
          }
          // 2026-09-03 AGENT §6.1: 调用解锁服务 —— 插件复核放行（严格模式：
          // finger3 真实触点距按钮区 ≤ drawer_unlock_distance_threshold 且电机
          // 实际伸出 ≥ floor；db1 §4.2 simulated_linkage：真实电机行程 + 左右
          // 把手真实保持）后放行轨道闩，抽屉进入可拉自由态（此刻仍被双侧钩爪
          // 与支撑把持，不会漂移）。服务失败同样先尽力收回解锁电机再原样抛出。
          try {
            set_drawer_unlock(goal_handle, *control, true, true);
          } catch (...) {
            try {
              drive_unlock_motor(
                goal_handle, *control, *move_group,
                control->unlock_retracted_position, "retract",
                &result->operation_executed);
            } catch (const std::exception & retract_error) {
              RCLCPP_WARN(
                get_logger(),
                "Drawer '%s' unlock failed and the unlock-motor retract also "
                "failed: %s; subsequent arm motions risk scraping the extended "
                "probe.",
                control->id.c_str(), retract_error.what());
            }
            throw;
          }
          // 2026-09-06 AGENT doc §4.2: 放行后把插件的模式标签亮出来 ——
          // simulated_linkage 放行绝不报告为"finger3 真实端点按到了按钮"。
          if (last_unlock_simulated_) {
            RCLCPP_WARN(
              get_logger(),
              "Drawer '%s' unlock accepted under simulated_linkage: %s",
              control->id.c_str(),
              last_unlock_acceptance_message_.c_str());
            publish_operate_feedback(
              goal_handle,
              OperateCabinetControl::Feedback::APPROACHING,
              0.46F, target_position,
              "simulated_linkage: rail latch released via an unmodelled "
              "linkage (unlock motor real stroke; finger3 real tip not at "
              "the button).");
          }
          // 2026-09-03 AGENT §7.2 cap5: 真实解锁取证（§8.7）——服务放行、
          // 轨道闩已释放；解锁电机保持按压（不回撤，贯穿后续 attach/pull）。
          if (debug_stage_cap == 5) {
            finish_capped_drawer_stage(
              goal_handle, *control, drawer_left_move_group, move_group, 5,
              last_unlock_simulated_ ?
                "unlock service released the rail latch under AGENT §4.2 "
                "simulated_linkage (right unlock motor real stroke; latch "
                "release via an unmodelled linkage; finger3 real tip NOT at "
                "the button); drawer free and held by hooks and supports." :
                "unlock service released the rail latch with finger-3 still "
                "pressed at the button zone; drawer free and held by hooks "
                "and supports.",
              drawer_bimanual_attached, result);
          }
        }
        result->diagnostic_stage = "grasp";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.52F, target_position,
          "Attaching both end effectors to the drawer handles.");
        // 2026-09-03 AGENT §6.1/§6.3: 正式抽拉路径 attach 恒带 base_free=false
        // —— 插件随即创建底盘固定制动关节，反力落地，底盘在整段拉距内保持
        // docked（双臂拉拽 + 支撑电缸锚点，禁用底盘拖拽）。耦合期间插件实时
        // 校验双侧尖端仍贴住把手，脱离超时即中止耦合。
        set_drawer_bimanual_grasp(goal_handle, *control, true, false);
        drawer_bimanual_attached = true;
        interruptible_hold(goal_handle, grasp_attach_settle_duration_);
        // 2026-09-03 AGENT §7.2 caps 6/7/8（真实 attach + 耦合稳定后取证）：
        //   cap6 = 耦合保持 3 s 取证（§8.8），不移动抽屉；
        //   cap7 = 拉距封顶 0.03 m（§8.9）；cap8 = 拉距封顶 0.10 m（§8.10，
        //     越过已知 p≈0.15 自碰撞临界的一半路程，取"双臂拉拽现阶段能到
        //     多远"的实测证据）。
        // 封顶距离非 detent，用位置轮询取证（wait_for_slider_detent 对空状态
        // 名退化为纯位置判据）。cap7/8 必须传 debug_capped_pull_tolerance_
        // 紧带 —— 通用 slider_position_tolerance_ (0.03) ≥ cap7 拉距 0.03，
        // 曾让拉取被跳过、位置判据在抽屉从未移动时空转通过（假 PASS）；跳过
        // 判据同步收紧到 capped_pull_already_at_tolerance_。cap7/8 拉取 →
        // 紧带验证 → 推回 → 复闩后统一收尾（finish_capped_drawer_stage 抛出
        // DebugStageCapReached，不会返回）。
        if (!closing && debug_stage_cap >= 6 && debug_stage_cap <= 8) {
          const double cap6_hold_seconds = 3.0;
          const double capped_pull = debug_stage_cap == 7 ? 0.03 :
            (debug_stage_cap == 8 ? 0.10 : 0.0);
          double measured_pull = 0.0;
          double measured_return = 0.0;
          if (debug_stage_cap == 6) {
            publish_operate_feedback(
              goal_handle,
              OperateCabinetControl::Feedback::MANIPULATING,
              0.60F, 0.0,
              "Holding the bimanual coupling engaged at the closed detent "
              "(AGENT §7.2 cap6).");
            interruptible_hold(goal_handle, cap6_hold_seconds);
          }
          if (capped_pull > 0.0) {
            publish_operate_feedback(
              goal_handle,
              OperateCabinetControl::Feedback::MANIPULATING,
              0.62F, capped_pull,
              "Pulling the drawer to the capped commissioning distance "
              "(AGENT §7.2).");
            pull_drawer_by_arm_waypoints(
              drawer_left_move_group, move_group, goal_handle, *control,
              capped_pull, &result->operation_executed,
              capped_pull_already_at_tolerance_);
            if (!wait_for_slider_detent(
                goal_handle, *control, "", capped_pull,
                door_settle_timeout_, debug_capped_pull_tolerance_,
                &measured_pull))
            {
              throw GenericOperationError(
                      OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                      "The drawer did not reach the capped pull distance of " +
                      std::to_string(capped_pull) +
                      " m (measured rail position " +
                      std::to_string(measured_pull) + " m, verify band +-" +
                      std::to_string(debug_capped_pull_tolerance_) +
                      " m; AGENT §7.2 stage " +
                      std::to_string(debug_stage_cap) + ").");
            }
            publish_operate_feedback(
              goal_handle,
              OperateCabinetControl::Feedback::MANIPULATING,
              0.74F, 0.0,
              "Pushing the capped drawer back to its closed detent.");
            pull_drawer_by_arm_waypoints(
              drawer_left_move_group, move_group, goal_handle, *control,
              0.0, &result->operation_executed,
              capped_pull_already_at_tolerance_);
            if (!wait_for_slider_detent(
                goal_handle, *control, "", 0.0, door_settle_timeout_,
                debug_capped_pull_tolerance_, &measured_return))
            {
              throw GenericOperationError(
                      OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                      "The capped drawer did not return to the closed detent "
                      "after the AGENT §7.2 push-back (measured rail "
                      "position " + std::to_string(measured_return) +
                      " m, verify band +-" +
                      std::to_string(debug_capped_pull_tolerance_) + " m).");
            }
          }
          finish_capped_drawer_stage(
            goal_handle, *control, drawer_left_move_group, move_group,
            debug_stage_cap,
            debug_stage_cap == 6 ?
            std::string("bimanual coupling held engaged for ") +
            std::to_string(cap6_hold_seconds) +
            " s at the closed detent without drawer motion, then released." :
            std::string("bimanual pull to ") + std::to_string(capped_pull) +
            " m verified by the strict rail-position gate: measured " +
            std::to_string(measured_pull) + " m on the pull (band +-" +
            std::to_string(debug_capped_pull_tolerance_) + " m), measured " +
            std::to_string(measured_return) +
            " m on the push-back, then re-latched.",
            drawer_bimanual_attached, result);
        }
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          closing ? 0.72F : 0.62F, target_position,
          closing ?
          "Pushing the drawer closed with both arms." :
          "Pulling the drawer open with both arms.");
        // 2026-09-03 AGENT §6.3/§13.2（用户定夺）：正式抽拉/推回 = 双臂同步
        // waypoint 拉拽（pull_drawer_by_arm_waypoints）—— §4 支撑电缸抵住
        // 把手附近柜体提供反力，双臂在抽屉轴上折叠拉取，插件线性耦合让
        // 抽屉 1:1 跟随双侧尖端；拉距每段先两臂各自规划再同步时长执行，
        // 失败按完成 waypoint 折算到达位置响亮上报（不再使用 P3-8 的底盘
        // grab-and-drive，函数保留作兜底参考）。
        pull_drawer_by_arm_waypoints(
          drawer_left_move_group, move_group, goal_handle, *control,
          target_position, &result->operation_executed);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.84F, target_position,
          "Verifying that the drawer reached its requested detent.");
        if (!wait_for_slider_detent(
            goal_handle, *control, target_state, target_position,
            door_settle_timeout_))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                  "The drawer did not reach the requested detent '" +
                  target_state + "' after the bimanual arm pull.");
        }
        result->diagnostic_stage = "release";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RELEASING,
          0.90F, target_position,
          "Releasing the bimanual drawer grasp at the requested detent.");
        set_drawer_bimanual_grasp(goal_handle, *control, false);
        drawer_bimanual_attached = false;
        result->grasp_released = true;
        // 2026-09-03 AGENT §4.4: 解锁/support/gripper 电缸全部回位后再收臂，
        // 否则机械臂退让会把仍伸出的杆端拖过抽屉前脸（best-effort：回位失败
        // 只告警，不阻断主流程，抽屉此刻已被目标 detent 闩锁）。
        best_effort_drawer_rods_home(
          goal_handle, *control, *move_group, &result->operation_executed);
        result->diagnostic_stage = "retreat";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.94F, target_position,
          "Retreating both arms before stowing.");
        best_effort_bimanual_retreat_and_stow(
          drawer_left_move_group, move_group, *control,
          &result->operation_executed);
        should_attempt_retreat = false;
        const auto drawer_state = button_snapshot(*control);
        result->peak_position = drawer_state.peak_position;
        result->button_triggered = target_state != initial_state.state_id;
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else if (is_slider && control->grasp_operated) {
        // 正面抽拉抽屉：与 door/knob 的 grasp 流程同构，但为平移关节 --
        // ready 停在面板前方 prepress，精确规划到 near-grasp pregrasp 后用短
        // 直线逼近前脸，set_control_grasp(true) 把探针刚性地附着到面板，再沿
        // 滑动轴拖拽到目标 detent（拉出/推回），验证、释放、撤回。slider 无
        // 过中点释放机制，拖拽即是全部行程。
        const bool closing = target_position < initial_state.position;
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the drawer ready pose.");
        plan_and_execute_pose(
          *move_group, goal_handle, drawer_poses.ready_pose,
          contact_tool_link_, &result->operation_executed, control.get());
        should_attempt_retreat = true;
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.43F, target_position,
          "Approaching the drawer panel grasp point.");
        // 同 door/knob：先精确规划到 near-grasp pregrasp，避免 ready-pose 的
        // IK 分支无法完成长直线内推（r_arm_0 <-> r_arm_2 自碰撞）。
        plan_and_execute_pose(
          *move_group, goal_handle, drawer_poses.pregrasp_pose,
          contact_tool_link_, &result->operation_executed, control.get());
        wait_for_pregrasp_controls_stable(
          goal_handle, *control, pregrasp_stability_references,
          std::chrono::steady_clock::now());
        execute_cartesian_path(
          *move_group, goal_handle, {drawer_poses.grasp_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          0.99, &result->operation_executed);
        result->diagnostic_stage = "grasp";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.52F, target_position,
          "Attaching the probe at the drawer panel front face.");
        set_control_grasp(goal_handle, control->id, true);
        grasp_attached = true;
        interruptible_hold(goal_handle, grasp_attach_settle_duration_);
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.62F, target_position,
          closing ?
          "Dragging the drawer panel closed to its detent." :
          "Pulling the drawer panel open to its detent.");
        // 分段直线拖拽（同 door 长弧的分段机制）：每段从实测机器人状态重启，
        // 避免整段 Cartesian 插值在 IK 分支中途失效。
        std::size_t drawer_completed_waypoints = 0U;
        execute_segmented_cartesian_path(
          *move_group, goal_handle, drawer_waypoints,
          static_cast<std::size_t>(door_cartesian_segment_waypoints_),
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          drawer_completed_waypoints, &result->operation_executed);
        if (std::abs(target_position - initial_state.position) >
          target_tolerance_)
        {
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.71F, target_position,
            "Verifying that the drawer panel reached its requested detent.");
          if (!wait_for_slider_detent(
              goal_handle, *control, target_state, target_position,
              press_detection_timeout_))
          {
            throw GenericOperationError(
                    OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                    "The drawer panel did not reach the requested detent '" +
                    target_state + "'.");
          }
        }
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.74F, target_position,
          "Holding the target detent briefly before grasp release.");
        interruptible_hold(goal_handle, grasp_release_settle_duration_);
        result->diagnostic_stage = "release";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RELEASING,
          0.78F, target_position,
          "Releasing the drawer grasp at the requested detent.");
        set_control_grasp(goal_handle, control->id, false);
        grasp_attached = false;
        result->grasp_released = true;
        result->diagnostic_stage = "retreat";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.84F, target_position,
          "Retreating before verifying the released drawer.");
        const auto drawer_retreat_pose = calculate_drawer_tool_pose(
          *control, target_position, prepress_distance_);
        execute_cartesian_path(
          *move_group, goal_handle, {drawer_retreat_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, &result->operation_executed);
        should_attempt_retreat = false;
        result->diagnostic_stage = "verification";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.90F, target_position,
          "Verifying that the drawer panel settled at its detent.");
        if (!wait_for_slider_detent(
            goal_handle, *control, target_state, target_position,
            door_settle_timeout_))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::RELEASE_FAILED,
                  "The drawer panel did not latch at the requested detent.");
        }
        const auto drawer_state = button_snapshot(*control);
        result->peak_position = drawer_state.peak_position;
        result->button_triggered = target_state != initial_state.state_id;
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else if (is_slider) {
        const bool closing = target_position < initial_state.position;
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the slider ready pose.");
        execute_slider_ready_approach_press(
          *move_group, goal_handle, result, slider_poses, closing,
          target_position, *control, &result->operation_executed);
        should_attempt_retreat = true;
        // The ready-branch retry inside the helper covers PLANNING failures
        // (OMPL can land on an IK branch from which the 0->target push cannot
        // be planned; the button type in B3a got a good branch by luck).
        // Force tracking then covers shortfalls that survive a good branch
        // (detent friction): it measures the PHYSICAL panel position and
        // re-pushes from the current face until it converges.  Only the open
        // push needs it -- a close is a push-past-detent release where the
        // panel springs away from the tool as soon as the release trips, and
        // re-pushing there would re-open the panel.
        if (!closing) {
          result->diagnostic_stage = "manipulation";
          // The open force-tracking loop must converge on the SAME
          // compensated target as the initial press: without the offset it
          // re-pushes toward the raw 0.3 detent, whose re-push peak (commanded
          // ~0.301 plus ~0.03 tool over-travel = ~0.332) can cross the
          // push-push release and spring the panel shut (boot16 open).  Both
          // the initial press and the force-tracking convergence share
          // slider_open_press_offset so the open peak stays below the release
          // with margin.
          track_slider_force(
            *move_group, goal_handle, *control,
            target_position - control->slider_open_press_offset,
            &result->operation_executed);
        }
        // Let the detent spring settle the panel before verifying; the tip is
        // still pressed against the face, so the panel is at the target.
        interruptible_hold(goal_handle, press_hold_seconds_);
        if (!closing) {
          result->diagnostic_stage = "verification";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::VERIFYING,
            0.77F, target_position,
            "Verifying the slider panel reached its target detent.");
          if (!wait_for_slider_detent(
              goal_handle, *control, target_state, target_position,
              press_detection_timeout_))
          {
            throw GenericOperationError(
                    OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                    "The slider panel did not reach the requested detent '" +
                    target_state + "'.");
          }
          interruptible_hold(goal_handle, press_hold_seconds_);
        }
        result->diagnostic_stage = "retreat";
        // For a close the tip must clear the closing panel, so the retreat
        // waypoints anchor on the CLOSED face rather than the release face;
        // the panel follows the retreating tip and settles at the closed
        // detent once the tip is out of its travel.
        const OperationPoses retreat_poses = closing ?
          calculate_slider_operation_poses(
          *control, target_position, target_position) :
          slider_poses;
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.85F, 0.0,
          closing ?
          "Retracting clear of the closing panel; the detent spring returns "
          "it to the closed detent." :
          "Retracting the probe from the slider panel.");
        if (closing) {
          // A close is a push-past-detent release: once the release trips, the
          // 80 kg panel must spring back to the closed detent on its own, which
          // requires the tip to clear the panel's entire travel.  The earlier
          // Cartesian retreat over that ~0.36 m westward sweep failed planning
          // at ~85% on every close -- the tool cannot hold its press orientation
          // over the long sweep, and the failure aborts BEFORE executing, so the
          // tip stayed pressed against the panel and held it at the release for
          // the whole 90 s settle window (boot20 close).  A joint-space OMPL
          // retreat to the closed-face prepress is far more robust: the planning
          // scene keeps the panel as free space ("按净空规划", only db1's own
          // face cylinder, exempted during operation, lives near the path), so
          // only the single end pose must be reachable, not every intermediate
          // orientation.  The retreat is a get-clear motion; its completion is
          // only a means to the real check, the detent latch below, so a blocked
          // retreat must not fail the operation: log it and verify the panel
          // instead.  If the tool is genuinely still inside the panel's travel,
          // that verification times out and the operation fails accurately.
          try {
            plan_and_execute_pose(
              *move_group, goal_handle, retreat_poses.prepress_pose,
              contact_tool_link_, &result->operation_executed, control.get());
          } catch (const OperationError & retreat_error) {
            RCLCPP_WARN(
              get_logger(),
              "Slider close retreat could not complete (%s); the panel must "
              "still latch closed on its own -- proceeding to detent "
              "verification.",
              retreat_error.what());
          }
        } else {
          execute_cartesian_path(
            *move_group, goal_handle,
            {retreat_poses.contact_pose, retreat_poses.prepress_pose},
            cartesian_velocity_scale_, cartesian_acceleration_scale_,
            0.99, &result->operation_executed);
        }
        should_attempt_retreat = false;
        // The bistable detent must hold the panel after the tool leaves; a
        // spring-back to the source detent means the latch never engaged.  A
        // close's panel is pushed past the release (to ~0.36, riding up to the
        // joint limit) before it springs back, and the 80 kg panel coasts back
        // slowly (spring force ~4 N vs 0.2 N friction, terminal ~0.14 m/s) --
        // it can take many seconds to cross the midpoint and flip state_id to
        // closed.  The 3 s press/release detection timeouts are button-scale;
        // use the door-scale settle timeout so the verification does not race
        // the spring-back (boot17 close: panel reached 0.008 but the 3 s
        // window expired mid-return).
        result->diagnostic_stage = "verification";
        if (!wait_for_slider_detent(
            goal_handle, *control, target_state, target_position,
            door_settle_timeout_))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::RELEASE_FAILED,
                  closing ?
                  "The slider panel did not return to the closed detent." :
                  "The slider panel did not latch at the requested detent.");
        }
        const auto measured_state = button_snapshot(*control);
        result->peak_position = measured_state.peak_position;
        result->button_triggered =
          target_state != initial_state.state_id;
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else {
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the control ready pose.");
        // Runtime branch selection: a knob's ready/pregrasp plans must land on
        // an IK family whose whole Cartesian chain (50 mm approach + the
        // manipulation arc) is feasible from the REAL dock TF.  The configured
        // seed alone is not enough -- it can fail the arc via r_arm_0<->r_arm_2
        // self-collision or a wrist joint limit.  Doors keep their segmented
        // arc mechanism and the configured seed unchanged.
        std::vector<double> rotary_branch_seed;
        const std::vector<double> * rotary_branch_seed_ptr = nullptr;
        if (control->control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
        {
          // The post-release retreat target (knob angle at the manipulation
          // position, prepress standoff) must match the retreat the operator
          // actually executes after releasing the grasp.
          const auto retreat_pose = calculate_rotary_tool_pose(
            *control, rotary_manip_pos, prepress_distance_, false,
            rotary_tool_roll_offset);
          rotary_branch_seed = select_rotary_branch_seed(
            *move_group, *control, rotary_poses.pregrasp_pose,
            rotary_poses.grasp_pose, rotary_arc_waypoints, retreat_pose,
            contact_tool_link_);
          if (!rotary_branch_seed.empty()) {
            rotary_branch_seed_ptr = &rotary_branch_seed;
          }
        }
        plan_and_execute_pose(
          *move_group, goal_handle, rotary_poses.ready_pose,
          contact_tool_link_, &result->operation_executed, control.get(),
          rotary_branch_seed_ptr);
        should_attempt_retreat = true;
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.43F, target_position,
          "Approaching the physical grasp point.");
        if (control->control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR ||
          control->control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
        {
          // A pose-only OMPL plan can select a ready-pose IK branch that
          // cannot finish the final inward Cartesian approach.  Require an
          // exact collision-checked plan to a near-grasp pose first, then
          // preserve a short straight-line final approach.  Knobs share the
          // failure: the left-detent ready pose lands on an r_arm branch
          // whose 94 mm straight approach self-collides r_arm_0 <-> r_arm_2;
          // a 50 mm final approach is clean on every branch.  The knob
          // pregrasp itself must sit far enough out that the pose-planned
          // motion to it cannot sweep the clamped blade (see the
          // knob_pregrasp_clearance comment); doors keep their 10 mm variant.
          // The pregrasp plan must use the same calibrated IK seed as the
          // ready plan: an unseeded OMPL plan can pick a different IK branch
          // (observed: r_arm_2 = +0.89) whose 50 mm approach self-collides at
          // ~25 % before the blade clamp, while every seed-derived branch is
          // clean over the full distance.  For knobs the seed is the runtime
          // branch picked by select_rotary_branch_seed (same as the ready
          // plan); doors keep their configured seed.
          plan_and_execute_pose(
            *move_group, goal_handle, rotary_poses.pregrasp_pose,
            contact_tool_link_, &result->operation_executed, control.get(),
            rotary_branch_seed_ptr);
        }
        wait_for_pregrasp_controls_stable(
          goal_handle, *control, pregrasp_stability_references,
          std::chrono::steady_clock::now());
        execute_cartesian_path(
          *move_group, goal_handle, {rotary_poses.grasp_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          0.99, &result->operation_executed);
        result->diagnostic_stage = "grasp";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.52F, target_position,
          "Attaching the probe at the verified near-distance grasp point.");
        set_control_grasp(goal_handle, control->id, true);
        grasp_attached = true;
        // Let the transient grasp-active notification reach the planar
        // stabilizer before the arm starts driving the physical constraint.
        interruptible_hold(goal_handle, grasp_attach_settle_duration_);
        // Doors and calibrated knobs are over-center mechanisms.  Moving
        // safely beyond the next midpoint lets the physical detent spring
        // finish the requested travel after release, without demanding an
        // unreachable full-angle Cartesian wrist arc.  These were computed
        // before the ready pose plan so the branch selection could validate
        // the same arc against the real dock TF.
        const double & manipulation_position = rotary_manip_pos;
        const auto & waypoints = rotary_arc_waypoints;
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.62F, target_position,
          "Following the control joint arc without commanding the joint.");
        if (control->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR)
        {
          // A long door arc can keep MoveIt's Cartesian interpolation on an
          // IK branch that becomes invalid near the end of the sweep.  Short
          // complete segments preserve physical continuity while allowing
          // each next segment to start from the measured robot state.
          door_arc_progress.in_progress = true;
          door_arc_progress.completed_waypoints = 0U;
          door_arc_progress.total_waypoints = waypoints.size();
          execute_segmented_cartesian_path(
            *move_group, goal_handle, waypoints,
            static_cast<std::size_t>(door_cartesian_segment_waypoints_),
            cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            door_arc_progress.completed_waypoints,
            &result->operation_executed);
          door_arc_progress.in_progress = false;
        } else {
          execute_cartesian_path(
            *move_group, goal_handle, waypoints,
            cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            0.99, &result->operation_executed);
        }
        if (control->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR &&
          std::abs(target_position - initial_state.position) >
          target_tolerance_)
        {
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.71F, target_position,
            "Verifying that the physical door crossed its safe release "
            "position.");
          wait_for_door_release_position(
            goal_handle, *control, initial_state.position,
            manipulation_position, std::chrono::steady_clock::now());
        }
        const bool uses_partial_knob_release =
          control->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB &&
          control->detent_release_fraction<1.0 &&
            std::abs(target_position - initial_state.position)>
          target_tolerance_;
        if (uses_partial_knob_release) {
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.71F, target_position,
            "Verifying that the physical knob latched the requested "
            "adjacent detent.");
          wait_for_knob_release_position(
            goal_handle, *control, initial_state.position, target_position,
            target_state, std::chrono::steady_clock::now());
        }
        const auto release_hold_started = std::chrono::steady_clock::now();
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.74F, target_position,
          "Holding the target detent briefly before grasp release.");
        interruptible_hold(goal_handle, grasp_release_settle_duration_);
        if (control->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR &&
          std::abs(target_position - initial_state.position) >
          target_tolerance_)
        {
          // A single threshold crossing is not sufficient if the physical
          // door rebounds during the release hold.  Require a new sample on
          // the safe side immediately before detaching the grasp.
          wait_for_door_release_position(
            goal_handle, *control, initial_state.position,
            manipulation_position, release_hold_started);
        }
        if (uses_partial_knob_release) {
          // Require a fresh target-side sample immediately before detach so a
          // transient crossing cannot be mistaken for a latched detent.
          wait_for_knob_release_position(
            goal_handle, *control, initial_state.position, target_position,
            target_state, release_hold_started);
        }
        const bool is_door = control->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
        const double release_clearance = is_door ?
          door_release_clearance_ : prepress_distance_;
        const auto target_ready = calculate_rotary_tool_pose(
          *control, manipulation_position, release_clearance, false,
          rotary_tool_roll_offset);
        std::chrono::steady_clock::time_point released_at;
        if (is_door) {
          // Compliant door grasping couples only tool rotation to the hinge.
          // Keep that coupling active while translating the tool beyond the
          // remaining door sweep.  Releasing first lets the closer drive the
          // visible panel through the retreating arm.
          result->diagnostic_stage = "retreat";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RETREATING,
            0.78F, target_position,
            "Holding the door angle while clearing its remaining sweep.");
          execute_cartesian_path(
            *move_group, goal_handle, {target_ready},
            cartesian_velocity_scale_, cartesian_acceleration_scale_,
            0.99, &result->operation_executed);
          should_attempt_retreat = false;
          result->diagnostic_stage = "release";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RELEASING,
            0.84F, target_position,
            "Releasing the door after the tool cleared its sweep.");
          set_control_grasp(goal_handle, control->id, false);
          grasp_attached = false;
          result->grasp_released = true;
          released_at = std::chrono::steady_clock::now();
        } else {
          result->diagnostic_stage = "release";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RELEASING,
            0.78F, target_position,
            "Releasing the physical grasp at the requested detent.");
          set_control_grasp(goal_handle, control->id, false);
          grasp_attached = false;
          result->grasp_released = true;
          released_at = std::chrono::steady_clock::now();
          result->diagnostic_stage = "retreat";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RETREATING,
            0.84F, target_position,
            "Retreating before verifying the released control.");
          execute_cartesian_path(
            *move_group, goal_handle, {target_ready},
            cartesian_velocity_scale_, cartesian_acceleration_scale_,
            0.99, &result->operation_executed);
          should_attempt_retreat = false;
        }
        result->diagnostic_stage = "verification";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.90F, target_position,
          "Waiting for articulated parent controls to settle.");
        wait_for_parent_controls_stable(
          goal_handle, *control, expected_parent_states);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.94F, target_position,
          "Verifying target position and low-velocity stability after "
          "release.");
        wait_for_target_stable(
          goal_handle, *control, target_position, target_state, released_at);
        // The released knob/switch/door is stable at its requested detent.
        // This is the physical side-effect commit; transport remains a
        // fallible safety phase but is no longer client-cancelable.
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      }

      result->diagnostic_stage = "transport";
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::RETREATING,
        0.98F, target_position,
        "Returning the arm to its safe transport target.");
      plan_and_execute_named_target(
        *move_group, goal_handle, transport_named_target_,
        &result->operation_executed, true);
      result->transport_succeeded = true;

      const auto final_state = button_snapshot(*control);
      if (!result->operation_executed) {
        throw GenericOperationError(
                OperateCabinetControl::Result::INTERNAL_ERROR,
                "Cabinet operation reached final verification without "
                "issuing a physical motion command.");
      }
      result->success = true;
      result->error_code = OperateCabinetControl::Result::SUCCESS;
      result->message = "Operated " + control->id + " successfully.";
      // ``diagnostic_stage`` describes the origin of a physical-operation
      // failure.  Keep successful physical results empty so clients do not
      // mistake them for planning-only validation diagnostics.
      result->diagnostic_stage.clear();
      result->final_position = final_state.position;
      result->peak_position = final_state.peak_position;
      result->final_state = final_state.state_id;
      if (is_button) {
        result->estimated_force =
          final_state.peak_position * control->spring_stiffness;
        result->button_triggered =
          result->button_triggered ||
          final_state.peak_position + 1.0e-9 >= control->press_threshold;
      }
      result->physical_outcome_confirmed = true;
      result->final_state_verified = true;
      result->transport_succeeded = true;
      result->recovery_succeeded = true;
      result->grasp_released = true;
      request_success = true;
    } catch (const DebugStageCapReached & cap) {
      // 2026-09-03 AGENT §7.2: 封顶成功交付，不是失败。封顶分支已在抛出前
      // 完成脱离/电缸回位/退让收起/轨道闩重锁的完整安全序列，这里只塑造结果
      // 并落入共享收尾 —— 不得调用 recover_failed_operate_goal（会重复收尾）。
      // 若收尾期间恰好撞上取消/租约丢失，仍按失败交付。
      if (operation_lease_lost_.load()) {
        result->success = false;
        result->error_code = OperateCabinetControl::Result::LEASE_LOST;
        result->message =
          "The global robot operation lease was lost during the capped-stage "
          "cleanup; all motion was stopped.";
      } else if (is_operate_goal_canceling(goal_handle)) {
        result->success = false;
        result->error_code = OperateCabinetControl::Result::CANCELED;
        result->message =
          "Cabinet operation was canceled during the capped-stage cleanup.";
      } else {
        if (validation_only) {
          snapshot_planning_only_result(result, control);
        } else {
          const auto cap_state = button_snapshot(*control);
          result->success = true;
          result->error_code = OperateCabinetControl::Result::SUCCESS;
          // 成功路径惯例：diagnostic_stage 清空（不冒充规划校验诊断），
          // 封顶证据全部留在 message 中供现场取证。
          result->diagnostic_stage.clear();
          result->final_position = cap_state.position;
          result->peak_position = cap_state.peak_position;
          result->final_state = cap_state.state_id;
          result->physical_outcome_confirmed = true;
          result->final_state_verified = true;
          result->transport_succeeded = true;
          result->recovery_succeeded = true;
          result->grasp_released = true;
        }
        result->message = std::string("debug_stage_cap=") +
          std::to_string(cap.stage) + " reached: " + cap.what();
        request_success = true;
        // 物理结果已发生（电缸/臂/闩均有动作），照常提交活跃目标物理结果；
        // 提交失败（罕见竞态）只告警，封顶属调试路径，不再改写结果。
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          RCLCPP_WARN(
            get_logger(),
            "Could not commit the physical outcome of the capped drawer "
            "stage %d; finishing as success anyway (AGENT §7.2 debug).",
            cap.stage);
        }
      }
    } catch (const GenericOperationError & error) {
      result->success = false;
      result->error_code = error.error_code;
      result->message = error.what();
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group, goal_handle);
      }
    } catch (const OperationError & error) {
      result->success = false;
      result->error_code = map_legacy_error_code(error.error_code);
      result->message = error.what();
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group, goal_handle);
      }
    } catch (const std::exception & error) {
      result->success = false;
      result->error_code = operation_lease_lost_.load() ?
        OperateCabinetControl::Result::LEASE_LOST :
        is_operate_goal_canceling(goal_handle) ?
        OperateCabinetControl::Result::CANCELED :
        OperateCabinetControl::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        validation_only ?
        "The global robot operation lease was lost; planning-only validation "
        "was canceled without publishing a physical motion command." :
        "The global robot operation lease was lost; all motion was stopped." :
        is_operate_goal_canceling(goal_handle) ?
        "Cabinet operation was canceled." :
        std::string("Cabinet operation failed: ") + error.what();
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group, goal_handle);
      }
    } catch (...) {
      result->success = false;
      result->error_code = operation_lease_lost_.load() ?
        OperateCabinetControl::Result::LEASE_LOST :
        OperateCabinetControl::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        validation_only ?
        "The global robot operation lease was lost; planning-only validation "
        "was canceled without publishing a physical motion command." :
        "The global robot operation lease was lost; all motion was stopped." :
        "Cabinet operation failed with an unknown error.";
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group, goal_handle);
      }
    }

    clear_latched_cabinet_transform();
    const bool owned_physical_motion_resources =
      active_goal_owns_physical_motion_resources(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id());
    if (owned_physical_motion_resources) {
      stop_active_motion();
      cancel_active_navigation();
      if (preparation_policy.enter_manual_base_mode) {
        request_navigation_mode_without_wait(false);
      }
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    relinquish_active_goal_physical_motion_resources(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id());
    release_operation_lease_noexcept();
    publish_active_control("");
    finish_operate_goal_noexcept(goal_handle, result, request_success);
    clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
    operation_active_.store(false);
  }

  std::pair<double, std::string> resolve_operation_target(
    const ButtonSpec & control,
    const OperateCabinetControl::Goal & goal,
    const ButtonSnapshot & current) const
  {
    if (control.control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
    {
      return {
        control.state_positions.back(), control.state_ids.back()};
    }
    if (goal.command == OperateCabinetControl::Goal::COMMAND_SET_STATE) {
      const auto iterator = std::find(
        control.state_ids.begin(), control.state_ids.end(), goal.target_state);
      if (iterator == control.state_ids.end()) {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Unknown target state '" + goal.target_state + "'.");
      }
      const auto index = static_cast<std::size_t>(
        std::distance(control.state_ids.begin(), iterator));
      return {control.state_positions[index], control.state_ids[index]};
    }
    if (goal.command == OperateCabinetControl::Goal::COMMAND_SET_POSITION) {
      if (!goal.use_target_position || !std::isfinite(goal.target_position) ||
        goal.target_position < control.min_position ||
        goal.target_position > control.max_position)
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Target position is outside the configured control limits.");
      }
      std::string closest_state;
      double closest_error = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < control.state_positions.size();
        ++index)
      {
        const double error = std::abs(
          goal.target_position - control.state_positions[index]);
        if (error < closest_error) {
          closest_error = error;
          closest_state = control.state_ids[index];
        }
      }
      if (closest_error > target_tolerance_) {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Only configured physical detent positions are supported.");
      }
      const auto iterator = std::find(
        control.state_ids.begin(), control.state_ids.end(), closest_state);
      const auto index = static_cast<std::size_t>(
        std::distance(control.state_ids.begin(), iterator));
      return {control.state_positions[index], closest_state};
    }
    if (goal.command == OperateCabinetControl::Goal::COMMAND_TOGGLE) {
      std::size_t current_index = 0U;
      const auto state_iterator = std::find(
        control.state_ids.begin(), control.state_ids.end(), current.state_id);
      if (state_iterator != control.state_ids.end()) {
        current_index = static_cast<std::size_t>(
          std::distance(control.state_ids.begin(), state_iterator));
      } else {
        double closest_error = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < control.state_positions.size();
          ++index)
        {
          const double error = std::abs(
            current.position - control.state_positions[index]);
          if (error < closest_error) {
            closest_error = error;
            current_index = index;
          }
        }
      }
      const std::size_t target_index =
        (current_index + 1U) % control.state_positions.size();
      return {
        control.state_positions[target_index],
        control.state_ids[target_index]};
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
            "The requested command is not supported by this control.");
  }

  std::size_t control_state_index(
    const ButtonSpec & control,
    const std::string & state_id) const
  {
    const auto iterator = std::find(
      control.state_ids.begin(), control.state_ids.end(), state_id);
    if (iterator == control.state_ids.end()) {
      throw GenericOperationError(
              OperateCabinetControl::Result::INTERNAL_ERROR,
              "Control '" + control.id +
              "' resolved an unknown target detent '" + state_id + "'.");
    }
    return static_cast<std::size_t>(
      std::distance(control.state_ids.begin(), iterator));
  }

  std::size_t control_state_index(
    const ButtonSpec & control,
    const ButtonSnapshot & state) const
  {
    const auto iterator = std::find(
      control.state_ids.begin(), control.state_ids.end(), state.state_id);
    if (iterator != control.state_ids.end()) {
      return static_cast<std::size_t>(
        std::distance(control.state_ids.begin(), iterator));
    }
    return static_cast<std::size_t>(std::distance(
             control.state_positions.begin(),
             std::min_element(
               control.state_positions.begin(), control.state_positions.end(),
               [&state](double left, double right) {
                 return std::abs(state.position - left) <
                 std::abs(state.position - right);
               })));
  }

  tf2::Transform lookup_cabinet_transform_to(
    const std::string & source_frame,
    const tf2::Duration & timeout) const
  {
    if (require_cabinet_pose_valid_ &&
      (!cabinet_pose_valid_received_.load() || !cabinet_pose_valid_.load()))
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The cabinet pose authority has no fresh valid pose.");
    }
    try {
      const auto transform = transform_buffer_->lookupTransform(
        source_frame, cabinet_frame_, tf2::TimePointZero, timeout);
      tf2::Transform result;
      tf2::fromMsg(transform.transform, result);
      result.getRotation().normalize();
      // 2026-09-04 fix#3 v5 (cap-3 seal 根因, TF 锁存兜底路径): AMCL z 伪影
      // 消除。规划系 odom ≠ 导航系 map 时 odom→cabinet 必经 map→odom; AMCL
      // 2D 以 z≈0 初始位姿建边, 恒留 ~-0.55 m z 伪影 → 全部命令抬 ~0.55 m。
      // 真值 odom→cabinet.z == map→cabinet.z: 物理 odom 地面 = map 地面
      // (controller 里程边 odom→body z = 车体真高), 导航系→柜体为
      // pose-authority 静态直连边 (不经 AMCL)。xy/yaw 保留 AMCL 水平信念
      // (驶入/导航流所需); 同帧场景 (odom 锚定共享柜, nav == planning)
      // 不适用本修正。
      if (source_frame == planning_frame_ &&
        navigation_frame_ != planning_frame_)
      {
        try {
          const auto nav_cab = transform_buffer_->lookupTransform(
            navigation_frame_, cabinet_frame_, tf2::TimePointZero, timeout);
          tf2::Transform nav_cab_transform;
          tf2::fromMsg(nav_cab.transform, nav_cab_transform);
          result.getOrigin().setZ(nav_cab_transform.getOrigin().z());
        } catch (const tf2::TransformException & z_error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Cabinet AMCL z-artifact correction lookup '" +
                  navigation_frame_ + "' -> '" + cabinet_frame_ +
                  "' failed: " + z_error.what());
        }
      }
      return result;
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cabinet TF '" + source_frame + "' -> '" + cabinet_frame_ +
              "' is unavailable: " + error.what());
    }
  }

  tf2::Transform lookup_cabinet_transform(
    const tf2::Duration & timeout = tf2::durationFromSec(0.2)) const
  {
    return lookup_cabinet_transform_to(planning_frame_, timeout);
  }

  std::string cabinet_verify_frame() const
  {
    return cabinet_verify_frame_.empty() ?
      planning_frame_ : cabinet_verify_frame_;
  }

  // 2026-09-05 fix#cap2 (cap-2 根因终结): 物理锚定。latch 出的 odom→cabinet 在
  // map 锚定夹具（pose_parent_frame: map, 几何声明于 map 恒等）上途经
  // AMCL map→odom（信念 δ 每轮不同, 实测 0-7 mm+）或站位锚定的车身落地误差
  // (2-6 mm), 两路都会把 δ 打进全部 odom 帧命令 → 右钩 6-7 mm 北偏、phase-3
  // 右杆骑缝塌缩。夹具几何连杆 (b1 抽屉本体等) 是 gazebo world==odom 的直接
  // 实体, 测其规划系真位姿、减去 cabinet 系声明局部原点, 即得夹具系原点的规划
  // 系真值 —— 不经任何信念/落地链。rails/工作位姿 = 修正后 odom→cabinet ⊗
  // 局部坐标, 精确落在物理板面中心。仅当配置了实体且实体测量成功时修正; 否则
  // 保留 latch 原值 (真机无 /get_entity_state 或非夹具场景零回归)。
  std::optional<tf2::Vector3> measure_physics_anchor_entity_origin()
  {
    if (drawer_entity_client_ == nullptr || physics_anchor_entity_.empty()) {
      return std::nullopt;
    }
    if (!drawer_entity_measure_available_ &&
      !drawer_entity_client_->service_is_ready() &&
      !drawer_entity_client_->wait_for_service(100ms))
    {
      if (!physics_anchor_measure_warned_) {
        physics_anchor_measure_warned_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Cabinet physics-anchor service '%s' unavailable; keeping the "
          "latched odom->cabinet unchanged (AMCL/landing bias may remain).",
          drawer_entity_client_->get_service_name());
      }
      return std::nullopt;
    }
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = physics_anchor_entity_;
    auto future = drawer_entity_client_->async_send_request(request);
    if (future.wait_for(2s) != std::future_status::ready) {
      if (!physics_anchor_measure_warned_) {
        physics_anchor_measure_warned_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Cabinet physics-anchor query for '%s' timed out; keeping the "
          "latched odom->cabinet unchanged.",
          physics_anchor_entity_.c_str());
      }
      return std::nullopt;
    }
    const auto response = future.get();
    if (response == nullptr || !response->success) {
      if (!physics_anchor_measure_warned_) {
        physics_anchor_measure_warned_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Cabinet physics-anchor query for '%s' failed; keeping the latched "
          "odom->cabinet unchanged.",
          physics_anchor_entity_.c_str());
      }
      return std::nullopt;
    }
    const auto & pose = response->state.pose;
    const auto & p = pose.position;
    const auto & o = pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.z) ||
      !std::isfinite(o.w))
    {
      return std::nullopt;
    }
    drawer_entity_measure_available_ = true;
    return tf2::Vector3(p.x, p.y, p.z);
  }

  // 把物理锚定原点覆写到待锁存的 odom→cabinet（保持其旋转不变）。
  // 返回 true = 已应用物理锚定; false = 未配置 / 测不到实体, latch 原值保留。
  bool apply_physics_anchor_to_latch(tf2::Transform & cabinet_to_planning)
  {
    if (!physics_anchor_configured_) {
      return false;
    }
    const auto measured = measure_physics_anchor_entity_origin();
    if (!measured.has_value()) {
      return false;
    }
    const tf2::Vector3 corrected_origin = *measured -
      tf2::quatRotate(
        cabinet_to_planning.getRotation(), physics_anchor_local_origin_);
    const tf2::Vector3 previous_origin = cabinet_to_planning.getOrigin();
    const double shift = corrected_origin.distance(previous_origin);
    cabinet_to_planning.setOrigin(corrected_origin);
    RCLCPP_INFO(
      get_logger(),
      "Cabinet physics-anchored to entity '%s': odom->cabinet origin moved "
      "%.1f mm (%.4f, %.4f, %.4f) -> (%.4f, %.4f, %.4f); rails/work poses now "
      "track the measured fixture geometry.",
      physics_anchor_entity_.c_str(), shift * 1000.0,
      previous_origin.x(), previous_origin.y(), previous_origin.z(),
      corrected_origin.x(), corrected_origin.y(), corrected_origin.z());
    return true;
  }

  void latch_cabinet_transform()
  {
    auto transform = lookup_cabinet_transform(
      tf2::durationFromSec(system_wait_timeout_));
    const auto verify_transform = lookup_cabinet_transform_to(
      cabinet_verify_frame(), tf2::durationFromSec(system_wait_timeout_));
    apply_physics_anchor_to_latch(transform);
    std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
    latched_cabinet_transform_ = transform;
    latched_cabinet_verify_transform_ = verify_transform;
    cabinet_transform_latched_ = true;
  }

  // 2026-09-04 fix#3 (cap-3 root cause): 站位锚定锁存。几何门以「真实杆端 ↔
  // 真实把手」为据, 而 TF 锁存 odom→cabinet 途经 map→odom (AMCL 信念), 每轮
  // 的 belief 误差 δ (实测 0-10 mm) 会被打进所有 odom 帧命令: 两臂共享锚点
  // 漂移, 而 TF 树内的 tip/target 同时过 AMCL 相互抵消, 几何门对 δ 不可见
  // (v8 的 2.6 mm PASS 与 cap3-r1 的 9.9 mm FAIL 就是同一代码的两轮 δ 抽签)。
  // 本函数把 odom→cabinet 改建于遥移真值: 驱动脚本把车体精确遥移到站位 pose
  // (gazebo world = map 权威, 数学与 preposition_base.py / staging 同源), 车
  // 体停驻不动, 于是 odom→cabinet = T_odom→body (实时里程, 不经 AMCL) ⊗
  // (站位 body pose)⁻¹ ⊗ T_map→cabinet (静态夹具锚)。命令落点从此与 δ 无关,
  // 只剩机械臂执行误差。仅在 AMCL 粗门证明车体确在站位 (xy ≤ 0.12 m、yaw ≤
  // 0.30 rad) 时锚定; 门内失败返回 false, 由调用方退回原 TF 锁存 —— Nav2
  // 驶入 / 精确对接 / 站位外 operate 等非遥移流全部照旧。z 不入门: 遥移与
  // AMCL 重置在 map→odom 的 z 上各自留 artifact, 与水平放置无关。
  bool try_station_anchor_latch(const ButtonSpec & control)
  {
    if (!control.navigation_station) {
      return false;
    }
    const auto & station = *control.navigation_station;
    const std::string nav_frame = station.frame_id.empty() ?
      navigation_frame_ : station.frame_id;
    // 锚定公式假定「车体真值 = 站位 pose」, 仅在驱动方把车体遥移到导航帧站位
    // 的流里成立; 非导航帧站位 (odom 锚定的共享柜体) 一律退回 TF 锁存。
    if (nav_frame != navigation_frame_) {
      return false;
    }
    constexpr double kPi = 3.14159265358979323846;
    try {
      // 站位 body pose = calculate_control_staging_poses 同源数学 (无 drawer
      // shift; local 经 cabinet 全四元数旋转; body_yaw = base_yaw + π/2)。
      tf2::Transform cabinet_nav;
      {
        const auto tf = transform_buffer_->lookupTransform(
          nav_frame, cabinet_frame_, tf2::TimePointZero,
          tf2::durationFromSec(system_wait_timeout_));
        tf2::fromMsg(tf.transform, cabinet_nav);
        cabinet_nav.getRotation().normalize();
      }
      const tf2::Vector3 local_position =
        station.local_anchor + station.outward_axis * station.standoff;
      const tf2::Vector3 nav_position = cabinet_nav * local_position;
      tf2::Vector3 nav_outward = tf2::quatRotate(
        cabinet_nav.getRotation(), station.outward_axis);
      const double horizontal_norm = std::hypot(
        nav_outward.x(), nav_outward.y());
      if (!std::isfinite(horizontal_norm) || horizontal_norm <= 1.0e-9) {
        return false;
      }
      nav_outward /= nav_outward.length();
      tf2::Quaternion body_yaw_rotation;
      body_yaw_rotation.setRPY(
        0.0, 0.0,
        std::atan2(-nav_outward.y(), -nav_outward.x()) +
        station.base_yaw_offset + kPi / 2.0);
      body_yaw_rotation.normalize();
      // 2026-09-04 fix#3 v5 (cap-3 seal 根因): 站位 body pose 必须取 3D。
      // 旧实现 origin z = 0 (nav_position 2D 楼层投影) 而 odom_body z = 车体
      // 物理高 (~0.549), 锚定 z = 0.549 - 0 + map→cab.z 残留车体高度 —— 全部
      // 命令抬 ~0.55 m, 工作位姿杆端悬在柜面 z≈1.5 自由空间, seal 永远无接触
      // 力证据 (v9-v11 三败根因)。平坦场景 controller 里程边 (不经 AMCL) 的
      // odom→body z = 车体真高 = 停驻时 map z (物理 odom 地面 = map 地面), 故
      // 站位 origin z := odom_body.z() 使锚定 z 精确抵消为 map→cab 设计 z。
      tf2::Transform odom_body;
      {
        const auto tf = transform_buffer_->lookupTransform(
          planning_frame_, docking_base_frame_, tf2::TimePointZero,
          tf2::durationFromSec(system_wait_timeout_));
        tf2::fromMsg(tf.transform, odom_body);
        odom_body.getRotation().normalize();
      }
      tf2::Vector3 station_origin(nav_position);
      station_origin.setZ(odom_body.getOrigin().z());
      const tf2::Transform body_at_station(body_yaw_rotation, station_origin);

      // AMCL 粗门: 导航帧里车体现在 (belief) 必须贴着站位, 否则车不在站位
      // (Nav2 驶入中 / 对接错位 / 站位外操作), 锚定公式不适用。
      tf2::Transform belief_body;
      {
        const auto tf = transform_buffer_->lookupTransform(
          nav_frame, docking_base_frame_, tf2::TimePointZero,
          tf2::durationFromSec(system_wait_timeout_));
        tf2::fromMsg(tf.transform, belief_body);
        belief_body.getRotation().normalize();
      }
      constexpr double kStationAnchorGateXY = 0.12;   // m
      constexpr double kStationAnchorGateYaw = 0.30;  // rad
      const tf2::Vector3 displacement =
        body_at_station.getOrigin() - belief_body.getOrigin();
      const double xy_error = std::hypot(
        displacement.x(), displacement.y());
      const double yaw_error = body_at_station.getRotation().
        angleShortestPath(belief_body.getRotation());
      if (xy_error > kStationAnchorGateXY ||
        yaw_error > kStationAnchorGateYaw)
      {
        RCLCPP_WARN(
          get_logger(),
          "Cabinet '%s' station anchor not used: belief is %.3f m / %.3f rad "
          "off the '%s' station (gates %.2f m / %.2f rad); falling back to "
          "the TF latch.",
          control.id.c_str(), xy_error, yaw_error, nav_frame.c_str(),
          kStationAnchorGateXY, kStationAnchorGateYaw);
        return false;
      }

      // odom→cabinet = T_odom→body ⊗ (body_at_station)⁻¹ ⊗ T_map→cabinet:
      // 与 δ 无关, 命令落点 = 真值站位上的真实柜体。z 在 3D 站位下精确抵消
      // (odom_body.z − body_at_station.z + map→cab.z = 设计 z)。
      tf2::Transform anchored =
        odom_body * body_at_station.inverse() * cabinet_nav;
      // fix#cap2: 站位锚定/遥移真值仍带车身落地误差 (2-6 mm), 经配置实体物理
      // 锚定覆写原点后彻底与 landing 误差解耦（保持旋转）。
      apply_physics_anchor_to_latch(anchored);
      const auto verify_transform = lookup_cabinet_transform_to(
        cabinet_verify_frame(), tf2::durationFromSec(system_wait_timeout_));
      std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
      latched_cabinet_transform_ = anchored;
      latched_cabinet_verify_transform_ = verify_transform;
      cabinet_transform_latched_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Cabinet '%s' latched to the '%s' station anchor (belief %.1f mm / "
        "%.1f deg off the teleport truth): odom->cabinet is now AMCL-free.",
        control.id.c_str(), nav_frame.c_str(), xy_error * 1000.0,
        yaw_error * 180.0 / kPi);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(
        get_logger(),
        "Cabinet '%s' station anchor lookup failed (%s); falling back to the "
        "TF latch.",
        control.id.c_str(), error.what());
      return false;
    }
  }

  void clear_latched_cabinet_transform() noexcept
  {
    std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
    cabinet_transform_latched_ = false;
  }

  tf2::Transform resolve_cabinet_transform()
  {
    {
      std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
      if (cabinet_transform_latched_) {
        return latched_cabinet_transform_;
      }
    }
    return lookup_cabinet_transform();
  }

  void verify_latched_cabinet_transform() const
  {
    tf2::Transform latched;
    {
      std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
      if (!cabinet_transform_latched_) {
        return;
      }
      latched = latched_cabinet_verify_transform_;
    }
    const auto current = lookup_cabinet_transform_to(
      cabinet_verify_frame(), tf2::durationFromSec(0.05));
    const double translation_change =
      latched.getOrigin().distance(current.getOrigin());
    const double rotation_change = latched.getRotation().angleShortestPath(
      current.getRotation());
    if (translation_change > cabinet_pose_translation_tolerance_ ||
      rotation_change > cabinet_pose_rotation_tolerance_)
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cabinet pose moved by " + std::to_string(translation_change) +
              " m / " + std::to_string(rotation_change) +
              " rad during operation; motion was stopped.");
    }
  }

  std::vector<const ButtonSpec *> control_ancestors(
    const ButtonSpec & control) const
  {
    std::vector<const ButtonSpec *> ancestors;
    std::string parent_id = control.parent_control_id;
    while (!parent_id.empty()) {
      const auto iterator = buttons_by_id_.find(parent_id);
      if (iterator == buttons_by_id_.end()) {
        throw GenericOperationError(
                OperateCabinetControl::Result::INTERNAL_ERROR,
                "Control '" + control.id + "' has an unresolved parent '" +
                parent_id + "'.");
      }
      ancestors.push_back(iterator->second.get());
      parent_id = iterator->second->parent_control_id;
    }
    std::reverse(ancestors.begin(), ancestors.end());
    return ancestors;
  }

  tf2::Transform resolve_control_parent_transform(
    const ButtonSpec & control,
    bool require_stable = true) const
  {
    const auto ancestors = control_ancestors(control);

    tf2::Transform result;
    result.setIdentity();
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start +
      std::chrono::duration<double>(system_wait_timeout_);
    for (const auto * ancestor : ancestors) {
      auto state = button_snapshot(*ancestor);
      // 父控制（如按钮的旋钮）在按压/重解析几何的瞬间可能正处于物理回落中
      // （工具拖带或重力回中）。失败即抛会把一次瞬态判成 NOT_READY；这里改为
      // 轮询等待其稳定（≤ system_wait_timeout_），稳定后按最终落位重建几何。
      // 旋钮是自由转动的展示件，不要求停在特定档位，仅要求"不再运动"。
      auto fresh = [&]() {
        return structured_control_state_is_usable(
          state.structured_received, state.valid, true,
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            state.structured_received_at).count() <=
          button_state_timeout_);
      };
      if (require_stable) {
        while (!(fresh() && !state.in_motion &&
          std::abs(state.velocity) <= stable_velocity_tolerance_))
        {
          if (std::chrono::steady_clock::now() >= deadline) {
            throw GenericOperationError(
                    OperateCabinetControl::Result::NOT_READY,
                    "Parent control '" + ancestor->id +
                    "' did not become stable before the operation "
                    "geometry was needed.");
          }
          std::this_thread::sleep_for(50ms);
          state = button_snapshot(*ancestor);
        }
      } else if (!fresh()) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Parent control '" + ancestor->id +
                "' does not have a fresh valid physical state.");
      }

      tf2::Quaternion rotation;
      rotation.setRotation(ancestor->axis, state.position);
      rotation.normalize();
      const tf2::Vector3 pivot(
        ancestor->pivot_x, ancestor->pivot_y, ancestor->pivot_z);
      tf2::Transform joint_motion;
      joint_motion.setRotation(rotation);
      joint_motion.setOrigin(pivot - tf2::quatRotate(rotation, pivot));
      result *= joint_motion;
    }
    return result;
  }

  ResolvedControlGeometry resolve_control_geometry(
    const ButtonSpec & control,
    bool require_stable_parent = true) const
  {
    const tf2::Transform parent = resolve_control_parent_transform(
      control, require_stable_parent);
    ResolvedControlGeometry geometry{
      parent * tf2::Vector3(
        control.local_x, control.local_y, control.local_z),
      parent * tf2::Vector3(
        control.pivot_x, control.pivot_y, control.pivot_z),
      tf2::quatRotate(parent.getRotation(), control.axis),
      tf2::quatRotate(parent.getRotation(), control.approach_normal)};
    geometry.axis.normalize();
    geometry.approach_normal.normalize();
    return geometry;
  }

  tf2::Quaternion tool_rotation_from_outward(
    const tf2::Vector3 & outward) const
  {
    tf2::Vector3 tool_z = outward.normalized();
    tf2::Vector3 tool_x(0.0, 0.0, 1.0);
    tool_x -= tool_z * tool_z.dot(tool_x);
    if (tool_x.length2() < 1.0e-6) {
      tool_x = tf2::Vector3(1.0, 0.0, 0.0);
      tool_x -= tool_z * tool_z.dot(tool_x);
    }
    tool_x.normalize();
    tf2::Vector3 tool_y = tool_z.cross(tool_x);
    tool_y.normalize();
    const tf2::Matrix3x3 basis(
      tool_x.x(), tool_y.x(), tool_z.x(),
      tool_x.y(), tool_y.y(), tool_z.y(),
      tool_x.z(), tool_y.z(), tool_z.z());
    tf2::Quaternion rotation;
    basis.getRotation(rotation);
    rotation.normalize();
    return rotation;
  }

  geometry_msgs::msg::Pose calculate_rotary_tool_pose(
    const ButtonSpec & control,
    double position,
    double outward_offset,
    bool require_stable_parent = true,
    double tool_roll_offset = 0.0)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(
      control, require_stable_parent);
    tf2::Quaternion local_rotation;
    local_rotation.setRotation(geometry.axis, position);
    local_rotation.normalize();
    const tf2::Vector3 local_grasp = geometry.pivot +
      tf2::quatRotate(
      local_rotation, geometry.grasp_zero - geometry.pivot);
    const tf2::Vector3 grasp = cabinet * local_grasp;
    const tf2::Vector3 outward_zero = tf2::quatRotate(
      cabinet.getRotation(), geometry.approach_normal);
    const tf2::Vector3 world_axis = tf2::quatRotate(
      cabinet.getRotation(), geometry.axis);
    tf2::Quaternion world_rotation;
    world_rotation.setRotation(world_axis, position);
    world_rotation.normalize();
    const tf2::Vector3 outward = tf2::quatRotate(
      world_rotation, outward_zero).normalized();
    // Pincer tools (rotate_button, rocker) carry their jaws/rotor on local +Z,
    // so +Z must point at the control for the wrist to stay out of the cabinet.
    // The offset direction (outward) remains unchanged so retraction and the
    // prepress standoff still move away from the control.
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward_zero : outward_zero;
    const tf2::Quaternion tool_zero =
      tool_rotation_from_outward(tool_axis_reference);
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(tf2::Vector3(0.0, 0.0, 1.0), tool_roll_offset);
    tool_roll.normalize();
    // Post-multiply so the calibration rotates around contact-tool local +Z.
    // This moves passive sibling tools within the panel plane while preserving
    // the tool's outward axis and the calibrated contact point.
    tf2::Quaternion tool_rotation = world_rotation * tool_zero * tool_roll;
    tool_rotation.normalize();

    geometry_msgs::msg::Pose pose;
    // Place the measured 3-D business point, rather than the contact-link
    // origin, on the desired grasp point.  This is essential for off-axis
    // fingers whose empty tool centre must never be aimed at the control.
    const tf2::Vector3 desired_tip_position =
      grasp + outward * outward_offset;
    const tf2::Vector3 position_world = desired_tip_position -
      tf2::quatRotate(tool_rotation, tool_tip_position_);
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  ControlStagingPoses calculate_control_staging_poses(
    const ButtonSpec & control)
  {
    if (!control.navigation_station) {
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Control '" + control.id +
              "' has no explicit robot-adapter navigation station.");
    }
    const auto & station = *control.navigation_station;

    // Resolve the cabinet transform in the navigation frame (typically
    // "map") so the station position is authoritative for Nav2 and not
    // subject to AMCL map→odom drift.  Then transform to the planning
    // frame for MoveIt.
    const std::string nav_frame =
      station.frame_id.empty() ? navigation_frame_ : station.frame_id;
    tf2::Transform cabinet_nav;
    try {
      const auto tf = transform_buffer_->lookupTransform(
        nav_frame, cabinet_frame_, tf2::TimePointZero,
        tf2::durationFromSec(system_wait_timeout_));
      tf2::fromMsg(tf.transform, cabinet_nav);
      cabinet_nav.getRotation().normalize();
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cabinet TF '" + nav_frame + "' -> '" + cabinet_frame_ +
              "' is unavailable for staging pose: " + error.what());
    }

    // Bimanual drawers slide along drawer_axis: the current position moves the
    // panel face (and therefore the grasp targets) along that axis.  From the
    // standard dock an OPEN drawer puts the wrist inside the robot body, so the
    // ready planning self-collides (reach to the open wrist collapses while the
    // body stays put).  Re-measure the standoff from the CURRENT face instead
    // by shifting the dock along the slide axis by the live drawer position;
    // this preserves the closed-drawer reach and keeps the wrist clear of the
    // body exactly as in the closed case.  Zero for a closed drawer, so the
    // open flow (and every non-drawer control) keeps the configured dock.
    const double drawer_dock_shift =
      is_drawer_type(control.control_type) ?
      button_snapshot(control).position : 0.0;
    const tf2::Vector3 local_position = station.local_anchor +
      station.outward_axis * station.standoff +
      control.drawer_axis * drawer_dock_shift;
    const tf2::Vector3 nav_position = cabinet_nav * local_position;
    tf2::Vector3 nav_outward = tf2::quatRotate(
      cabinet_nav.getRotation(), station.outward_axis);
    const double horizontal_norm = std::hypot(
      nav_outward.x(), nav_outward.y());
    if (!std::isfinite(horizontal_norm) || horizontal_norm <= 1.0e-9) {
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Control '" + control.id +
              "' navigation station has no horizontal outward direction.");
    }
    nav_outward /= nav_outward.length();

    // Build the navigation pose in the nav frame.
    ControlStagingPoses poses;
    poses.navigation_pose.header.frame_id = nav_frame;
    poses.navigation_pose.pose.position.x = nav_position.x();
    poses.navigation_pose.pose.position.y = nav_position.y();
    poses.navigation_pose.pose.position.z = nav_position.z();
    tf2::Quaternion nav_rotation;
    nav_rotation.setRPY(
      0.0, 0.0,
      std::atan2(-nav_outward.y(), -nav_outward.x()) +
      station.base_yaw_offset);
    nav_rotation.normalize();
    poses.navigation_pose.pose.orientation = to_message(nav_rotation);

    // Build the planning pose in the planning frame (for MoveIt).
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 planning_position = cabinet * local_position;
    tf2::Vector3 planning_outward = tf2::quatRotate(
      cabinet.getRotation(), station.outward_axis);
    planning_outward.normalize();

    poses.planning_pose.header.frame_id = planning_frame_;
    poses.planning_pose.pose.position.x = planning_position.x();
    poses.planning_pose.pose.position.y = planning_position.y();
    poses.planning_pose.pose.position.z = planning_position.z();
    tf2::Quaternion planning_rotation;
    planning_rotation.setRPY(
      0.0, 0.0,
      std::atan2(-planning_outward.y(), -planning_outward.x()) +
      station.base_yaw_offset);
    planning_rotation.normalize();
    poses.planning_pose.pose.orientation = to_message(planning_rotation);

    return poses;
  }

  RotaryOperationPoses calculate_rotary_operation_poses(
    const ButtonSpec & control,
    double position,
    double tool_roll_offset = 0.0)
  {
    RotaryOperationPoses poses;
    poses.ready_pose = calculate_rotary_tool_pose(
      control, position, prepress_distance_, true, tool_roll_offset);
    // Knobs need a far pregrasp so the pose-planned motion to it stays clear
    // of the blade; doors keep the small shared clearance they were tuned with.
    const double pregrasp_clearance =
      control.control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB ?
      knob_pregrasp_clearance_ : rotary_pregrasp_clearance_;
    poses.pregrasp_pose = calculate_rotary_tool_pose(
      control, position,
      control.grasp_outward_offset + pregrasp_clearance, true,
      tool_roll_offset);
    poses.grasp_pose = calculate_rotary_tool_pose(
      control, position, control.grasp_outward_offset, true,
      tool_roll_offset);
    return poses;
  }

  /**
   * Place the measured business point on a sliding drawer front face.
   *
   * A prismatic drawer differs from a rotary control: the grasp point does not
   * swing about a pivot, it translates linearly with the panel.  The tool
   * orientation stays fixed (outward along the face normal) and only the tip
   * rides the slide axis -- local_grasp = grasp_zero + axis * position.  The
   * planner-facing geometry (grasp_zero, axis, approach_normal) comes from the
   * same resolved control geometry as the rotary path, so the front station
   * and the tool calibration are unchanged.
   */
  geometry_msgs::msg::Pose calculate_drawer_tool_pose(
    const ButtonSpec & control,
    double position,
    double outward_offset,
    double tool_roll_offset = 0.0)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(control, true);
    const tf2::Vector3 local_grasp =
      geometry.grasp_zero + geometry.axis * position;
    const tf2::Vector3 grasp = cabinet * local_grasp;
    const tf2::Vector3 outward = tf2::quatRotate(
      cabinet.getRotation(), geometry.approach_normal).normalized();
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward : outward;
    const tf2::Quaternion tool_zero =
      tool_rotation_from_outward(tool_axis_reference);
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(tf2::Vector3(0.0, 0.0, 1.0), tool_roll_offset);
    tool_roll.normalize();
    tf2::Quaternion tool_rotation = tool_zero * tool_roll;
    tool_rotation.normalize();

    geometry_msgs::msg::Pose pose;
    // Place the measured 3-D business point, rather than the contact-link
    // origin, on the desired grasp point (same convention as the rotary tool).
    const tf2::Vector3 desired_tip_position =
      grasp + outward * outward_offset;
    const tf2::Vector3 position_world = desired_tip_position -
      tf2::quatRotate(tool_rotation, tool_tip_position_);
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  RotaryOperationPoses calculate_drawer_operation_poses(
    const ButtonSpec & control,
    double position,
    double tool_roll_offset = 0.0)
  {
    // Same ready/pregrasp/grasp shape as the rotary flow: the pose-planned
    // pregrasp keeps the final inward approach short so any IK branch can
    // complete it (see the ready-branch comment in the grasp flow).
    RotaryOperationPoses poses;
    poses.ready_pose = calculate_drawer_tool_pose(
      control, position, prepress_distance_, tool_roll_offset);
    poses.pregrasp_pose = calculate_drawer_tool_pose(
      control, position,
      control.grasp_outward_offset + rotary_pregrasp_clearance_,
      tool_roll_offset);
    poses.grasp_pose = calculate_drawer_tool_pose(
      control, position, control.grasp_outward_offset, tool_roll_offset);
    return poses;
  }

  std::vector<geometry_msgs::msg::Pose> calculate_drawer_waypoints(
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    double tool_roll_offset = 0.0)
  {
    // A linear drag from the panel's current slide position to the target
    // detent, sampled at drawer_waypoint_step_ so each segment is short enough
    // to re-plan from the measured robot state (same rationale as the door
    // segmented arc).  The tool is rigidly attached to the panel, so the panel
    // follows the tool tip exactly along the slide axis.
    const double travel = target_position - initial_position;
    const std::size_t count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(std::abs(travel) / drawer_waypoint_step_)));
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(count);
    for (std::size_t index = 1U; index <= count; ++index) {
      const double ratio = static_cast<double>(index) /
        static_cast<double>(count);
      waypoints.push_back(calculate_drawer_tool_pose(
        control, initial_position + travel * ratio,
        control.grasp_outward_offset, tool_roll_offset));
    }
    return waypoints;
  }

  const BimanualToolProfile & drawer_tool_profile(DrawerSide side) const
  {
    return side == DrawerSide::LEFT ? drawer_left_tool_ : drawer_right_tool_;
  }

  tf2::Vector3 drawer_side_point(
    const ButtonSpec & control, DrawerSide side) const
  {
    switch (side) {
      case DrawerSide::LEFT:
        if (!control.has_left_handle_point) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Drawer '" + control.id +
                  "' is missing its left handle point contract.");
        }
        return control.left_handle_point;
      case DrawerSide::RIGHT:
        if (!control.has_right_handle_point) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Drawer '" + control.id +
                  "' is missing its right handle point contract.");
        }
        return control.right_handle_point;
    }
    return tf2::Vector3(0.0, 0.0, 0.0);
  }

  std::string drawer_side_name(DrawerSide side) const
  {
    return side == DrawerSide::LEFT ? "left" : "right";
  }

  tf2::Quaternion drawer_tool_rotation(
    const tf2::Vector3 & outward, DrawerSide side) const
  {
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward : outward;
    const tf2::Quaternion tool_zero =
      tool_rotation_from_outward(tool_axis_reference);
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(
      tf2::Vector3(0.0, 0.0, 1.0),
      drawer_tool_profile(side).tool_roll_offset);
    tool_roll.normalize();
    tf2::Quaternion rotation = tool_zero * tool_roll;
    rotation.normalize();
    return rotation;
  }

  static tf2::Vector3 drawer_rod_point_at_position(
    const tf2::Vector3 & point_at_zero,
    double zero_position, double commanded_position)
  {
    return point_at_zero + tf2::Vector3(
      0.0, 0.0, -(commanded_position - zero_position));
  }

  tf2::Vector3 drawer_gripper_work_point(
    const BimanualToolProfile & profile) const
  {
    if (!profile.has_gripper_role ||
      profile.gripper_grasp_position <= profile.gripper_open_position)
    {
      return profile.tool_tip_position;
    }
    return drawer_rod_point_at_position(
      profile.gripper_contact_point_base_at_zero,
      profile.gripper_open_position, profile.gripper_grasp_position);
  }

  /**
   * Place one side's measured tool business point on the drawer.
   *
   * A bimanual drawer differs from the legacy single-arm drawer flow in that
   * the left and right tools have different business-point offsets and
   * approach the two handles from their own arm chains.  The drawer slides
   * along drawer_axis (P0: +X, toward the robot); at rail position p the
   * side's handle point sits at side_point + axis*p in cabinet frame.  The
   * tool orientation convention matches calculate_drawer_tool_pose (tool axis
   * along the outward normal), and the pose compensates the side's own
   * business-point offset so the measured tip lands on the target.
   */
  geometry_msgs::msg::Pose calculate_drawer_side_tool_pose(
    const ButtonSpec & control,
    DrawerSide side,
    double position,
    double outward_offset)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 axis = tf2::quatRotate(
      cabinet.getRotation(), control.drawer_axis).normalized();
    const tf2::Vector3 outward = tf2::quatRotate(
      cabinet.getRotation(), control.approach_normal).normalized();
    const tf2::Vector3 side_point = drawer_side_point(control, side);
    // cap2 几何根因 fix#3 v3: + drawer_pose_aim_offset 补偿执行横向偏差（见
    // BimanualToolProfile 注释）, 使真实杆端落在把手中心、几何门恢复真校验。
    const tf2::Vector3 local_target = side_point + axis * position +
      drawer_tool_profile(side).drawer_pose_aim_offset;
    const tf2::Vector3 target = cabinet * local_target;
    const tf2::Quaternion tool_rotation = drawer_tool_rotation(outward, side);

    geometry_msgs::msg::Pose pose;
    const auto & profile = drawer_tool_profile(side);
    const tf2::Vector3 business_point = drawer_gripper_work_point(profile);
    const tf2::Vector3 desired_tip_position =
      target + outward * outward_offset;
    const tf2::Vector3 position_world = desired_tip_position -
      tf2::quatRotate(tool_rotation, business_point);
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  // 2026-09-03 AGENT §6.4 (P4 重构): 单一工作位姿族。drawer 流程不再存在
  // 第二套位姿 —— ready/support/pregrasp/grasp 与右臂 unlock 位姿全部废止：
  // 双臂从停靠位一次规划到达唯一工作位姿（语义 = 旧 grasp 位姿，杆端锚定
  // 面板前缘把手点、西向压入量 = drawer_grasp_contact_offset），到达时钩爪
  // 电缸收拢、真实尖端悬于把手立板外侧 ~10mm（杆端沿杆轴向未伸出的让量）；
  // 随后 hook → support → unlock 全部是 rod 电缸在全关节 FJT 下的直线伸出，
  // 不再移动任何整臂。旧 calculate_drawer_support_pose / calculate_drawer_
  // unlock_pose / unlock_press_pose(_at_depth) / drawer_live_rail 与解锁
  // 整臂接近-回退函数族已按 §6.4 删除。
  struct DrawerBimanualPoses
  {
    geometry_msgs::msg::Pose left_work_pose;
    geometry_msgs::msg::Pose right_work_pose;
  };

  // The grasp pose and the pull hold each tool tip pressed INTO its handle
  // plate by drawer_grasp_press_depth (west of the riser face), so the plugin's
  // tight attach gate and coupling keepalive see genuine contact, not a hover.
  // 2026-09-02: the press is saturated to the drawer's available westward
  // give.  Pressing west only "bites" if the free drawer can yield west to
  // meet the pose.  At / near the closed rail hard stop (position ~0) a full
  // press pose is physically unreachable and the arm controller aborts on
  // state tolerance (run evidence 2026-09-02) -- keep at least
  // drawer_grasp_closed_floor_ of closed-side travel as slack and let the tips
  // stop AT the plate face instead.
  double drawer_grasp_contact_offset(
    const ButtonSpec & control, double position) const
  {
    const double available_give =
      std::max(0.0, position - drawer_grasp_closed_floor_);
    return -std::min(control.drawer_grasp_press_depth, available_give);
  }

  // 2026-09-03 AGENT §6.1/§6.4 (P4): 唯一工作位姿对。双侧都取"杆端锚定把手
  // 立板"语义 —— outward offset = drawer_grasp_contact_offset（西向压入量按
  // 可用让量饱和，见上）。位姿以调用方传入的轨位锚定（分支到达前的 live
  // 读数）；到达时钩爪电缸仍收拢、真实尖端沿杆轴向悬于把手外侧 ~10mm，
  // 随后的 hook 电缸闭合才真正咬合把手。
  DrawerBimanualPoses calculate_drawer_bimanual_work_poses(
    const ButtonSpec & control,
    double position)
  {
    DrawerBimanualPoses poses;
    poses.left_work_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::LEFT, position,
      drawer_grasp_contact_offset(control, position));
    poses.right_work_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::RIGHT, position,
      drawer_grasp_contact_offset(control, position));
    return poses;
  }

  // 2026-09-05 AGENT §8.4 cap-2 v5 取证 fix #6: 密封钩爪抽屉首次到达后的实测
  // 自对中（闭环，替代按 boot 反复重标 aim）。v5 取证：到达验证（位姿容差
  // ≤0.0055 m）通过后，真实钩杆端相对把手中心的横向残差仍可随冷启动/换 IK
  // 分支游走 ±5-7 mm —— R 侧 z-aim −12 mm 实际只执行约 −5 mm、y 却 +7 mm
  // 北漂，钩杆骑上右立板北缘，30.7 N 顶死在板面前 ~16 mm（压不上面、还在
  // seat-pin 期间边磨边让位 → settle 必败）；静态 aim 追不上这种按 boot/分支
  // 漂移的执行残差，0.008 m 几何门在 S13 三次冷启动下不可靠。解法：到达稳定
  // 后立即量双侧钩杆端（fix#7 起取 gazebo 物理真值，见 drawer_physics_link_point；
  // v7 现场闭环 FK 在此位姿有 ~17 mm z 幻影偏差，FK 测量只会把真实钩杆越推越
  // 远），求相对把手中心、垂直轨轴的横向残差；超带（2.5 mm）才把残差反平移进
  // 工作位姿靶点并做构型锁定关节微动修正（fix#12：不再整臂再到达——整臂
  // 重到达的 IK 会跳构型谷，两构型间 FK-物理幻影差使毫米级修正落空 1-13 mm，
  // 见函数内 6760 行附近 fix#12 详注）。
  // 沿轨轴（= 压贴轴向）的分量由压贴/电缸吸收，不修正。有界：单轮修正 ≤20
  // mm、至多 2 轮；超限照实失败（NOT_READY，收尾走退让收起，绝不带偏压贴）。
  // 仅双侧密封钩爪且带把手合同点的抽屉进入，其余抽屉/场景零回归；拉中分段
  // 到达不调用本函数（钩已咬合，压入量是刻意偏移，不得"对中"）。
  template<typename GoalHandleT>
  void self_center_drawer_work_pose(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const std::string & left_group_name,
    const std::string & right_group_name,
    double rail_position,
    DrawerBimanualPoses & poses,
    bool * operation_executed,
    const std::string & description)
  {
    const bool hooks_sealed = drawer_left_tool_.has_gripper_role &&
      drawer_right_tool_.has_gripper_role &&
      drawer_left_tool_.gripper_grasp_position > 0.0 &&
      drawer_right_tool_.gripper_grasp_position > 0.0;
    if (!hooks_sealed || !control.has_left_handle_point ||
      !control.has_right_handle_point)
    {
      return;
    }
    // 2026-09-05 fix#14 (cap2 run A/B 取证, §4.6 先取证再调参): 收敛带放宽到
    // 5 mm。dwell 主动保持下钩杆端仍以 ~±2-3 mm 横向晃动 —— 测量窗逐轴 spread
    // 现场实测 2.6-6.5 mm（run B round-2 R 侧 6.5 mm），1 s 中位数窗只能把真偏
    // 平均到 ~spread/2 的相位残差；旧 2.5 mm 带与测量自身分辨率同量级，已对中
    // 臂被噪声误判超带：run A/B 第二轮 L 0.1/0.3 mm 干净收敛、R 0.1/2.9 mm，
    // 后者的 2.9 mm 是晃动中位数相位偏置而非真脱靶（同窗长窗中位数差 2.3 mm），
    // 2.5 mm 带立在噪声地板上会逐次翻摆、cap2 ×3 无法稳定。真实脱靶（round-1
    // 未修正残差）实测 ≥6 mm（run A L 11.2 / R 8.5，run B L 6.1 / R 10.2 mm）
    // —— 5 mm 带仍拦得住真脱靶，只放行"钩已压到立板面上"的带内量。钩爪是否真
    // 密封由其后 phase-1 力地板 + 3 s 保持 + 顶开上限这些真安全断言把关（§8.2
    // 一律不动），本带只负责挡住明显的整体脱靶（≥6 mm 起）。超带仍按
    // clamp_step 修正（轮数预算见下方 fix#22，3 轮后仍超带照旧硬失败）。
    constexpr double kSelfCenterBand = 0.0050;      // m: 收敛带（横向残差范数）
    constexpr double kSelfCenterStepLimit = 0.020;  // m: 单轮修正上限
    // 2026-09-06 fix#22 (plate 时代 cap5 取证): 预算 2→3 轮。bjiq9qs75
    // (s42d attempt#1) 自对中 round-1 左 z −24.8mm 超单步 20mm 上限 → 只修正
    // 一次 → round-2 左 5.48mm 仍超 5mm 带被拦（R 0.009mm 已收敛）。残差全程
    // 单调收敛（25.4→5.5mm），只是 2 轮预算饿死收敛算法：首轮被 clamp 截断的
    // 超带脱靶永远差一轮。带 5mm/步长 20mm/各门限一律未动（非 §8.4 放宽），仅
    // 放行"收敛途中差一轮"的单调情形；3 轮后仍超带照旧硬失败并提示重标 aim。
    constexpr int kSelfCenterMaxIterations = 3;
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 axis_tf = tf2::quatRotate(
      cabinet.getRotation(), control.drawer_axis);
    const Eigen::Vector3d axis_world(axis_tf.x(), axis_tf.y(), axis_tf.z());
    const double axis_length2 = axis_world.squaredNorm();
    if (axis_length2 < 1e-12) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id +
              "' has a degenerate drawer_axis; cannot self-center its work "
              "pose.");
    }
    // 量出真实钩杆端并投影出横向（垂直轨轴、立板面内）残差。与几何门同源：
    // 同一 gripper 接触连杆/局部偏移，同一 rail-point 合同，同一 rail 位置。
    // 钩杆端优先取物理真值（drawer_physics_link_point，fix#7）——v7 现场证明
    // MoveIt FK 在此位姿相对物理有 ~17 mm z 偏差，追 FK 残差等于追幻影；
    // 实体服务缺失（真机）时才回退 FK。
    // 测量必须在 dwell 保持中进行且取时间窗中位数，不能单快照：cap-2 现场
    // 实测杆端在物理里以 mm 级晃动（静止态 x 方向 spread ~9 mm、arm0 关节
    // ~8 mrad 缓摆；dwell 主动保持中收窄但仍 ~mm 级），而收敛带只有 2.5 mm
    // ——单点测量把瞬时晃动当真偏（fix#8 run-4 首轮 R 2.6 mm 修正到位后次轮
    // 竟"变"5.4 mm，实为两次单快照各踩一次抖动）。窗内逐轴取中位数后，
    // 真偏被平均出来，晃动只留 ~mm/sqrt(N) 残差；窗内同时记录逐轴 spread，
    // 随残差一起打印，便于现场判断晃动是否收敛。
    // 2026-09-05 fix#11-v2 (v22 取证推翻 v21 沉降理论): 门限与修正对象回归
    // 纯物理真值切向残差，且测量前必须先停稳。v22 决定性证据（rest 构型 CSV
    // 对照 + 同窗 FK/物理采样）：
    //  1) FK-物理偏差在工作位姿不是 ≤0.7 mm 而是 ~15-18 mm（L (−4,+10,−14)
    //     R (−5,−3,−14) mm；与 fix#7 已录"work 位姿 FK ~17 mm z 幻影"同源），
    //     且随构型分布游走 —— fix#11 的 comp = (实测−FK) + (指令−轨点) 因此
    //     在门限语义上被 ~15 mm 幻影污染成垃圾（v22 round-1 comp 竟报 58-69
    //     mm，修正把右臂推飞 ~135 mm）。FK 只能用于"真机无实体服务"的兜底。
    //  2) 真实沉降（des−act）静止态仅 ±0.1-3 mrad/关节 ≈ 尖端 1-2 mm —— 不是
    //     v21 断言的 2-8 mrad→1-5 mm 主力项；v21/v22 首轮测量的更大误差来自
    //     dwell 两点轨迹的斜坡：FJT 可提前 SUCCESS，dwell 斜坡在验证通过后仍
    //     以 ~0.5 mrad/s 蠕动数十秒（motion 门只查短窗瞬时运动、8 mm 位置带内
    //     即放行），蠕动期间开窗把"仍在走向靶点"的瞬态当稳态，残差偏大且随
    //     测量时刻漂移。→ 每条测量前置停稳闸（真实关节 1.5 s 最大漂移 ≤0.6
    //     mrad 判稳；期限到仍不稳则带证测量，settled=false 打日志不静默）。
    // 修正在此语义下 = 把工作位姿靶点反平移 实测残差（切向、逐侧、≤20 mm 钳
    // 位、至多 2 轮、带内侧不动），把带内量留给钩爪闭合的接触自定位（8 mm
    // 门）吸收。fix#11-v2 曾假定"重到达钉同构型谷 → FK-物理偏差两次近似相
    // 同 → 修正 1:1"，v23a/v23b 证伪该前提：分支种子 IK 在近满伸竖直修正处
    // 跳谷（rad 级重构），重到达后的第二轮残差 = 两构型间幻影差 δφ 1-13 mm，
    // 与真实 aim 无关（带内 0 修正侧同样被无谓重到达推走 mm 级）。→ fix#12
    // 起修正以构型锁定关节微动执行（数值 Jacobian 伪逆 + 落点校订，见函数
    // 内 6760 行附近），构型不跳谷 → δφ→0，2.5 mm 收敛带可达。
    constexpr double kSelfCenterMeasureSeconds = 1.0;  // ~20 样本 @50 ms
    // 停稳闸（非冻结调参，v22 现场定）：1.5 s 观察窗内全部本侧臂关节最大漂移
    // ≤ 此量判稳；期限 60 s（覆盖 dwell 斜坡尾段）到仍不稳则带证照量。
    constexpr double kSelfCenterSettleDriftBand = 0.0006;  // rad / 1.5 s
    constexpr double kSelfCenterSettleSpanSeconds = 1.5;
    constexpr double kSelfCenterSettleDeadlineSeconds = 60.0;
    const auto axis_median = [](std::vector<double> & v) -> double {
      if (v.empty()) {
        return 0.0;
      }
      const auto mid = v.begin() + v.size() / 2;
      std::nth_element(v.begin(), mid, v.end());
      return *mid;
    };
    // 一次测量的返回值：residual = 切向物理残差（实测钩杆端 − 轨点；门限与
    // 修正的唯一对象）；spread = 测量窗逐轴最大极差；settled / settle_drift_
    // mrad = 停稳闸结论取证（残差可信度：窗内关节漂移越小越可信）。
    struct SideMeasure {
      Eigen::Vector3d residual;
      double spread;
      bool settled;
      double settle_drift_mrad;
    };
    const auto measure_side = [&](DrawerSide side) -> SideMeasure {
      const BimanualToolProfile & tool = drawer_tool_profile(side);
      const std::string label = drawer_side_name(side);
      MoveGroupInterface & group =
        side == DrawerSide::LEFT ? *left_group : *right_group;
      const std::string group_name = side == DrawerSide::LEFT ?
        left_group_name : right_group_name;
      const tf2::Vector3 rail = drawer_world_rail_point(
        control, drawer_side_point(control, side), rail_position);
      const tf2::Vector3 contact_local = tool.gripper_contact_point_local;
      // (a) 停稳闸：测量窗开窗前先等本侧整臂真正停稳。v22 取证：dwell 两点
      // 轨迹斜坡可在验证通过后仍以 ~0.5 mrad/s 蠕动数十秒，蠕动期间开窗把
      // "仍在走向靶点"的瞬态当稳态 → 残差偏大且随测量时刻漂移。判稳 = 真实
      // 关节 1.5 s 窗最大漂移 ≤0.6 mrad（des/act 跟随 ~2 mrad 内，des 冻结
      // ≈ act 冻结）；60 s 期限到仍未稳则带证照量（settled=false，不静默）。
      const moveit::core::JointModelGroup * settle_jmg =
        group.getRobotModel() == nullptr ? nullptr :
        group.getRobotModel()->getJointModelGroup(group_name);
      const std::vector<std::string> arm_joints =
        settle_jmg == nullptr ? std::vector<std::string>() :
        settle_jmg->getActiveJointModelNames();
      bool settled = false;
      double settle_drift_mrad = 0.0;
      if (!arm_joints.empty()) {
        std::vector<std::pair<double, std::vector<double>>> trace;
        const auto settle_deadline = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(kSelfCenterSettleDeadlineSeconds);
        for (;;) {
          check_cancel(goal_handle);
          const auto st = synchronized_current_robot_state(group);
          std::vector<double> pos;
          pos.reserve(arm_joints.size());
          for (const auto & name : arm_joints) {
            pos.push_back(st->getVariablePosition(name));
          }
          const double now_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
          trace.emplace_back(now_s, std::move(pos));
          while (!trace.empty() &&
                 now_s - trace.front().first > 2.0 * kSelfCenterSettleSpanSeconds)
          {
            trace.erase(trace.begin());
          }
          if (!trace.empty()) {
            const auto & base = trace.front().second;
            const auto & latest = trace.back().second;
            double max_drift = 0.0;
            for (std::size_t i = 0; i < latest.size(); ++i) {
              max_drift = std::max(
                max_drift, std::fabs(latest[i] - base[i]));
            }
            settle_drift_mrad = max_drift * 1000.0;
            if (now_s - trace.front().first >= kSelfCenterSettleSpanSeconds &&
                max_drift <= kSelfCenterSettleDriftBand)
            {
              settled = true;
              break;
            }
          }
          if (std::chrono::steady_clock::now() >= settle_deadline) {
            break;
          }
          interruptible_hold(goal_handle, 0.25);
        }
        RCLCPP_INFO(
          get_logger(),
          "Drawer '%s' %s self-center settle: %s (max %s-arm joint drift "
          "%.2f mrad over the last %.1f s).",
          control.id.c_str(), description.c_str(),
          settled ? "stable" : "still drifting at the deadline",
          label.c_str(), settle_drift_mrad, kSelfCenterSettleSpanSeconds);
      }
      // (b) 1 s 中位数测量窗：仅物理真值（fix#7 drawer_physics_link_point）；
      // FK 只作实体服务缺失（真机等）的兜底 —— 工作位姿 FK-物理偏差可达
      // ~15 mm（v22 取证），进门限即污染，兜底路径打日志照实报告来源。
      std::vector<double> xs, ys, zs;     // 实测杆端（物理优先，FK 兜底）
      bool used_fk_fallback = false;
      const auto measure_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(kSelfCenterMeasureSeconds);
      for (;;) {
        check_cancel(goal_handle);
        const auto physics_tip = drawer_physics_link_point(
          tool.gripper_contact_link, contact_local);
        if (physics_tip.has_value()) {
          xs.push_back(physics_tip->x()); ys.push_back(physics_tip->y());
          zs.push_back(physics_tip->z());
        } else {
          try {
            const auto state = synchronized_current_robot_state(group);
            const auto * link_model =
              state->getLinkModel(tool.gripper_contact_link);
            if (link_model != nullptr) {
              const Eigen::Vector3d fk_tip =
                state->getGlobalLinkTransform(link_model) *
                Eigen::Vector3d(
                  contact_local.x(), contact_local.y(), contact_local.z());
              xs.push_back(fk_tip.x()); ys.push_back(fk_tip.y());
              zs.push_back(fk_tip.z());
              used_fk_fallback = true;
            }
          } catch (const std::exception &) {
          }
        }
        if (std::chrono::steady_clock::now() >= measure_deadline) {
          break;
        }
        interruptible_hold(goal_handle, 0.05);
      }
      const auto de_axialize = [&](Eigen::Vector3d v) {
        return v - axis_world * (v.dot(axis_world) / axis_length2);
      };
      SideMeasure measure;
      measure.settled = settled;
      measure.settle_drift_mrad = settle_drift_mrad;
      // 窗口内一个样本都没有（理论仅当实体服务掉线且 FK 也拿不到）——兜底
      // 返回当前单次 FK 快照的切向残差，不让测量门静默跳过。
      if (xs.empty()) {
        const auto state = synchronized_current_robot_state(group);
        const auto * link_model =
          state->getLinkModel(tool.gripper_contact_link);
        if (link_model == nullptr) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Drawer '" + control.id +
                  "' self-centering cannot find the " + label +
                  " hook link '" + tool.gripper_contact_link + "'.");
        }
        const Eigen::Vector3d fallback =
          state->getGlobalLinkTransform(link_model) *
          Eigen::Vector3d(
            contact_local.x(), contact_local.y(), contact_local.z());
        measure.residual = de_axialize(Eigen::Vector3d(
          fallback.x() - rail.x(), fallback.y() - rail.y(),
          fallback.z() - rail.z()));
        measure.spread = 0.0;
        return measure;
      }
      if (used_fk_fallback) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Drawer '%s' %s self-center measured some samples from FK "
          "(physics entity service unavailable); FK-physics deviation at "
          "work poses can reach ~15 mm and skews the residual.",
          control.id.c_str(), description.c_str());
      }
      const Eigen::Vector3d tip(axis_median(xs), axis_median(ys),
        axis_median(zs));
      const double spread = std::max({
        *std::max_element(xs.begin(), xs.end()) -
          *std::min_element(xs.begin(), xs.end()),
        *std::max_element(ys.begin(), ys.end()) -
          *std::min_element(ys.begin(), ys.end()),
        *std::max_element(zs.begin(), zs.end()) -
          *std::min_element(zs.begin(), zs.end())});
      measure.residual = de_axialize(Eigen::Vector3d(
        tip.x() - rail.x(), tip.y() - rail.y(), tip.z() - rail.z()));
      measure.spread = spread;
      return measure;
    };
    for (int iteration = 0; iteration < kSelfCenterMaxIterations; ++iteration) {
      check_cancel(goal_handle);
      const auto left_measure = measure_side(DrawerSide::LEFT);
      const auto right_measure = measure_side(DrawerSide::RIGHT);
      const Eigen::Vector3d & left_residual = left_measure.residual;
      const Eigen::Vector3d & right_residual = right_measure.residual;
      const double left_norm = left_residual.norm();
      const double right_norm = right_residual.norm();
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' %s self-center round %d: hook residual (physics truth "
        "vs rail) left (%.1f, %.1f, %.1f) mm / %.1f mm, right (%.1f, %.1f, "
        "%.1f) mm / %.1f mm (band %.1f mm; settle L %s / R %s, max "
        "arm-joint drift L %.2f / R %.2f mrad; tip spread L %.1f / R "
        "%.1f mm).",
        control.id.c_str(), description.c_str(), iteration + 1,
        left_residual.x() * 1000.0, left_residual.y() * 1000.0,
        left_residual.z() * 1000.0, left_norm * 1000.0,
        right_residual.x() * 1000.0, right_residual.y() * 1000.0,
        right_residual.z() * 1000.0, right_norm * 1000.0,
        kSelfCenterBand * 1000.0,
        left_measure.settled ? "stable" : "UNSETTLED",
        right_measure.settled ? "stable" : "UNSETTLED",
        left_measure.settle_drift_mrad, right_measure.settle_drift_mrad,
        left_measure.spread * 1000.0, right_measure.spread * 1000.0);
      if (left_norm <= kSelfCenterBand && right_norm <= kSelfCenterBand) {
        if (iteration == 0) {
          RCLCPP_INFO(
            get_logger(),
            "Drawer '%s' hook aim already within the self-center band "
            "(physics truth).",
            control.id.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Drawer '%s' work pose aim self-centered after %d correction "
            "round(s) (physics truth).",
            control.id.c_str(), iteration);
        }
        return;
      }
      if (iteration + 1 == kSelfCenterMaxIterations) {
        // 最后测量仍超带：不发偏压贴，照实失败（收尾退让收起）。
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id + "' hook tips remain off the handle "
                "centers after " +
                std::to_string(kSelfCenterMaxIterations) +
                " self-center rounds (physics-truth lateral left " +
                std::to_string(left_norm) + " m, right " +
                std::to_string(right_norm) + " m, band " +
                std::to_string(kSelfCenterBand) +
                " m; settle L " +
                std::string(left_measure.settled ? "stable" : "unsettled") +
                ", R " +
                std::string(right_measure.settled ? "stable" : "unsettled") +
                "); the aim is too far off to converge. Recalibrate "
                "drawer_pose_aim_offset for this drawer.");
      }
      const auto clamp_step = [](const Eigen::Vector3d & v) {
        return v.norm() > kSelfCenterStepLimit ?
          v * (kSelfCenterStepLimit / v.norm()) : v;
      };
      // 只对超带侧施加修正，带内侧保持原目标不动：把好的一侧也推走只会让
      // 它承受一次新的到达误差（run2 round-1 右臂 1.5 mm 带内却被无谓的
      // 1.6 mm 修正推到 8.9 mm，直接耗尽 2 轮预算）。
      const Eigen::Vector3d left_shift =
        left_norm > kSelfCenterBand ? clamp_step(-left_residual) :
        Eigen::Vector3d::Zero();
      const Eigen::Vector3d right_shift =
        right_norm > kSelfCenterBand ? clamp_step(-right_residual) :
        Eigen::Vector3d::Zero();
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' self-centering the work pose: shifting the left target "
        "by (%.1f, %.1f, %.1f) mm and the right target by (%.1f, %.1f, "
        "%.1f) mm (world; 0 = side already within the band).",
        control.id.c_str(),
        left_shift.x() * 1000.0, left_shift.y() * 1000.0,
        left_shift.z() * 1000.0, right_shift.x() * 1000.0,
        right_shift.y() * 1000.0, right_shift.z() * 1000.0);
      geometry_msgs::msg::Pose & left_pose = poses.left_work_pose;
      geometry_msgs::msg::Pose & right_pose = poses.right_work_pose;
      left_pose.position.x += left_shift.x();
      left_pose.position.y += left_shift.y();
      left_pose.position.z += left_shift.z();
      right_pose.position.x += right_shift.x();
      right_pose.position.y += right_shift.y();
      right_pose.position.z += right_shift.z();
      // 2026-09-05 fix#12 (v23a/v23b 取证): 旧"整臂再到达"式修正到不了位。
      // 分支种子 IK 在近满伸的竖直修正处会跳谷 —— v23b round-2 右臂实测
      // r_arm2 −3.10→+0.48 / r_arm6 −1.06→+2.07（rad 级重构，钩端却只移
      // ~9 mm）；而 FK-物理偏差随构型游走（v22 取证 rest +35 / work −17 mm），
      // 跳谷后第二轮量到的 1-13 mm 残差几乎全是两个构型间的幻影差 δφ，与
      // 真实 aim 无关（v23a 左侧 0 修正也被无谓重到达推走 3.1 mm；带内
      // 2.1 mm 第二轮竟"变"4.6 mm 同源）。→ 修正不经过 IK/OMPL：以实测关节
      // 构型为基准，对接触点（gripper_contact_link + 局部偏移，与 measure
      // 同点同源）数值 Jacobian 求阻尼伪逆 + 牛顿残差校订，把 ≤20 mm 的世界
      // 系修正转成关节增量（每关节 ≤~0.1 rad，构型不跳谷 → δφ→~0，落点误差
      // 只剩控制器收敛残差与线性化二阶项 ~0.3-1 mm ≪ 2.5 mm 收敛带），沿
      // FJT 双点轨迹直发控制器执行。带内侧不发轨迹、构型保持；双侧都重挂
      // 到达驻留（12 s / 0.002 rad，同 plan_and_execute_bimanual_poses 语义），
      // 下一轮 measure 仍在 active 驻留保持下开窗。
      // 2026-09-05 fix#13 (v28 取证): 修正轨迹以“控制器当前指令参考 X_arr”
      // （controller_state.reference，订阅缓存）为锚，不以实测构型 start 为
      // 锚。v28 实锤：负载下物理 = 指令 + 稳态偏置 O = des − act（可达 ±7
      // mrad/关节），控制器换轨 t=0 阶跃 X_arr→act 时物理在 ~0.8 s 内再沉降
      // 一个 O（trace: L arm4 指令 194.6→191.0，物理 191.1→187.4）；终态锚
      // act + dj 时净物理位移 = dj + O ≠ dj（修正自毁：L 侧 2.5 → 5.3 mm，
      // 物理位移 ≈ FK(dj) + FK(O) 定量吻合）。t=0 锚 X_arr（无阶跃 → 无再沉
      // 降）、终态 = X_arr + dj 后，物理恰好落 act + dj = 修正目标。dj 仍以
      // 实测构型 start 为 FK 求解基准（物理系，不变）。参考缺失/过期时回退
      // start 锚（旧语义）并告警，下一轮测量照实兜底。
      struct SideCorrection {
        std::vector<std::string> joint_names;  // 空 = 带内侧，无需移动
        std::vector<double> start;             // 实测构型（FK 数值求解基准，物理系）
        std::vector<double> anchor;            // 轨迹 t=0 指令 = 控制器当前参考 X_arr（缺失回退 start）
        std::vector<double> terminal;          // 轨迹终态指令 = anchor + dj（净物理位移恰 = dj）
      };
      const auto build_side_correction = [&](
          const std::shared_ptr<MoveGroupInterface> & group,
          const std::string & group_name, DrawerSide side,
          const Eigen::Vector3d & shift) -> SideCorrection {
        SideCorrection correction;
        if (shift.norm() < 1e-6) {
          return correction;  // 带内侧：不动，不再承受一次重到达误差
        }
        const std::string label = drawer_side_name(side);
        const BimanualToolProfile & tool = drawer_tool_profile(side);
        const auto robot_model = group->getRobotModel();
        const auto * joint_model_group =
          robot_model == nullptr ? nullptr :
          robot_model->getJointModelGroup(group_name);
        const auto base_state = synchronized_current_robot_state(*group);
        const auto * link_model = base_state->getLinkModel(
          tool.gripper_contact_link);
        if (joint_model_group == nullptr || link_model == nullptr) {
          throw OperationError(
                  PressCabinetButton::Result::PLANNING_FAILED,
                  "Drawer '" + control.id + "' self-center micro-correction "
                  "cannot resolve the " + label + " arm model/link.");
        }
        correction.joint_names = joint_model_group->getActiveJointModelNames();
        const std::size_t joint_count = correction.joint_names.size();
        if (joint_count == 0) {
          throw OperationError(
                  PressCabinetButton::Result::PLANNING_FAILED,
                  "Drawer '" + control.id + "' self-center micro-correction "
                  "found no active joints on the " + label + " arm.");
        }
        correction.start.reserve(joint_count);
        for (const auto & name : correction.joint_names) {
          correction.start.push_back(base_state->getVariablePosition(name));
        }
        const Eigen::Vector3d local(
          tool.gripper_contact_point_local.x(),
          tool.gripper_contact_point_local.y(),
          tool.gripper_contact_point_local.z());
        // 注意：数值 lambda 一律带显式具体返回类型（Vector3d/MatrixXd/VectorXd），
        // 不能依赖 auto 推导 —— 推导会把惰性表达式树（Product/Solve 引用栈上
        // 临时对象）原样返回，调用方在 lambda 栈帧销毁后才求值 → 悬垂解引用
        // 崩溃（fix#12 v25/v26 gdb 取证：dls_solve 返回的悬垂 Solve 在
        // _solve_impl_transposed 里确定性段错误）。显式返回类型在 return 处求值。
        const auto point_of = [&](moveit::core::RobotState & st)
            -> Eigen::Vector3d {
          st.update();
          return st.getGlobalLinkTransform(link_model) * local;
        };
        const auto contact_point = [&](const Eigen::VectorXd & dj)
            -> Eigen::Vector3d {
          moveit::core::RobotState nudged(*base_state);
          for (std::size_t i = 0; i < joint_count; ++i) {
            nudged.setVariablePosition(
              correction.joint_names[i],
              correction.start[i] + dj(static_cast<Eigen::Index>(i)));
          }
          return point_of(nudged);
        };
        const auto jacobian_at = [&](const Eigen::VectorXd & dj)
            -> Eigen::MatrixXd {
          Eigen::MatrixXd jac(3, static_cast<Eigen::Index>(joint_count));
          const double h = 1e-6;  // rad 数值微分步长
          for (std::size_t i = 0; i < joint_count; ++i) {
            const double base_value =
              correction.start[i] + dj(static_cast<Eigen::Index>(i));
            moveit::core::RobotState fwd(*base_state);
            fwd.setVariablePosition(correction.joint_names[i], base_value + h);
            moveit::core::RobotState bwd(*base_state);
            bwd.setVariablePosition(correction.joint_names[i], base_value - h);
            jac.col(static_cast<Eigen::Index>(i)) =
              (point_of(fwd) - point_of(bwd)) / (2.0 * h);
          }
          return jac;
        };
        const auto dls_solve = [](const Eigen::MatrixXd & jac,
            const Eigen::Vector3d & d) -> Eigen::VectorXd {
          const Eigen::MatrixXd jt = jac.transpose();
          const Eigen::MatrixXd jjt = jac * jt;
          const double lam =
            1e-10 * std::max(1.0, jjt.diagonal().maxCoeff());
          return jt *
            (jjt + lam * Eigen::MatrixXd::Identity(3, 3)).ldlt().solve(d);
        };
        // 阻尼最小二乘 + 至多 3 轮牛顿残差校订（每轮重算 Jacobian），把预测
        // 落点误差压到亚毫米。
        const Eigen::VectorXd zero_dj =
          Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_count));
        const Eigen::Vector3d base_point = contact_point(zero_dj);
        Eigen::VectorXd dj = zero_dj;
        Eigen::Vector3d remaining = shift;
        for (int pass = 0; pass < 3 && remaining.norm() > 5e-4; ++pass) {
          dj += dls_solve(jacobian_at(dj), remaining);
          remaining = shift - (contact_point(dj) - base_point);
        }
        const double max_delta =
          dj.size() == 0 ? 0.0 : dj.cwiseAbs().maxCoeff();
        if (max_delta > 0.5) {
          throw OperationError(
                  PressCabinetButton::Result::PLANNING_FAILED,
                  "Drawer '" + control.id + "' self-center micro-correction "
                  "diverged on the " + label + " arm (max joint delta " +
                  std::to_string(max_delta) +
                  " rad); cannot converge the aim. Recalibrate "
                  "drawer_pose_aim_offset for this drawer.");
        }
        // 诚实收敛检查（fix#13）：3 轮牛顿校订后落点残差仍 > 1.5 mm = 求解
        // 停滞（近奇异/线性化失效），照实失败——放行只会把一次无效修正包装成
        // “已修正”，浪费下一轮预算（旧代码无此检查，v28 取证期间靠外部 FK
        // oracle 才看出 ~83-98% 落点缺口疑云）。
        if (remaining.norm() > 1.5e-3) {
          throw OperationError(
                  PressCabinetButton::Result::PLANNING_FAILED,
                  "Drawer '" + control.id + "' self-center micro-correction "
                  "failed to converge in 3 Newton passes on the " + label +
                  " arm (residual " + std::to_string(remaining.norm()) +
                  " m > 1.5e-3 m); cannot realize the " +
                  std::to_string(shift.norm()) +
                  " m shift. Recalibrate drawer_pose_aim_offset for this "
                  "drawer.");
        }
        // 终态相对控制器当前指令参考 X_arr 表达（见 SideCorrection 注释）。
        // anchor 缺失/过期 → 回退 start（fix#13 前语义，re-anchor 偏置残留）
        // 并告警——能锚则锚，不能锚也要让现场知道净位移将含 O。
        correction.anchor.reserve(joint_count);
        correction.terminal.reserve(joint_count);
        std::vector<double> reference;
        const bool anchored = arm_controller_reference(
          side == DrawerSide::LEFT, correction.joint_names, reference);
        if (!anchored) {
          RCLCPP_WARN(
            get_logger(),
            "Drawer '%s' %s %s-arm self-center correction has no fresh "
            "controller command reference; anchoring the trajectory at the "
            "measured state (re-anchor offset O = des-act remains, net "
            "physical move will include it).",
            control.id.c_str(), description.c_str(), label.c_str());
        }
        for (std::size_t i = 0; i < joint_count; ++i) {
          const double x_arr = anchored ?
            reference[i] : correction.start[i];
          correction.anchor.push_back(x_arr);
          correction.terminal.push_back(
            x_arr + dj(static_cast<Eigen::Index>(i)));
        }
        return correction;
      };
      const auto left_correction = build_side_correction(
        left_group, left_group_name, DrawerSide::LEFT, left_shift);
      const auto right_correction = build_side_correction(
        right_group, right_group_name, DrawerSide::RIGHT, right_shift);
      // 4 s 双点关节轨迹直发控制器；带内侧空轨迹 → 执行原语立即 SUCCESS
      // （不移动、构型保持）。
      constexpr double kMicroMoveSeconds = 4.0;
      moveit_msgs::msg::RobotTrajectory left_micro_trajectory;
      moveit_msgs::msg::RobotTrajectory right_micro_trajectory;
      const auto assemble_micro_trajectory = [&](
          const SideCorrection & correction) {
        moveit_msgs::msg::RobotTrajectory trajectory;
        if (correction.joint_names.empty()) {
          return trajectory;
        }
        auto & joint = trajectory.joint_trajectory;
        joint.joint_names = correction.joint_names;
        joint.points.resize(2);
        for (std::size_t point_index = 0; point_index < 2; ++point_index) {
          auto & point = joint.points[point_index];
          // t=0 必须 = 控制器当前指令参考 X_arr（anchor），不能 = 实测构型
          // start：从 X_arr 阶跃到 act 会触发一次负载再沉降 O（fix#13，
          // 见 SideCorrection 注释）。终态 = X_arr + dj → 物理恰好落 act + dj。
          point.positions = point_index == 0 ?
            correction.anchor : correction.terminal;
          point.velocities.assign(correction.joint_names.size(), 0.0);
          point.accelerations.assign(correction.joint_names.size(), 0.0);
          point.time_from_start.sec = static_cast<int32_t>(
            point_index == 0 ? 0 :
            static_cast<int32_t>(kMicroMoveSeconds));
          point.time_from_start.nanosec = 0;
        }
        return trajectory;
      };
      left_micro_trajectory = assemble_micro_trajectory(left_correction);
      right_micro_trajectory = assemble_micro_trajectory(right_correction);
      const auto micro_execution = execute_bimanual_trajectories_bounded(
        left_group, left_micro_trajectory, right_group,
        right_micro_trajectory,
        [this, &goal_handle]() { return goal_should_stop(goal_handle); },
        std::chrono::seconds(120),
        description + " self-center micro correction");
      if (micro_execution.left_code !=
          moveit::core::MoveItErrorCode::SUCCESS ||
        micro_execution.right_code != moveit::core::MoveItErrorCode::SUCCESS)
      {
        throw OperationError(
                PressCabinetButton::Result::EXECUTION_FAILED,
                "MoveIt failed to execute the '" + description +
                "' self-center micro correction (left=" +
                std::to_string(micro_execution.left_code.val) + " right=" +
                std::to_string(micro_execution.right_code.val) + ").");
      }
      if (operation_executed) {
        *operation_executed = true;
      }
      // 到达驻留重挂（t=0 阶跃钉精确终态 + 12 s 保持 + 0.002 rad/关节容差，
      // 与 plan_and_execute_bimanual_poses 的 arrival dwell 同语义）：修正臂钉
      // 终态；带内侧（joint_names 空）以当前实测构型钉住。发送失败不判失败
      // —— 下一轮 measure 的静止门禁照实兜底。
      constexpr double kCorrectionDwellHoldSeconds = 12.0;
      constexpr double kCorrectionSettleSeconds = 3.0;
      const auto send_hold_dwell = [this, &goal_handle, &control, &description](
          const std::shared_ptr<MoveGroupInterface> & group,
          const std::string & group_name,
          const std::vector<std::string> & preferred_names,
          const std::vector<double> & preferred_terminal,
          const std::string & side) -> bool {
        std::vector<std::string> joint_names = preferred_names;
        std::vector<double> terminal = preferred_terminal;
        if (joint_names.empty()) {
          const auto robot_model = group->getRobotModel();
          const auto * joint_model_group =
            robot_model == nullptr ? nullptr :
            robot_model->getJointModelGroup(group_name);
          if (joint_model_group == nullptr) {
            return false;
          }
          const auto st = synchronized_current_robot_state(*group);
          joint_names = joint_model_group->getActiveJointModelNames();
          terminal.reserve(joint_names.size());
          // 带内侧驻留同样必须锚控制器当前指令参考 X_arr，不能锚实测构型：
          // 锚 act 会让 t=0 从 X_arr 阶跃到 act，物理再沉降一个 O —— “带内
          // 侧第二/三轮莫名漂移 mm 级”的幽灵同源于此（fix#13，v22 起多轮取
          // 证）。无新鲜参考时回退 act 并告警（下一轮测量兜底）。
          std::vector<double> reference;
          const bool anchored = arm_controller_reference(
            side == "left", joint_names, reference);
          if (!anchored) {
            RCLCPP_WARN(
              get_logger(),
              "Drawer '%s' %s %s-arm in-band dwell has no fresh controller "
              "command reference; holding the measured state (re-anchor "
              "offset O remains).",
              control.id.c_str(), description.c_str(), side.c_str());
          }
          for (std::size_t i = 0; i < joint_names.size(); ++i) {
            terminal.push_back(anchored ?
              reference[i] : st->getVariablePosition(joint_names[i]));
          }
        }
        if (joint_names.empty()) {
          return false;
        }
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory> *
          client = side == "left" ? left_fjt_client_.get() :
          right_fjt_client_.get();
        auto & slot = side == "left" ?
          bimanual_controller_handles_->left :
          bimanual_controller_handles_->right;
        if (!client->wait_for_action_server(std::chrono::seconds(10))) {
          RCLCPP_ERROR(
            get_logger(),
            "Bimanual %s-arm correction dwell controller unavailable.",
            side.c_str());
          return false;
        }
        auto goal = control_msgs::action::FollowJointTrajectory::Goal();
        auto & trajectory = goal.trajectory;
        trajectory.joint_names = joint_names;
        const std::size_t joint_count = joint_names.size();
        trajectory.points.resize(2);
        for (std::size_t point_index = 0; point_index < 2; ++point_index) {
          auto & point = trajectory.points[point_index];
          point.positions = terminal;
          point.velocities.assign(joint_count, 0.0);
          point.accelerations.assign(joint_count, 0.0);
          point.time_from_start.sec = static_cast<int32_t>(
            point_index == 0 ? 0 :
            static_cast<int32_t>(kCorrectionDwellHoldSeconds));
          point.time_from_start.nanosec = 0;
        }
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);
        goal.goal_tolerance.resize(joint_count);
        for (std::size_t index = 0U; index < joint_count; ++index) {
          auto & tolerance = goal.goal_tolerance[index];
          tolerance.name = joint_names[index];
          tolerance.position = 0.002;
          tolerance.velocity = 0.0;
          tolerance.acceleration = 0.0;
        }
        try {
          check_cancel(goal_handle);
          const auto send_future = client->async_send_goal(goal);
          const auto send_deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(5);
          while (send_future.wait_for(50ms) != std::future_status::ready) {
            check_cancel(goal_handle);
            if (std::chrono::steady_clock::now() >= send_deadline) {
              RCLCPP_ERROR(
                get_logger(),
                "Bimanual %s-arm correction dwell goal was not accepted in "
                "time.",
                side.c_str());
              return false;
            }
          }
          const auto accepted = send_future.get();
          if (!accepted) {
            RCLCPP_ERROR(
              get_logger(),
              "Bimanual %s-arm controller rejected the correction dwell "
              "goal.",
              side.c_str());
            return false;
          }
          {
            std::lock_guard<std::mutex> lock(
              bimanual_controller_handles_->mutex);
            slot = accepted;
          }
          return true;
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            get_logger(),
            "Bimanual %s-arm correction dwell was not armed: %s",
            side.c_str(), error.what());
          return false;
        }
      };
      const bool left_dwell = send_hold_dwell(
        left_group, left_group_name, left_correction.joint_names,
        left_correction.terminal, "left");
      const bool right_dwell = send_hold_dwell(
        right_group, right_group_name, right_correction.joint_names,
        right_correction.terminal, "right");
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' %s self-center correction applied (joint move L %d / "
        "R %d; dwell L %d / R %d); re-measuring under active hold.",
        control.id.c_str(), description.c_str(),
        static_cast<int>(left_correction.joint_names.empty() ? 0 : 1),
        static_cast<int>(right_correction.joint_names.empty() ? 0 : 1),
        left_dwell ? 1 : 0, right_dwell ? 1 : 0);
      // fix#13 取证日志：逐关节打印指令锚点 X_arr / FK 基准 act / 终态，单位
      // mrad。anchor−start = 负载偏置 O（re-anchor 自毁的本体）：锚点正确时
      // 净物理位移 = FK(dj)（= terminal−anchor 引起的），对不上即可当场定位。
      const auto joint_row_mrad = [](const std::vector<double> & v) {
        std::ostringstream oss;
        for (std::size_t i = 0; i < v.size(); ++i) {
          if (i > 0) {
            oss << ' ';
          }
          oss << std::fixed << std::setprecision(2) << v[i] * 1000.0;
        }
        return oss.str();
      };
      const std::string left_anchor = left_correction.anchor.empty() ?
        "-" : joint_row_mrad(left_correction.anchor);
      const std::string right_anchor = right_correction.anchor.empty() ?
        "-" : joint_row_mrad(right_correction.anchor);
      const std::string left_terminal = left_correction.terminal.empty() ?
        "-" : joint_row_mrad(left_correction.terminal);
      const std::string right_terminal = right_correction.terminal.empty() ?
        "-" : joint_row_mrad(right_correction.terminal);
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' %s self-center anchor trace (mrad, arm0..arm6): L "
        "anchor(X_arr) %s | terminal %s | R anchor(X_arr) %s | terminal %s.",
        control.id.c_str(), description.c_str(),
        left_anchor.c_str(), left_terminal.c_str(),
        right_anchor.c_str(), right_terminal.c_str());
      // 沉降等待：驻留 t=0 的阶跃修正先衰减完（可取消），下一轮 measure 的
      // 停稳闸随后在 active 驻留下开窗。
      interruptible_hold(goal_handle, kCorrectionSettleSeconds);
    }
  }

  // Matched waypoints for both arms: the i-th entry of each vector commands
  // the SAME drawer rail position, so the two arms traverse the rail in lock
  // step and the drawer never sees one-sided travel.
  void calculate_drawer_bimanual_waypoints(
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    std::vector<geometry_msgs::msg::Pose> & left_waypoints,
    std::vector<geometry_msgs::msg::Pose> & right_waypoints)
  {
    const double travel = target_position - initial_position;
    const std::size_t count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(std::abs(travel) / drawer_waypoint_step_)));
    left_waypoints.clear();
    right_waypoints.clear();
    left_waypoints.reserve(count);
    right_waypoints.reserve(count);
    // The pull waypoints hold the tips pressed INTO the handle plates (same
    // contact offset as the grasp pose), matching what the base-translation
    // drive actually holds — the coupling keepalive requires this contact.
    const double contact_offset = drawer_grasp_contact_offset(
      control, initial_position);
    for (std::size_t index = 1U; index <= count; ++index) {
      const double ratio = static_cast<double>(index) /
        static_cast<double>(count);
      const double position = initial_position + travel * ratio;
      left_waypoints.push_back(calculate_drawer_side_tool_pose(
        control, DrawerSide::LEFT, position, contact_offset));
      right_waypoints.push_back(calculate_drawer_side_tool_pose(
        control, DrawerSide::RIGHT, position, contact_offset));
    }
  }

  std::vector<geometry_msgs::msg::Pose> calculate_rotation_waypoints(
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    double tool_roll_offset = 0.0)
  {
    const double travel = target_position - initial_position;
    const std::size_t count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(std::abs(travel) / rotation_waypoint_step_)));
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(count);
    for (std::size_t index = 1U; index <= count; ++index) {
      const double ratio = static_cast<double>(index) /
        static_cast<double>(count);
      waypoints.push_back(
        calculate_rotary_tool_pose(
          control, initial_position + travel * ratio,
          control.grasp_outward_offset, true, tool_roll_offset));
    }
    return waypoints;
  }

  double rotary_manipulation_position(
    const ButtonSpec & control,
    double initial_position,
    double target_position) const
  {
    double release_fraction = 1.0;
    if (control.control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR)
    {
      release_fraction = door_release_fraction_;
    } else if (control.control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
    {
      release_fraction = control.detent_release_fraction;
    }
    return rotary_release_position(
      initial_position, target_position, release_fraction,
      target_tolerance_);
  }

  template<typename GoalHandleT>
  void set_control_grasp(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::string & control_id,
    bool attach)
  {
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (!grasp_client_->wait_for_service(50ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Cabinet grasp service is unavailable.");
      }
    }
    auto request = std::make_shared<
      xczs_inspection_robot_interfaces::srv::SetCabinetGrasp::Request>();
    request->control_id = control_id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (attach &&
        (!operation_lease_held_.load() || operation_lease_lost_.load() ||
        operation_lease_id_.empty()))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::GRASP_FAILED,
                "Cabinet grasp attach requires an active global operation "
                "lease.");
      }
      request->operation_lease_id = operation_lease_id_;
    }
    request->robot_model = robot_model_name_;
    request->robot_link = grasp_link_;
    request->robot_grasp_point.x = grasp_point_position_.x();
    request->robot_grasp_point.y = grasp_point_position_.y();
    request->robot_grasp_point.z = grasp_point_position_.z();
    request->robot_base_link = grasp_brake_link_;
    request->attach = attach;
    auto future = grasp_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Cabinet grasp request timed out.");
      }
    }
    const auto response = future.get();
    if (!response->success) {
      throw GenericOperationError(
              attach ? OperateCabinetControl::Result::GRASP_FAILED :
              OperateCabinetControl::Result::RELEASE_FAILED,
              response->message + " (distance " +
              std::to_string(response->distance) + " m)");
    }
  }

  void release_control_grasp_noexcept(const std::string & control_id) noexcept
  {
    try {
      if (!grasp_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(
          get_logger(),
          "Emergency grasp release service is unavailable for '%s'.",
          control_id.c_str());
        return;
      }
      constexpr int kReleaseAttempts = 2;
      for (int attempt = 1; attempt <= kReleaseAttempts; ++attempt) {
        auto request = std::make_shared<
          xczs_inspection_robot_interfaces::srv::SetCabinetGrasp::Request>();
        request->control_id = control_id;
        {
          std::lock_guard<std::mutex> lock(operation_lease_mutex_);
          request->operation_lease_id = operation_lease_id_;
        }
        request->robot_model = robot_model_name_;
        request->robot_link = grasp_link_;
        request->robot_grasp_point.x = grasp_point_position_.x();
        request->robot_grasp_point.y = grasp_point_position_.y();
        request->robot_grasp_point.z = grasp_point_position_.z();
        request->robot_base_link = grasp_brake_link_;
        request->attach = false;
        auto future = grasp_client_->async_send_request(request);
        if (future.wait_for(2s) != std::future_status::ready) {
          RCLCPP_ERROR(
            get_logger(),
            "Emergency grasp release timed out for '%s' (attempt %d/%d).",
            control_id.c_str(), attempt, kReleaseAttempts);
          continue;
        }
        const auto response = future.get();
        if (response->success) {
          return;
        }
        RCLCPP_ERROR(
          get_logger(),
          "Emergency grasp release failed for '%s' (attempt %d/%d): %s",
          control_id.c_str(), attempt, kReleaseAttempts,
          response->message.c_str());
      }
      RCLCPP_ERROR(
        get_logger(),
        "Emergency grasp release exhausted all attempts for '%s'.",
        control_id.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Emergency grasp release failed: %s", error.what());
    }
  }

  void verify_bimanual_tool_calibration_state(
    const moveit::core::RobotState & left_state,
    const moveit::core::RobotState & right_state) const
  {
    const auto verify_one = [this](const BimanualToolProfile & profile,
        const moveit::core::RobotState & robot_state,
        const std::string & side) {
        for (std::size_t index = 0U;
          index < profile.calibration_joint_names.size(); ++index)
        {
          double measured;
          try {
            measured = robot_state.getVariablePosition(
              profile.calibration_joint_names[index]);
          } catch (const std::exception & error) {
            throw OperationError(
                    PressCabinetButton::Result::NOT_READY,
                    "Drawer " + side +
                    " tool business-point calibration references unknown "
                    "joint '" + profile.calibration_joint_names[index] +
                    "': " + error.what());
          }
          if (!std::isfinite(measured) ||
            std::abs(measured - profile.calibration_joint_positions[index]) >
            tool_tip_calibration_joint_tolerance_)
          {
            throw OperationError(
                    PressCabinetButton::Result::NOT_READY,
                    "Drawer " + side + " tool joint '" +
                    profile.calibration_joint_names[index] + "' is at " +
                    std::to_string(measured) +
                    " rad/m, outside its calibrated business-point position " +
                    std::to_string(profile.calibration_joint_positions[index]) +
                    " +/- " +
                    std::to_string(tool_tip_calibration_joint_tolerance_) +
                    "; bimanual motion was blocked. Reset the tool first.");
          }
        }
      };
    verify_one(drawer_left_tool_, left_state, "left");
    verify_one(drawer_right_tool_, right_state, "right");
  }

  // ---- 2026-09-03 AGENT §3: unlock-motor full-joint drive ----

  // The unlock motor (unlock_motor_joint, r_three_cyl_finger3_joint) is a joint
  // of the three_cylinder tool controller.  Sending a goal that omits the other
  // controller joints is silently DROPPED (allow_partial_joints_goal=false, P1
  // verified), so every drive sends a FULL-joint trajectory that holds the two
  // drawer-carrying fingers at their current (calibrated ~0) state while moving
  // the motor.  The drive also bypasses MoveIt group planning on purpose: group
  // planning would collision-check away the very contact the extension makes.
  //
  // ``send_unlock_motor_goal`` is the raw send+wait.  The action result is NOT
  // the arbiter of success — a weak-gain finger motor can finish its ramp short
  // of the commanded target (P1: ~0.5 mm shortfall on a 2 mm target) or abort
  // with GOAL_TOLERANCE_VIOLATED while still physically extended past the
  // unlock floor, so drive_unlock_motor judges by the REAL measured joint.
  template<typename GoalHandleT>
  void send_unlock_motor_goal(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & start_positions,
    double motor_target,
    std::size_t motor_index)
  {
    if (!unlock_motor_fjt_client_->wait_for_action_server(
        std::chrono::seconds(10))) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Three-cylinder controller (unlock motor) action server is "
              "unavailable.");
    }
    auto goal = control_msgs::action::FollowJointTrajectory::Goal();
    auto & trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;
    const std::size_t joint_count = joint_names.size();
    trajectory.points.resize(2);
    auto & point_0 = trajectory.points[0];
    point_0.positions = start_positions;
    point_0.velocities.assign(joint_count, 0.0);
    point_0.accelerations.assign(joint_count, 0.0);
    point_0.time_from_start.sec = 0;
    point_0.time_from_start.nanosec = 0;
    auto & point_1 = trajectory.points[1];
    point_1.positions = start_positions;
    point_1.positions[motor_index] = motor_target;
    point_1.velocities.assign(joint_count, 0.0);
    point_1.accelerations.assign(joint_count, 0.0);
    constexpr double kUnlockMotorRampSeconds = 2.0;
    point_1.time_from_start.sec =
      static_cast<int32_t>(kUnlockMotorRampSeconds);
    point_1.time_from_start.nanosec = 0;
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(5.0);
    goal.goal_tolerance.resize(joint_count);
    for (std::size_t index = 0U; index < joint_count; ++index) {
      auto & tolerance = goal.goal_tolerance[index];
      tolerance.name = joint_names[index];
      // Moving-joint tolerance is generous on purpose: a weak-gain finger motor
      // may rest a few mm short of target (drive_unlock_motor judges by the
      // measured joint, not by this tolerance).
      tolerance.position = (index == motor_index) ? 0.005 : 0.002;
      tolerance.velocity = 0.0;
      tolerance.acceleration = 0.0;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    auto send_future = unlock_motor_fjt_client_->async_send_goal(goal);
    while (send_future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Unlock motor goal could not be accepted by the three-cylinder "
                "controller within the deadline.");
      }
    }
    const auto accepted = send_future.get();
    if (!accepted) {
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "The three-cylinder controller rejected the unlock motor goal.");
    }
    auto result_future = unlock_motor_fjt_client_->async_get_result(accepted);
    try {
      while (result_future.wait_for(50ms) != std::future_status::ready) {
        check_cancel(goal_handle);
        if (std::chrono::steady_clock::now() >= deadline) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Unlock motor goal result did not arrive within the "
                  "deadline.");
        }
      }
    } catch (...) {
      // Cancel/abort: best-effort stop the motor goal so the controller does
      // not keep creeping toward target during the outer teardown, then rethrow.
      unlock_motor_fjt_client_->async_cancel_goal(accepted);
      throw;
    }
    result_future.get();  // outcome judged by the caller from the real joint
  }

  // Read the unlock motor joint's REAL position back from the robot state.
  double read_unlock_motor_position(
    MoveGroupInterface & move_group, const std::string & motor_joint)
  {
    const auto state = synchronized_current_robot_state(move_group);
    const double measured = state->getVariablePosition(motor_joint);
    if (!std::isfinite(measured)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Unlock motor joint '" + motor_joint +
              "' has no finite position in the robot state.");
    }
    return measured;
  }

  // Bounded drive of the unlock motor to ``target_position`` with real joint
  // readback.  ``phase`` is "extend" (toward unlock_pressed_position) or
  // "retract" (back to unlock_retracted_position).
  //
  // Extend success = the measured joint landed in [retracted+floor, ceiling):
  // below the floor the rod never engaged the zone (fail loud — the unlock
  // service is NOT called); at/above the ceiling it is bottoming out (fail
  // loud).  Retract success = the measured joint is back within 2 mm of
  // retracted so no probe is left sticking out for the following arm motions.
  // A weak-gain finger motor may still be creeping after its ramp, so the
  // measured joint is polled for a short window before judging.  An extend that
  // never reached the floor (or overshot the ceiling) retracts best-effort
  // before throwing so no extended probe is left behind.
  template<typename GoalHandleT>
  double drive_unlock_motor(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    MoveGroupInterface & move_group,
    double target_position,
    const std::string & phase,
    bool * operation_executed)
  {
    const bool extending = phase == "extend";
    const std::vector<std::string> & joint_names =
      drawer_right_tool_.calibration_joint_names;
    const auto motor_it = std::find(
      joint_names.begin(), joint_names.end(), control.unlock_motor_joint);
    if (motor_it == joint_names.end()) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' unlock motor joint '" +
              control.unlock_motor_joint +
              "' is not among the right drawer tool's calibrated joints; the "
              "unlock motor cannot be driven.");
    }
    const std::size_t motor_index = static_cast<std::size_t>(
      std::distance(joint_names.begin(), motor_it));
    if (target_position < control.unlock_retracted_position) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' unlock motor target " +
              std::to_string(target_position) + " m is below retracted " +
              std::to_string(control.unlock_retracted_position) + " m.");
    }
    if (extending && target_position >= control.unlock_extension_ceiling) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' unlock motor target " +
              std::to_string(target_position) + " m is at/above the extension "
              "ceiling " + std::to_string(control.unlock_extension_ceiling) +
              " m; the motor must not bottom out.");
    }

    // Hold every other finger at its current (calibrated ~0) position so the
    // goal covers the controller's full joint set (allow_partial_joints_goal).
    const auto state = synchronized_current_robot_state(move_group);
    std::vector<double> start_positions;
    start_positions.reserve(joint_names.size());
    for (const auto & name : joint_names) {
      const double current = state->getVariablePosition(name);
      if (!std::isfinite(current)) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Finger joint '" + name +
                "' has no finite position in the robot state; cannot hold it "
                "during the unlock motor drive.");
      }
      start_positions.push_back(current);
    }
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' unlock motor %s: commanding %s to %.4f m (other fingers "
      "held at their current state).",
      control.id.c_str(), phase.c_str(), control.unlock_motor_joint.c_str(),
      target_position);
    send_unlock_motor_goal(
      goal_handle, joint_names, start_positions, target_position, motor_index);
    *operation_executed = true;

    // Short settle, then poll the REAL motor joint for a short window — a
    // weak-gain finger motor keeps creeping after its ramp ends (P1).
    const double floor_position = control.unlock_retracted_position +
      control.unlock_extension_floor;
    const double retract_tolerance = 0.002;
    const auto poll_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(6.0);
    double measured = std::numeric_limits<double>::quiet_NaN();
    for (;;) {
      check_cancel(goal_handle);
      interruptible_hold(goal_handle, 0.05);
      measured = read_unlock_motor_position(
        move_group, control.unlock_motor_joint);
      const bool reached = extending ?
        measured >= floor_position : measured <=
        control.unlock_retracted_position + retract_tolerance;
      if (reached) {
        break;
      }
      if (std::chrono::steady_clock::now() >= poll_deadline) {
        break;
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' unlock motor %s: measured %s at %.4f m.",
      control.id.c_str(), phase.c_str(), control.unlock_motor_joint.c_str(),
      measured);

    // Best-effort pull back toward retracted before any loud failure, so no
    // probe is left sticking out for the outer retreat/cleanup arm motions.
    auto best_effort_retract = [&]() {
      try {
        drive_unlock_motor(
          goal_handle, control, move_group,
          control.unlock_retracted_position, "retract", operation_executed);
      } catch (const std::exception & retract_error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' unlock motor best-effort retract after failure also "
          "failed: %s",
          control.id.c_str(), retract_error.what());
      }
    };

    if (extending) {
      if (measured >= control.unlock_extension_ceiling) {
        best_effort_retract();
        throw OperationError(
                PressCabinetButton::Result::EXECUTION_FAILED,
                "Drawer '" + control.id + "' unlock motor bottomed out: "
                "measured " + std::to_string(measured) + " m at/above ceiling " +
                std::to_string(control.unlock_extension_ceiling) + " m.");
      }
      if (measured < floor_position) {
        best_effort_retract();
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id + "' unlock motor never extended into "
                "[retracted+floor, ceiling): measured " +
                std::to_string(measured) + " m < " +
                std::to_string(floor_position) +
                " m. The unlock service was NOT called.");
      }
    } else {
      if (measured > control.unlock_retracted_position + retract_tolerance) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id + "' unlock motor did not retract: "
                "measured " + std::to_string(measured) + " m, expected within " +
                std::to_string(retract_tolerance) +
                " m of retracted " +
                std::to_string(control.unlock_retracted_position) + " m.");
      }
    }
    return measured;
  }

  // ---------------------------------------------------------------------------
  // 2026-09-03 AGENT §4/§5: real support & gripper ROD stages.
  //
  // A drawer support/grasp is not just an arm pose: the side's support /
  // gripper cylinder must genuinely EXTEND (moving rod joint) so its rod end
  // reaches the drawer.  Each stage issues a bounded full-joint FJT on the
  // side's own cylinder controller (left two_cylinder, right three_cylinder —
  // both require full-joint goals, allow_partial_joints_goal=false), then
  // verifies the moving rod's REAL measured joint reached its work position
  // and stayed stable.  The rod-end contact distance (moving rod link + its
  // measured rod-end local offset vs the drawer support/handle point) and the
  // free-drawer immobility are measured and LOGGED here as the §7.2 stage 4/5
  // sealing metrics; their hard thresholds are committed to config once the
  // metric semantics are confirmed live.  The stage is gated by the §4.2 role
  // config: while support_contact_position / gripper_grasp_position are still
  // 0 (unsealed) the flow keeps the pre-§4 pose-only behaviour.

  // Read any moving rod joint's REAL position back from the robot state.
  double read_real_joint_position(
    MoveGroupInterface & move_group, const std::string & joint,
    const std::string & label)
  {
    // 2026-09-04 AGENT §8.4 fix#3 root-cause (v11): measured 必须来自真实 JS
    // 缓存——move_group current state 的杆位置卡在启动初始位形（从未收到
    // 真实 joint_states），deep-press evidence 读它必败。缓存缺样本/过期
    // （栈外断流）才退回 move_group scene 同步并告警。
    double measured = std::numeric_limits<double>::quiet_NaN();
    if (cached_real_joint_position(joint, measured)) {
      return measured;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Real joint-state cache has no fresh sample for '%s'; falling back to "
      "the move_group current state (stale risk for the '%s' verdict).",
      joint.c_str(), label.c_str());
    const auto state = synchronized_current_robot_state(move_group);
    measured = state->getVariablePosition(joint);
    if (!std::isfinite(measured)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              label + " joint '" + joint +
              "' has no finite position in the robot state.");
    }
    return measured;
  }

  // Same doctrine for joint effort (fix#3 v2 hook-seal press signal): the real
  // joint-state cache is the source; move_group current state is only a
  // stale-risk fallback.  Effort is used as a *sign* test (>= a positive floor),
  // never negated, so a fallback value must not accidentally read as pressed:
  // return 0.0 (no press) when the value is not finite/meaningful rather than
  // throwing — the seal then correctly fails on the force evidence.
  double read_real_joint_effort(
    MoveGroupInterface & move_group, const std::string & joint,
    const std::string & label)
  {
    double measured = std::numeric_limits<double>::quiet_NaN();
    if (cached_real_joint_effort(joint, measured)) {
      return std::isfinite(measured) ? measured : 0.0;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Real joint-state cache has no fresh effort sample for '%s'; falling "
      "back to the move_group current state (stale risk for the '%s' press "
      "verdict).",
      joint.c_str(), label.c_str());
    const auto state = synchronized_current_robot_state(move_group);
    measured = state->getVariableEffort(joint);
    return std::isfinite(measured) ? measured : 0.0;
  }

  // Full-joint two-point FJT acceptance handle (2026-09-04 AGENT §8.4 fix#3
  // probe path): the goal is sent and ACCEPTED without blocking on its result,
  // so BOTH sides' rods can be commanded toward the same phase together (the
  // seat retract must reach both controllers at once — see
  // seal_drawer_hooks_by_probe).  The caller polls the REAL measured joints —
  // measured stays the arbiter (execute_drawer_rod_stage_side doctrine).
  // `deadline` reproduces the blocking send's sim-time budget so the caller
  // can wait on (or abandon) the result with the same discipline.
  struct PendingRodGoal
  {
    using Client = rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>;
    using GoalHandle = rclcpp_action::ClientGoalHandle<
      control_msgs::action::FollowJointTrajectory>;
    GoalHandle::SharedPtr accepted;
    std::shared_future<Client::WrappedResult> result;
    std::chrono::steady_clock::time_point deadline;
  };

  // Accept half of send_cylinder_full_joint_goal (same server wait, goal build
  // and cancel/deadline discipline as the blocking send).
  template<typename GoalHandleT>
  PendingRodGoal accept_drawer_rod_goal(
    rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>::SharedPtr client,
    const std::string & controller_label,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    MoveGroupInterface & move_group,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & desired_positions,
    double goal_tolerance_seconds = 5.0,
    double ramp_seconds = 2.0,
    // fix#15 companion (AGENT §4.4): a downstream full-joint goal that carries
    // a hook PRESS-hold ref (gripper_override = press_ref, physically blocked
    // by the flush plate ~0.015 m short of 0.030) must not stall-abort at the
    // grace end and stop pushing the hook.  Give the HOLD joint a wide goal
    // tolerance so the JTC reports SUCCESS as soon as the role rod arrives;
    // the succeeded goal then holds its final ref (hook kept positively
    // pressed).  The caller judges arrival by the real measured ROLE joint, so
    // the wide hold tolerance never masks a failed role rod.  Empty name =
    // legacy per-joint auto tolerance (moving 0.005 / static 0.002).
    const std::string & wide_tolerance_joint = std::string(),
    double wide_position_tolerance = 0.0)
  {
    if (!client->wait_for_action_server(std::chrono::seconds(10))) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              controller_label +
              " controller (drawer rods) action server is unavailable.");
    }
    const std::size_t joint_count = joint_names.size();
    if (desired_positions.size() != joint_count) {
      throw std::invalid_argument(
              "accept_drawer_rod_goal requires desired_positions to "
              "match the controller joint set size.");
    }
    // 2026-09-04 AGENT §8.4 fix#3 root-cause (v11): 轨迹起点必须真实——用
    // move_group scene 的卡启动位形做首点，controller 会从物理真实位置向
    // 错误首点做一次瞬态猛冲。逐关节优先真实 JS 缓存，scene 仅兜底。
    moveit::core::RobotStatePtr scene_state;
    std::vector<double> start_positions;
    start_positions.reserve(joint_count);
    for (const auto & name : joint_names) {
      double current = std::numeric_limits<double>::quiet_NaN();
      if (!cached_real_joint_position(name, current)) {
        if (!scene_state) {
          scene_state = synchronized_current_robot_state(move_group);
        }
        current = scene_state->getVariablePosition(name);
      }
      if (!std::isfinite(current)) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Rod joint '" + name +
                "' has no finite position in the robot state; cannot hold it "
                "during the " + controller_label + " drive.");
      }
      start_positions.push_back(current);
    }
    auto goal = control_msgs::action::FollowJointTrajectory::Goal();
    auto & trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;
    trajectory.points.resize(2);
    auto & point_0 = trajectory.points[0];
    point_0.positions = start_positions;
    point_0.velocities.assign(joint_count, 0.0);
    point_0.accelerations.assign(joint_count, 0.0);
    point_0.time_from_start.sec = 0;
    point_0.time_from_start.nanosec = 0;
    auto & point_1 = trajectory.points[1];
    point_1.positions = desired_positions;
    point_1.velocities.assign(joint_count, 0.0);
    point_1.accelerations.assign(joint_count, 0.0);
    // ramp_seconds 默认 2.0 s (~11 mm/s 指令速率); seal 座封重驱传慢 ramp
    // (0.6 mm/s 级) —— 快 ramp 远超杆 ~1.4 mm/s 的自由空气外伸能力 → e 全程
    // 巨大 → 无积分上限的 effort-mode PID 大充能, 到接触后继续深磨 (v29c 磨进
    // 引擎, v31 re-drive 复现)。慢 ramp 让 e 保持小, 充能降至 ~0.1 N/s 级。
    point_1.time_from_start.sec = static_cast<int32_t>(ramp_seconds);
    point_1.time_from_start.nanosec = 0;
    goal.goal_time_tolerance =
      rclcpp::Duration::from_seconds(goal_tolerance_seconds);
    goal.goal_tolerance.resize(joint_count);
    for (std::size_t index = 0U; index < joint_count; ++index) {
      auto & tolerance = goal.goal_tolerance[index];
      tolerance.name = joint_names[index];
      // Moving-joint tolerance is generous on purpose: a weak-gain finger motor
      // may rest a few mm short of target; the caller judges by the measured
      // joint, not by this tolerance.
      const bool is_hold_joint = !wide_tolerance_joint.empty() &&
        joint_names[index] == wide_tolerance_joint;
      tolerance.position = is_hold_joint ?
        wide_position_tolerance :
        ((std::abs(desired_positions[index] - start_positions[index]) > 1e-4) ?
         0.005 : 0.002);
      tolerance.velocity = 0.0;
      tolerance.acceleration = 0.0;
    }
    // The JTC reports its result (success, or GOAL_TOLERANCE_VIOLATED at
    // ramp_end + goal_time_tolerance) in SIM time.  The acceptance world runs
    // at RTF ~0.47-0.55 (2026-09-03 live), so the 7 s sim worst case takes
    // ~13-15 s wall — the old flat 15 s deadline raced it and cancelled healthy
    // drives (cap2 run A/B evidence: "rod goal result did not arrive within
    // the deadline" while the drive was still legitimately in flight).  Budget
    // the goal's sim-time span at an RTF floor of 1/3 plus slack, never below
    // the base wait timeout.
    constexpr double kRodSimRtfFloor = 0.33;
    constexpr double kRodGoalWallSlack = 5.0;
    const double deadline_seconds = std::max(
      system_wait_timeout_,
      (ramp_seconds + goal.goal_time_tolerance.sec +
       goal.goal_time_tolerance.nanosec * 1e-9) / kRodSimRtfFloor +
      kRodGoalWallSlack);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(deadline_seconds);
    auto send_future = client->async_send_goal(goal);
    while (send_future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                controller_label +
                " rod goal could not be accepted within the deadline.");
      }
    }
    const auto accepted = send_future.get();
    if (!accepted) {
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "The " + controller_label + " controller rejected the rod goal.");
    }
    PendingRodGoal pending;
    pending.accepted = accepted;
    pending.result = client->async_get_result(accepted);
    // deadline carries a double-rep duration from the RTF-floor budget above;
    // store it at the clock's native resolution for comparison in the callers.
    pending.deadline =
      std::chrono::time_point_cast<std::chrono::steady_clock::duration>(
      deadline);
    return pending;
  }

  // Full-joint two-point FJT on `client` (the side's cylinder controller):
  // ramp every named joint from its current measured position to
  // `desired_positions`.  Judges nothing — the caller verifies the REAL
  // measured joints.  Mirrors send_unlock_motor_goal but takes the controller
  // client explicitly so the LEFT rods can use the two_cylinder controller.
  template<typename GoalHandleT>
  void send_cylinder_full_joint_goal(
    rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>::SharedPtr client,
    const std::string & controller_label,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    MoveGroupInterface & move_group,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & desired_positions,
    double goal_tolerance_seconds = 5.0,
    // fix#15 companion: pass-through to accept_drawer_rod_goal — widen the
    // HOLD joint's goal tolerance on full-joint goals carrying a press hold.
    const std::string & wide_tolerance_joint = std::string(),
    double wide_position_tolerance = 0.0)
  {
    const PendingRodGoal pending = accept_drawer_rod_goal(
      client, controller_label, goal_handle, move_group, joint_names,
      desired_positions, goal_tolerance_seconds, 2.0, wide_tolerance_joint,
      wide_position_tolerance);
    try {
      while (pending.result.wait_for(50ms) != std::future_status::ready) {
        check_cancel(goal_handle);
        if (std::chrono::steady_clock::now() >= pending.deadline) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  controller_label +
                  " rod goal result did not arrive within the deadline.");
        }
      }
    } catch (...) {
      // Cancel/abort: best-effort stop the rod goal so the controller does not
      // keep creeping toward target during the outer teardown, then rethrow.
      client->async_cancel_goal(pending.accepted);
      throw;
    }
    pending.result.get();  // outcome judged by the caller from the real joint
  }

  // Build the FULL desired position vector for one side's cylinder controller
  // at the given rod stage.  Unlisted joints (none today) keep their current
  // measured position; role joints are overridden per stage.  2026-09-03
  // AGENT §6.3 matrix (P4): the drawer runs hook → support only, and the hooks
  // STAY closed through the support stage (the old matrix opened the gripper
  // during "support", violating the §6.3 hold contract):
  //   "hook"     support->retracted,       gripper->grasp,  (right) unlock->retracted
  //   "support"  support->contact,         gripper->grasp(held §6.3), unlock->retracted
  //   "home"     support->retracted,       gripper->open,   (right) unlock->retracted
  // gripper_position_override (2026-09-04 fix#3) replaces the gripper value
  // with a probe ref (deep-press 0.015 / seat 0.006) while the "hook" matrix
  // keeps every other role at its default — used by
  // seal_drawer_hooks_by_probe.
  std::vector<double> drawer_rod_stage_desired(
    const BimanualToolProfile & tool, const ButtonSpec & control,
    MoveGroupInterface & move_group, const std::string & stage,
    double gripper_position_override =
      std::numeric_limits<double>::quiet_NaN(),
    // 2026-09-05 cap5 fix#16: 非 NaN 时覆盖本侧 support 关节值（活体 FK
    // 自标定的 q），与 gripper_position_override 同范式。仅 drive_drawer_
    // support_stage 传；其余阶段调用不传（默认静态 support_contact_position）。
    double support_position_override =
      std::numeric_limits<double>::quiet_NaN())
  {
    const auto state = synchronized_current_robot_state(move_group);
    std::vector<double> desired;
    desired.reserve(tool.calibration_joint_names.size());
    for (const auto & name : tool.calibration_joint_names) {
      double value = state->getVariablePosition(name);
      if (!std::isfinite(value)) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Rod joint '" + name +
                "' has no finite position in the robot state; cannot plan the "
                "rod stage '" + stage + "'.");
      }
      if (tool.has_support_role && name == tool.support_joint) {
        // 支撑杆: hook/home 阶段收拢, support 阶段伸到接触位。fix#16:
        // support 阶段若带活体 support_position_override（自标定 q），以它为
        // 目标而非静态 support_contact_position（x0 漂移时固定 q 会深插过缝口）。
        value = (stage == "support") ?
          (std::isfinite(support_position_override) ?
            support_position_override : tool.support_contact_position) :
          tool.support_retracted_position;
      } else if (tool.has_gripper_role && name == tool.gripper_joint) {
        // 钩爪杆: home 张开; hook/support 阶段都保持 grasp（§6.3 合同:
        // support 阶段钩爪不得松 —— 旧矩阵在此置 open, 违约时序已废止）。
        // fix#3 探针: gripper_position_override(深压 ref / 座封 ref)覆盖默认
        // 的 grasp, "hook" 矩阵其余角色保持不变。
        double gripper_value = (stage == "home") ?
          tool.gripper_open_position : tool.gripper_grasp_position;
        if (std::isfinite(gripper_position_override)) {
          gripper_value = gripper_position_override;
        }
        value = gripper_value;
      } else if (name == control.unlock_motor_joint) {
        value = control.unlock_retracted_position;  // right three-cylinder
      }
      desired.push_back(value);
    }
    return desired;
  }

  // fix#7 (2026-09-05 AGENT §7.2 cap-2 v7 取证): 抽屉接触几何改以 gazebo 物理
  // 真值为准。v7 现场闭环: gazebo 模型运动学与 MoveIt FK 存在位姿相关 z 偏差
  // （肩 +7 mm → 工具座 +36 mm @rest、−17 mm @work，按 boot/IK 分支游走）。
  // 逐关节对比已排除帧混合: 硬件态 == 广播流 == RSP 源、odom→body == entity
  // body (0.5486 精确一致)、两路 URDF 同文 —— 偏差在物理侧链本身。因此凡
  // “杆/钩端 vs latch 合同轨点” 的世界真值比较, 一律取 /get_entity_state 的
  // 连杆位姿 + 同一 local 接触偏移（odom==world，与轨点同帧直接可比）;
  // MoveIt FK 只保留作执行跟踪与无 gazebo 实体服务（真机）时的回退。
  std::optional<Eigen::Vector3d> drawer_physics_link_point(
    const std::string & rod_link, const tf2::Vector3 & rod_end_local)
  {
    if (drawer_entity_client_ == nullptr) {
      return std::nullopt;
    }
    if (!drawer_entity_measure_available_ &&
      !drawer_entity_client_->service_is_ready() &&
      !drawer_entity_client_->wait_for_service(100ms))
    {
      if (!drawer_entity_measure_warned_) {
        drawer_entity_measure_warned_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Drawer physics-truth service '%s' is unavailable; falling back to "
          "MoveIt FK geometry (real-hardware mode).",
          drawer_entity_client_->get_service_name());
      }
      return std::nullopt;
    }
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = rod_link;
    auto future = drawer_entity_client_->async_send_request(request);
    if (future.wait_for(2s) != std::future_status::ready) {
      drawer_entity_measure_available_ = false;  // 后续调用快速回退，不再各等 2 s
      if (!drawer_entity_measure_warned_) {
        drawer_entity_measure_warned_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Drawer physics-truth query for '%s' timed out; falling back to "
          "MoveIt FK geometry (real-hardware mode).",
          rod_link.c_str());
      }
      return std::nullopt;
    }
    const auto response = future.get();
    if (response == nullptr || !response->success) {
      if (!drawer_entity_measure_warned_) {
        drawer_entity_measure_warned_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Drawer physics-truth query for '%s' failed; falling back to MoveIt "
          "FK geometry (real-hardware mode).",
          rod_link.c_str());
      }
      return std::nullopt;
    }
    const auto & pose = response->state.pose;
    const auto & p = pose.position;
    const auto & o = pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.z) ||
      !std::isfinite(o.w))
    {
      return std::nullopt;
    }
    drawer_entity_measure_available_ = true;
    const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    const Eigen::Vector3d end = Eigen::Vector3d(p.x, p.y, p.z) +
      q * Eigen::Vector3d(
        rod_end_local.x(), rod_end_local.y(), rod_end_local.z());
    return end;
  }

  // Rod end (moving-rod link pose + its measured rod-end local offset) to a
  // world drawer target point.  This is the §4.3/§5.3 contact metric; its hard
  // threshold is sealed live at §7.2 stages 4/5.
  double rod_end_distance(
    MoveGroupInterface & move_group, const std::string & rod_link,
    const tf2::Vector3 & rod_end_local, const tf2::Vector3 & world_target)
  {
    Eigen::Vector3d end;
    const auto physics_end = drawer_physics_link_point(rod_link, rod_end_local);
    if (physics_end.has_value()) {
      end = *physics_end;
    } else {
      const auto state = synchronized_current_robot_state(move_group);
      const Eigen::Isometry3d & pose = state->getGlobalLinkTransform(rod_link);
      end = pose * Eigen::Vector3d(
        rod_end_local.x(), rod_end_local.y(), rod_end_local.z());
    }
    return (end - Eigen::Vector3d(
      world_target.x(), world_target.y(), world_target.z())).norm();
  }

  // Execute ONE side's rod stage: send the controller's full-joint goal built
  // for `stage`, then poll the REAL moving rod (`role_joint`) as evidence.
  //  - retracting: evidence must come back within retract_tolerance of 0
  //    (support/gripper home);
  //  - extending: evidence must reach at least target - reach_tolerance and
  //    stay strictly below the travel ceiling (bottoming = loud failure), then
  //    hold a stable window (stable_state_duration_, stable band) so the rod
  //    genuinely "arrived and settled" instead of bouncing.
  // Returns the final measured rod position.
  template<typename GoalHandleT>
  double execute_drawer_rod_stage_side(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    MoveGroupInterface & move_group,
    const BimanualToolProfile & tool,
    rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>::SharedPtr client,
    const std::string & controller_label,
    const std::string & stage,
    const std::string & role_joint,
    double evidence_target,
    bool retracting,
    bool * operation_executed,
    // 2026-09-05 AGENT doc §4.3/§4.4 (P1) + fix#15: 非 NaN 时作为本侧 gripper
    // 位置覆盖传给 drawer_rod_stage_desired —— support 阶段携带 seal 输出的深压
    // 保持参考 press_ref，防止静态 gripper_grasp_position 或座封 joint_ref 把钩爪
    // 命令降回失力位（ref 0.030→0.016 的积分退火正是 cap2 右杆回弹根因）。
    // 默认 NaN = legacy 语义（静态 grasp）；retract/其它阶段调用不传。
    double gripper_override = std::numeric_limits<double>::quiet_NaN(),
    // 2026-09-05 cap5 fix#16: 非 NaN 时作为本侧 support 位置覆盖传给
    // drawer_rod_stage_desired —— support 阶段携带活体 FK 自标定的 q，命令
    // 杆端停在缝口内侧（而非固定 support_contact_position 深插过缝口）。
    // 只 drive_drawer_support_stage 传；其余阶段调用不传（默认 NaN）。
    double support_position_override =
      std::numeric_limits<double>::quiet_NaN())
  {
    const auto & joint_names = tool.calibration_joint_names;
    if (std::find(joint_names.begin(), joint_names.end(), role_joint) ==
      joint_names.end())
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' rod joint '" + role_joint +
              "' is not among the drawer tool's calibrated joints.");
    }
    constexpr double kRodTravelCeiling = 0.120;   // URDF finger-rod upper limit
    if (!retracting && evidence_target >= kRodTravelCeiling) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' rod stage '" + stage +
              "' target " + std::to_string(evidence_target) +
              " m is at/above the rod travel ceiling " +
              std::to_string(kRodTravelCeiling) + " m; the rod must not bottom "
              "out.");
    }
    // Contact pushes (extend) grant a LONG JTC tolerance grace so the
    // controller keeps servo-pushing to the P+I ceiling (~18 N at the
    // i_clamp) until the rod breaks free or the grace expires — see the
    // mechanism comment below the first send.  Retract keeps the short grace.
    constexpr double kRodExtendGraceSeconds = 60.0;  // SIM seconds
    const double grace_seconds = retracting ? 5.0 : kRodExtendGraceSeconds;
    const std::vector<double> desired_positions = drawer_rod_stage_desired(
      tool, control, move_group, stage, gripper_override,
      support_position_override);
    // fix#15 companion (AGENT §4.4): 本阶段若携带有限 gripper_override（深压
    // 保持参考 press_ref），钩爪物理上被位面钉在实测贴位（~0.015 m，press_ref
    // 0.030 不可达）—— 给该 HOLD 关节宽目标容差，让 JTC 在 role 杆到位即报
    // SUCCESS（成功目标继续顶住 press_ref 保持正压），而不是等 60 s 宽限耗尽
    // 报 GOAL_TOLERANCE_VIOLATED 停推、钩爪复现 cap2 回弹。容差须覆盖
    // press_ref − 最小可能贴位；role 杆仍持自动容差 + 调用方实测判据，不受影响。
    constexpr double kRodGripperHoldTolerance = 0.030;  // m
    const bool holding_gripper_override = std::isfinite(gripper_override);
    const std::string hold_tolerance_joint = holding_gripper_override ?
      tool.gripper_joint : std::string();
    const double hold_tolerance = holding_gripper_override ?
      kRodGripperHoldTolerance : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' %s rod stage on the %s controller: commanding '%s' "
      "to %.4f m.",
      control.id.c_str(), stage.c_str(), controller_label.c_str(),
      role_joint.c_str(), evidence_target);
    send_cylinder_full_joint_goal(
      client, controller_label, goal_handle, move_group, joint_names,
      desired_positions, grace_seconds, hold_tolerance_joint,
      hold_tolerance);
    *operation_executed = true;

    // Poll the REAL rod joint: a weak-gain finger motor may keep creeping after
    // its ramp (P1) or abort short with GOAL_TOLERANCE_VIOLATED while already
    // physically there, so the measured joint is the arbiter.  A contact push
    // that stalls must ALSO be given the time to break free: the JTC aborts a
    // stalled goal at ramp_end + goal_time_tolerance (SIM time) and stops
    // servo-pushing — on a fresh acceptance world the reference then decays
    // back to the measured joint and the drive is over (fresh-stack cap2 rerun
    // 2026-09-03: abort ~10 s wall after the 0.0100 ramp ended, ref snap from
    // 0.0100 to ~-0.0003, effort collapse to the residual hold).  During the
    // short grace the P-term alone (800 x err <= 8 N for a 0.0100 target)
    // cannot break the claw/handle stiction, so the rod stays pinned ~0.0002
    // at the contact while the integral needs ~20+ s sim to climb toward its
    // clamp (~18 N ceiling at P + i_clamp).  Extend stages (hook/support
    // contact pushes) therefore grant a LONG tolerance grace so the controller
    // keeps pushing the reference at full error until the rod slips or the
    // grace expires; retract stages keep the short grace (they arrive into the
    // 0 stop quickly).  The measured joint stays the arbiter either way, and
    // "arrived" means SUSTAINED inside the band — a single slip blip (<0.3 s,
    // snap-back) must not fire arrival, hence the sustain window below.
    constexpr double kRodReachTolerance = 0.006;
    constexpr double kRodRetractTolerance = 0.006;
    constexpr double kRodStableBand = 0.003;
    constexpr double kRodArrivalSustainSeconds = 1.0;  // blip-proof arrival
    // The send below blocks until the goal result (SUCCESS early, or
    // GOAL_TOLERANCE_VIOLATED at grace end ~130-190 s wall for the long
    // grace); the rod may slip any time during that window and settles at the
    // contact, so the poll budget below only needs to OBSERVE the outcome and
    // hold the sustain + stability gates.  Retract polls cover the mirrored
    // pin when a contact-hold unwinds.
    constexpr double kRodExtendPollBudget = 60.0;
    constexpr double kRodRetractPollBudget = 30.0;
    constexpr int kRodRedriveAttempts = 1;  // one fresh ramp before giving up
    const auto in_band = [&](double value) {
      return retracting ?
        value <= kRodRetractTolerance :
        (value >= evidence_target - kRodReachTolerance &&
         value < kRodTravelCeiling);
    };
    double measured = std::numeric_limits<double>::quiet_NaN();
    bool converged = false;
    const int max_rounds = 1 + kRodRedriveAttempts;
    for (int round = 0; round < max_rounds && !converged; ++round) {
      if (round > 0) {
        // Re-drive from the measured pose: a fresh two-point ramp re-asserts
        // the reference and re-saturates the drive if the earlier goal was
        // cancelled or the rod stalled outside the band.
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' rod stage '%s': %s not converged after the first "
          "drive (measured %.4f m), re-driving once.",
          control.id.c_str(), stage.c_str(), role_joint.c_str(), measured);
        send_cylinder_full_joint_goal(
          client, controller_label, goal_handle, move_group, joint_names,
          desired_positions, grace_seconds, hold_tolerance_joint,
          hold_tolerance);
      }
      const double budget = retracting ? kRodRetractPollBudget :
        kRodExtendPollBudget;
      const auto poll_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(budget);
      double good_since = std::numeric_limits<double>::quiet_NaN();
      while (std::chrono::steady_clock::now() < poll_deadline) {
        check_cancel(goal_handle);
        interruptible_hold(goal_handle, 0.05);
        measured = read_real_joint_position(
          move_group, role_joint, controller_label);
        if (!retracting && measured >= kRodTravelCeiling) {
          throw OperationError(
                  PressCabinetButton::Result::EXECUTION_FAILED,
                  "Drawer '" + control.id + "' rod stage '" + stage +
                  "' bottomed out: measured " + std::to_string(measured) +
                  " m at/above the travel ceiling.");
        }
        const auto now = std::chrono::steady_clock::now();
        if (in_band(measured)) {
          if (std::isnan(good_since)) {
            good_since = std::chrono::duration<double>(
              now.time_since_epoch()).count();
          } else if (std::chrono::duration<double>(now.time_since_epoch())
                     .count() - good_since >= kRodArrivalSustainSeconds) {
            converged = true;
            break;
          }
        } else {
          good_since = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }
    if (!converged) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' rod stage '" + stage + "' never "
              "reached its target: measured " + std::to_string(measured) +
              " m (target " + std::to_string(evidence_target) +
              (retracting ? " m, retract)." : " m, extend)."));
    }
    // Stability window: the rod must hold a tight band over the project's
    // stable-state duration (no drift / bounce after arrival).
    const auto stable_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(stable_state_duration_);
    double band_min = measured;
    double band_max = measured;
    while (std::chrono::steady_clock::now() < stable_deadline) {
      check_cancel(goal_handle);
      interruptible_hold(goal_handle, 0.05);
      const double sample = read_real_joint_position(
        move_group, role_joint, controller_label);
      band_min = std::min(band_min, sample);
      band_max = std::max(band_max, sample);
      if (band_max - band_min > kRodStableBand) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id + "' rod stage '" + stage +
                "' did not stay stable after arrival: measured swung "
                "[" + std::to_string(band_min) + ", " +
                std::to_string(band_max) + "] m within " +
                std::to_string(stable_state_duration_) + " s.");
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' rod stage '%s': %s measured %.4f m and held stable "
      "over %.3f s.",
      control.id.c_str(), stage.c_str(), role_joint.c_str(), measured,
      stable_state_duration_);
    return measured;
  }

  // Best-effort return of every rod to its retracted/open home after a failed
  // rod stage or during teardown, so no extended rod is left for the following
  // arm motions.  Never throws — failures are logged and swallowed.
  template<typename GoalHandleT>
  void best_effort_drawer_rods_home(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    MoveGroupInterface & move_group,
    bool * operation_executed)
  {
    const auto rods_home_side = [&](const BimanualToolProfile & tool,
      rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::
        SharedPtr client, const std::string & controller_label) {
      if (!tool.has_support_role && !tool.has_gripper_role) {
        return;
      }
      try {
        const std::vector<double> home_positions = drawer_rod_stage_desired(
          tool, control, move_group, "home");
        send_cylinder_full_joint_goal(
          client, controller_label, goal_handle, move_group,
          tool.calibration_joint_names, home_positions);
        *operation_executed = true;
      } catch (const std::exception & rod_home_error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' best-effort rod home on the %s controller failed: %s",
          control.id.c_str(), controller_label.c_str(),
          rod_home_error.what());
      }
    };
    if (drawer_left_tool_.has_support_role || drawer_left_tool_.has_gripper_role) {
      rods_home_side(
        drawer_left_tool_, two_cylinder_fjt_client_, "two-cylinder (left)");
    }
    if (drawer_right_tool_.has_support_role || drawer_right_tool_.has_gripper_role) {
      rods_home_side(
        drawer_right_tool_, unlock_motor_fjt_client_,
        "three-cylinder (right)");
    }
  }

  tf2::Vector3 drawer_world_rail_point(
    const ButtonSpec & control, const tf2::Vector3 & local_point,
    double rail_position)
  {
    return resolve_cabinet_transform() *
      (local_point + control.drawer_axis * rail_position);
  }

  double measure_drawer_rod_contact(
    MoveGroupInterface & move_group, const ButtonSpec & control,
    const std::string & role_label, const std::string & rod_link,
    const tf2::Vector3 & rod_end_local, const tf2::Vector3 & world_target)
  {
    const double distance = rod_end_distance(
      move_group, rod_link, rod_end_local, world_target);
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' %s rod end (%s) is %.4f m from the drawer %s target.",
      control.id.c_str(), role_label.c_str(), rod_link.c_str(), distance,
      role_label.c_str());
    return distance;
  }

  // 2026-09-03 AGENT §4.3/§6.1 (P4): support stage.  Arms sit at the single
  // work pose with the hook fingers ALREADY closed onto the handles (hook
  // stage ran first and §6.3 keeps them closed through this stage); this
  // drives both SUPPORT rods out to their §4.2 contact positions so each tool
  // gains real contact reaction for the pull, then verifies rod arrival +
  // stability + hook-retain (hooks still on the plates) + drawer immobility.
  template<typename GoalHandleT>
  void drive_drawer_support_stage(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    MoveGroupInterface & move_group,
    const DrawerHookHoldState & hook_hold,
    bool * operation_executed)
  {
    const bool left_enabled = drawer_left_tool_.has_support_role &&
      drawer_left_tool_.support_contact_position > 0.0;
    const bool right_enabled = drawer_right_tool_.has_support_role &&
      drawer_right_tool_.support_contact_position > 0.0;
    if (!left_enabled && !right_enabled) {
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' support rod stage is not enabled (support_contact_"
        "position unsealed) — keeping pose-only support.",
        control.id.c_str());
      return;
    }
    if (left_enabled != right_enabled) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id +
              "' support rod stage must be symmetric: both sides need a sealed "
              "support_contact_position.");
    }
    const double slide_before = button_snapshot(control).position;
    // 2026-09-05 cap5 fix#16: 活体 FK 自标定支撑目标 q。support_contact_position
    // 是「固定 q」，按某次 cap-1 实体探针的 q0 悬停 x0 反推（q = x0 − 0.097，
    // 左 0.1376/右 0.1391 封定）；但 x0 随停靠/工作位姿 run 间漂移（实测 cap5
    // run 左 x0≈0.1295，~9mm 更近），固定 q 在无碰撞的缝内空腔里深插过缝口
    // ~10mm → support 距离门必挂。修复 = 支撑伸出前用 FK 测当前 q0 悬停杆端
    // x（杆未伸出、自由空域，FK 即真值且无求解器噪声），命令 q = x0.x − 缝口x
    // + inset —— x0=0.1376 时正好复现 fix#9 的 0.041，x0 漂移时杆端始终停在
    // 缝口内侧 inset 处，门余量由几何合同恢复而非放宽容差。inset <= 0 或 FK
    // 失败 → 退回固定 q（默认 inset 0 = 其余抽屉/场景零回归）。
    const double kSupportLiveTargetCeiling = 0.050;  // 远低于 0.12 杆行程顶
    const auto support_target_x = [&](const BimanualToolProfile & tool,
      const tf2::Vector3 & support_point, double sealed_q) -> double {
      if (tool.support_contact_inset <= 0.0) {
        return sealed_q;
      }
      try {
        // 当前 (q≈0) 悬停杆端接触点的世界 x。优先 gazebo 物理真值 —— 与下方
        // support 距离门同源同帧（fix#7 原则），消除 fix#16 年代 MoveIt FK 与
        // 物理侧 x 偏差（cap4 run 两侧杆端因此短 ~7mm、卡 8mm 门）。实体服务
        // 不可用（真机）回退 MoveIt FK（自由空域即真值）。fix#17。
        double hover_x = std::numeric_limits<double>::quiet_NaN();
        const auto physics_end = drawer_physics_link_point(
          tool.support_contact_link, tool.support_contact_point_local);
        if (physics_end.has_value()) {
          hover_x = physics_end->x();
        } else {
          const auto state = synchronized_current_robot_state(move_group);
          const Eigen::Isometry3d & pose =
            state->getGlobalLinkTransform(tool.support_contact_link);
          const Eigen::Vector3d local(
            tool.support_contact_point_local.x(),
            tool.support_contact_point_local.y(),
            tool.support_contact_point_local.z());
          hover_x = (pose * local).x();
        }
        // 缝口平面随抽屉滑动的世界 x（与本函数下方距离门的参考同源）。
        const double mouth_x =
          drawer_world_rail_point(control, support_point, slide_before).x();
        double q = hover_x - mouth_x + tool.support_contact_inset;
        q = std::min(std::max(q, 0.0), kSupportLiveTargetCeiling);
        RCLCPP_INFO(
          get_logger(),
          "Drawer '%s' %s support live target: q0-hover x0 %.4f m, seam-mouth "
          "x %.4f m, inset %.4f m → command q %.4f m (sealed %.4f m).",
          control.id.c_str(), tool.contact_tool_link.c_str(), hover_x,
          mouth_x, tool.support_contact_inset, q, sealed_q);
        return q;
      } catch (const std::exception & fk_error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' %s support live FK failed (%s) — falling back to the "
          "sealed support_contact_position %.4f m.",
          control.id.c_str(), tool.contact_tool_link.c_str(),
          fk_error.what(), sealed_q);
        return sealed_q;
      }
    };
    const double support_target_left = control.has_left_support_point ?
      support_target_x(
        drawer_left_tool_, control.left_support_point,
        drawer_left_tool_.support_contact_position) :
      drawer_left_tool_.support_contact_position;
    const double support_target_right = control.has_right_support_point ?
      support_target_x(
        drawer_right_tool_, control.right_support_point,
        drawer_right_tool_.support_contact_position) :
      drawer_right_tool_.support_contact_position;
    double measured_left = 0.0;
    double measured_right = 0.0;
    try {
      measured_left = execute_drawer_rod_stage_side(
        goal_handle, control, move_group, drawer_left_tool_,
        two_cylinder_fjt_client_, "two-cylinder (left)", "support",
        drawer_left_tool_.support_joint,
        support_target_left, false, operation_executed,
        hook_hold.valid ? hook_hold.left_joint_ref :
          std::numeric_limits<double>::quiet_NaN(),
        support_target_left);
      measured_right = execute_drawer_rod_stage_side(
        goal_handle, control, move_group, drawer_right_tool_,
        unlock_motor_fjt_client_, "three-cylinder (right)", "support",
        drawer_right_tool_.support_joint,
        support_target_right, false, operation_executed,
        hook_hold.valid ? hook_hold.right_joint_ref :
          std::numeric_limits<double>::quiet_NaN(),
        support_target_right);
    } catch (...) {
      best_effort_drawer_rods_home(goal_handle, control, move_group,
        operation_executed);
      throw;
    }
    // 2026-09-05 cap4 fix#17: 支撑落位闭环补驱。首轮驱杆后物理杆端可能因欠冲/
    // 振铃停在缝口外数毫米（cap4 实测左 8.1mm/右 9.4mm > 8mm 门临界）。用与距离
    // 门同源的 /get_entity_state 物理真值判有符号短差，只对「仍偏外」侧小额补伸
    // （≤4mm/次、至多 3 次），使杆端进入落位带后再过 8mm 硬门；已过口(偏内)不回收
    // —— 缝内空腔深插安全、杆回缩有 stiction 拖不动。补驱复用 execute_…
    // _rod_stage_side（各自到达+稳定门），钩爪保持 ref 随重发（§6.3 不得松）。
    constexpr double kSupportLandingRecheck = 0.006;
    constexpr double kSupportTopUpMaxStep = 0.004;
    constexpr int kSupportTopUpAttempts = 3;
    const auto signed_support_shortfall = [&](const BimanualToolProfile & tool,
      const tf2::Vector3 & support_point, double slide) -> double {
      const auto tip = drawer_physics_link_point(
        tool.support_contact_link, tool.support_contact_point_local);
      if (!tip.has_value()) {
        return std::numeric_limits<double>::quiet_NaN();  // 真机: 物理不可用不补驱
      }
      const double mouth_x = drawer_world_rail_point(
        control, support_point, slide).x();
      // 伸向 −x: 目标杆端 x = 缝口 x − inset。>0 偏外(短) → 需 +q 补伸。
      return tip->x() - (mouth_x - tool.support_contact_inset);
    };
    double left_target = support_target_left;
    double right_target = support_target_right;
    for (int pass = 0; pass < kSupportTopUpAttempts; ++pass) {
      const double slide_ref = button_snapshot(control).position;
      bool left_short = false;
      bool right_short = false;
      if (control.has_left_support_point) {
        const double d = measure_drawer_rod_contact(
          move_group, control, "support",
          drawer_left_tool_.support_contact_link,
          drawer_left_tool_.support_contact_point_local,
          drawer_world_rail_point(
            control, control.left_support_point, slide_ref));
        const double s = signed_support_shortfall(
          drawer_left_tool_, control.left_support_point, slide_ref);
        if (std::isfinite(s) && d > kSupportLandingRecheck && s >= 0.001) {
          left_short = true;
          left_target = std::min(
            left_target + std::min(s, kSupportTopUpMaxStep),
            drawer_left_tool_.support_contact_position + 0.012);
        }
      }
      if (control.has_right_support_point) {
        const double d = measure_drawer_rod_contact(
          move_group, control, "support",
          drawer_right_tool_.support_contact_link,
          drawer_right_tool_.support_contact_point_local,
          drawer_world_rail_point(
            control, control.right_support_point, slide_ref));
        const double s = signed_support_shortfall(
          drawer_right_tool_, control.right_support_point, slide_ref);
        if (std::isfinite(s) && d > kSupportLandingRecheck && s >= 0.001) {
          right_short = true;
          right_target = std::min(
            right_target + std::min(s, kSupportTopUpMaxStep),
            drawer_right_tool_.support_contact_position + 0.012);
        }
      }
      if (!left_short && !right_short) {
        break;  // 无偏外侧或物理不可用 → 交下方硬门
      }
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' support landing pass %d: shortfall-driven top-up re-drive "
        "left to q %.4f, right to q %.4f.",
        control.id.c_str(), pass + 1, left_target, right_target);
      try {
        if (left_short) {
          measured_left = execute_drawer_rod_stage_side(
            goal_handle, control, move_group, drawer_left_tool_,
            two_cylinder_fjt_client_, "two-cylinder (left)", "support",
            drawer_left_tool_.support_joint, left_target, false,
            operation_executed,
            hook_hold.valid ? hook_hold.left_joint_ref :
              std::numeric_limits<double>::quiet_NaN(), left_target);
        }
        if (right_short) {
          measured_right = execute_drawer_rod_stage_side(
            goal_handle, control, move_group, drawer_right_tool_,
            unlock_motor_fjt_client_, "three-cylinder (right)", "support",
            drawer_right_tool_.support_joint, right_target, false,
            operation_executed,
            hook_hold.valid ? hook_hold.right_joint_ref :
              std::numeric_limits<double>::quiet_NaN(), right_target);
        }
      } catch (...) {
        best_effort_drawer_rods_home(
          goal_handle, control, move_group, operation_executed);
        throw;
      }
    }
    // Final 8 mm geometry gate (unchanged; drawer_support_contact_tolerance_).
    const double slide_after = button_snapshot(control).position;
    double left_contact_distance = std::numeric_limits<double>::infinity();
    double right_contact_distance = std::numeric_limits<double>::infinity();
    if (control.has_left_support_point) {
      left_contact_distance = measure_drawer_rod_contact(
        move_group, control, "support",
        drawer_left_tool_.support_contact_link,
        drawer_left_tool_.support_contact_point_local,
        drawer_world_rail_point(
          control, control.left_support_point, slide_after));
    }
    if (control.has_right_support_point) {
      right_contact_distance = measure_drawer_rod_contact(
        move_group, control, "support",
        drawer_right_tool_.support_contact_link,
        drawer_right_tool_.support_contact_point_local,
        drawer_world_rail_point(
          control, control.right_support_point, slide_after));
    }
    if (left_contact_distance > drawer_support_contact_tolerance_ ||
      right_contact_distance > drawer_support_contact_tolerance_)
    {
      best_effort_drawer_rods_home(
        goal_handle, control, move_group, operation_executed);
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id +
              "' support contact is outside tolerance (left " +
              std::to_string(left_contact_distance) + " m, right " +
              std::to_string(right_contact_distance) + " m, limit " +
              std::to_string(drawer_support_contact_tolerance_) + " m).");
    }
    // 2026-09-03 AGENT §6.3 (P4): hook-retain re-check.  The hook fingers were
    // already closed onto the handles BEFORE this support press (single work
    // pose; no arm has moved since the hook stage).  Verify the hooks are
    // still in contact with the handle plates now that the support rods pushed
    // into the seams — a support press that levered the hooks off the handles
    // must fail loud before the plugin can attach the drawer.  Enforced only
    // when the hook stage itself is sealed (grasp positions > 0), mirroring
    // the stage gates.
    const bool hooks_sealed = drawer_left_tool_.has_gripper_role &&
      drawer_right_tool_.has_gripper_role &&
      drawer_left_tool_.gripper_grasp_position > 0.0 &&
      drawer_right_tool_.gripper_grasp_position > 0.0;
    if (hooks_sealed) {
      double left_hook_distance = std::numeric_limits<double>::infinity();
      double right_hook_distance = std::numeric_limits<double>::infinity();
      if (control.has_left_handle_point) {
        left_hook_distance = measure_drawer_rod_contact(
          move_group, control, "hook retain",
          drawer_left_tool_.gripper_contact_link,
          drawer_left_tool_.gripper_contact_point_local,
          drawer_world_rail_point(
            control, control.left_handle_point, slide_after));
      }
      if (control.has_right_handle_point) {
        right_hook_distance = measure_drawer_rod_contact(
          move_group, control, "hook retain",
          drawer_right_tool_.gripper_contact_link,
          drawer_right_tool_.gripper_contact_point_local,
          drawer_world_rail_point(
            control, control.right_handle_point, slide_after));
      }
      if (left_hook_distance > drawer_gripper_contact_tolerance_ ||
        right_hook_distance > drawer_gripper_contact_tolerance_)
      {
        best_effort_drawer_rods_home(
          goal_handle, control, move_group, operation_executed);
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id +
                "' lost hook contact while extending the support rods (left " +
                std::to_string(left_hook_distance) + " m, right " +
                std::to_string(right_hook_distance) + " m, limit " +
                std::to_string(drawer_gripper_contact_tolerance_) + " m).");
      }
    }
    const double slide_delta = slide_after - slide_before;
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' support rod stage: left support measured %.4f m, right "
      "support measured %.4f m; free drawer slide moved %.4f m during the "
      "press (immobility threshold sealed at AGENT §7.2 cap3).",
      control.id.c_str(), measured_left, measured_right, slide_delta);
    if (std::abs(slide_delta) > 0.02) {
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "Drawer '" + control.id +
              "' support rod press shoved the free drawer by " +
              std::to_string(slide_delta) +
              " m (> 0.02 m); the support press depth needs re-sealing.");
    }
  }

  // 2026-09-04 AGENT §8.4 fix #3 (cap2 verify3 取证): 钩爪封定 = 深压探针 +
  // 双侧同时「压贴立板 + 3s 保持」(AGENT §8.4 fix#3 v2, cap2 run1/run2 取证)。
  // 取代旧「到达 grasp + 稳定性窗口」与 v1「深压穿透证据 + 座封拖抽屉」路径:
  //   v1 phase1 判据「实测杆位 ≥ grasp+0.0015(0.0115)」物理上不可达 —— 把手是
  //   贴前脸的空心立板(mesh x[0,0.069]×z[0.072,0.168], 前脸 x=0.099 世界),
  //   杆端自东沿 +X 压东侧立板面, rigid 面不可穿入; cap2 run1/run2 两侧杆在
  //   ~0.0047-0.006 m 处 ~68 N 顶死 —— 「贴住立板」物理上早已成立, 但杆位永远
  //   到不了 0.0115, 于是压得最正的左杆恒败、右杆偶发 0.023 其实是滑过立板缘
  //   进自由区(无啮合的假阳性)。固定杆位阈值在 rigid 面上对 ±5mm 位姿误差
  //   天生脆弱。v2 证据改用「压贴力」: 顶死时 gripper_joint 实测 effort 高
  //   (I 钳位 ~60-68 N), 悬空 settle / 滑过缘进自由区时 effort 崩落(~0 N) ——
  //   力是唯一能区分「压贴在立板上」与「悬空 / 滑到板旁」的信号。
  //
  //   phase 1 压贴: 两侧夹爪 FJT 同时低速压到 ref = grasp + press_overshoot
  //     (0.015)。rigid 面把杆钉在实测触点(随位姿误差浮动), P+I 在 I 钳位下
  //     维持 ~60 N 压贴力。证据(两侧独立, 持续 1 s):
  //     effort ≥ 力底(gripper_hook_press_force_floor) 且 杆位 ≥ 伸出底(0.001)。
  //     drawer 全程 ≤ 顶开上限(0.0025): 压贴不得把锁止抽屉顶开。
  //   phase 2 保持: 维持压贴 3 s(doc §8.4「低速钩住把手并保持 3 s」), 全程
  //     双侧 effort ≥ 力底 且 drawer ≤ 上限。
  //   cap2 门 = phase1 + phase2 + 调用方既有几何门(measure_drawer_rod_contact
  //     杆端真实点到把手点 ≤ tolerance, doc §7.1 钩距) + 抽屉不可动门。
  //   「座封拖抽屉至 latch 止点」不再判: 关位抽屉被 latch 锁住、hook 只压贴不
  //     互锁, 双侧同时回缩只会失去接触; 啮合的 1:1 拖动由其后 cap8 attach /
  //     cap9-11 抽拉实测抽屉位移证明。
  // 探针参数读自 drawer_tools.<side>.gripper_hook_press_overshoot /
  // gripper_hook_press_force_floor (adapter.yaml); > 0 才进入本路径, 默认 0
  // 由 drive_drawer_hook_stage 走旧到达路径(其他抽屉零回归)。
  //
  //   2026-09-05 AGENT doc §4.1/§4.3 (P1) 取代说明: 上面「fix#3 v2」的 phase 3
  //   （v32/v33 新增的 controller 重置 → 回 home → 慢驱重碰 → 90 s park/freeze
  //   四连环）已从正常路径删除（历史正文保留作失败根因记录）。doc §1.3 定案:
  //   cap3 的不稳定就来自这套封定后的复杂度本身 —— 重置先让杆失去接触再找回,
  //   追赶把混叠位置通道当座封（+10 mm 悬空事故）, park/freeze 长时间把链路
  //   挂在静止判据上。P1 后的 phase 3 = 「原位座封接管」: 压贴保持 3 s 后取
  //   physics 座封中位数, 沿真实伸缩轴有符号投影 extension 算 joint_ref
  //   （= home_joint + extension, clamp 到杆行程 [0,0.120]）。
  //   fix#15（cap2 run A/B/C 取证, 见函数体 phase 3 头注）: 「接管」不降参考 ——
  //   座封 joint_ref 只作记录与复核参照, phase-1/2 深压目标继续命令 press_ref,
  //   不给杆「松压→失贴→拖回 home」的窗口; 不重启控制器、不回家、不二次追赶、
  //   无 park/freeze。home 基准（p_home/j_home/axis）必须在压贴前采样（见函数
  //   体 phase 1 前置块）。输出 press_ref（深压保持参考）由 drive_drawer_hook_stage
  //   填入 DrawerHookHoldState 贯穿 support（doc §4.4 保持矩阵）。
  template<typename GoalHandleT>
  void seal_drawer_hooks_by_probe(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    MoveGroupInterface & move_group,
    double * measured_left,
    double * measured_right,
    bool * in_place_ok,
    bool * operation_executed)
  {
    // 2026-09-05 AGENT doc §4.1 (P1) 头注：本函数实现「钩爪压贴 → 保持 3 s →
    // 原位座封接管」。doc §1.3 已定案：cap3 失败根因是压贴后那套
    // deactivate/configure/activate → 回 home → 从 home 二次追赶 → 90 s
    // park/freeze 的四连环（v32/v33 引入）—— 控制器重启让杆先失接触再找回、
    // 追赶用混叠的位置通道当座封目标，凭空 +10 mm → 悬空。P1 已把这四连环从
    // 正常 db1 打开路径整体删除：压贴与 3 s 保持本身即稳定（phase 1/2 实测），
    // 之后只做一件事 —— 捕获两侧真实座封（gz physics 中位数），按真实伸缩轴
    // 有符号投影算 joint_ref（= home_joint + extension，clamp 到杆行程）。
    // fix#15（cap2 run A/B/C 取证）修正「以新参考立即原位接管」：接管降参考正是
    // 右杆回弹根因 —— ref 0.030→0.016 令积分退火、压贴力塌陷（Rg 36→27 N），
    // 位面无钩挂腔、无机械互锁，杆随即穿内孔回缩力带拖回 home 卡死。故座封投影
    // 只作记录与复核参照，phase-1/2 深压目标继续命令 press_ref，不给失接触窗口。
    // 控制器复位只允许留在异常恢复路径（doc §4.1），正常路径不再调用。
    // 输出约定：正常返回（两侧压贴保持 + 原位接管复核通过）→ *in_place_ok=true，
    // *measured_* = press_ref（深压 ref，doc §4.2 的 valid 语义 —— 下游 support
    // 的 hook_hold override 收到它才会持续正压；座封 joint_ref 只记录不命令）。
    // 任一侧面 home 基准 / 座封
    // 端点捕获失败、投影超门、接管后复核失败 → 一律抛错，整阶段失败进入统一安全
    // 收尾（§4.2「任一侧计算、发送或读回失败，整阶段失败并进入统一安全收尾」）。
    // 本函数不会带着 in_place_ok=false 正常返回 —— 那会把没有实测依据的钩爪深度
    // 放行给支撑/解锁（§4.5「左右任一侧无效时不得进入后续阶段」）。
    const BimanualToolProfile & left_tool = drawer_left_tool_;
    const BimanualToolProfile & right_tool = drawer_right_tool_;
    const std::string left_label = "two-cylinder (left)";
    const std::string right_label = "three-cylinder (right)";
    if (in_place_ok != nullptr) {
      *in_place_ok = false;
    }
    constexpr double kRodSealGraceSeconds = 60.0;  // SIM s: 压贴与保持都要长宽限
    constexpr double kRodSealPollBudget = 90.0;    // wall s
    constexpr double kRodSealSustainSeconds = 1.0; // blip-proof(压贴证据持续)
    constexpr double kRodPressReachFloor = 0.001;  // 杆至少离开 home(防卡死假力)
    constexpr double kHookDrawerPopCeiling = 0.0025; // 压贴/保持不得顶开锁止抽屉
    constexpr double kHookSealHoldSeconds = 3.0;   // doc §8.4「保持 3 s」cap2 门
    constexpr double kRodSealFreeHoverSeconds = 5.0; // 早退: 悬空到 ref 却无力
    const double left_press_ref = left_tool.gripper_grasp_position +
      left_tool.gripper_hook_press_overshoot;
    const double right_press_ref = right_tool.gripper_grasp_position +
      right_tool.gripper_hook_press_overshoot;
    const double left_force_floor = left_tool.gripper_hook_press_force_floor;
    const double right_force_floor = right_tool.gripper_hook_press_force_floor;
    const auto sustained_since = [&](double & good_since, const double now_s,
      const bool good, const double seconds) {
      if (!good) {
        good_since = std::numeric_limits<double>::quiet_NaN();
        return false;
      }
      if (std::isnan(good_since)) {
        good_since = now_s;
        return false;
      }
      return now_s - good_since >= seconds;
    };
    const auto now_since_epoch = []() {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    };

    // ---- 双臂驻留保活 (2026-09-05 fix#6 v32) ----
    // 到达验证发的 12 s 双臂驻留约 14 s 后会被控制器 goal_time_tolerance
    // abort; v31 取证: 驻留 409305.9 → 409319.9 abort, 恰落在 3d 静止验证窗
    // 正中 —— 臂失去指令后杆深度读数被臂松弛/反作用推出污染 (L2 p-p 2.7 mm),
    // 1 s@0.5 mm 稳定门永不闭合。seal 全程各轮询循环按 kRodArmHoldRefresh
    // Seconds 节奏为两臂各重发同一条 12 s 驻留 (2 点同钉 t0=t1=terminal)——
    // 锚 = controller 指令参考 X_arr (arm_controller_reference, ≤ 1 s 新鲜),
    // 不锚实测 act (fix#13 语义: 锚 act 让换轨 t=0 从 X_arr 阶跃到 act, 物理
    // 再沉降 ~O = des−act 一个偏置, 修正自毁)。参考缓存不新鲜时不刷新只告警
    // —— 用实测兜底会注入同样阶跃, 比驻留过期更糟。best-effort: 发送失败只
    // 记日志, 判定仍以实测为准。
    constexpr double kRodArmHoldRefreshSeconds = 9.0; // < 驻留 12 s + 容差 2 s
    constexpr double kRodArmDwellSeconds = 12.0;
    const auto send_arm_hold_dwell = [&](const bool left_side) -> bool {
      const std::string side_label = left_side ? "left" : "right";
      const auto & joint_order = left_side ? left_arm_controller_joint_order_ :
        right_arm_controller_joint_order_;
      if (joint_order.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' %s-arm hold refresh skipped: controller joint order "
          "not cached yet.",
          control.id.c_str(), side_label.c_str());
        return false;
      }
      rclcpp_action::Client<
        control_msgs::action::FollowJointTrajectory> * client =
          left_side ? left_fjt_client_.get() : right_fjt_client_.get();
      if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' %s-arm hold refresh: arm controller action server "
          "unavailable.",
          control.id.c_str(), side_label.c_str());
        return false;
      }
      std::vector<double> terminal;
      if (!arm_controller_reference(left_side, joint_order, terminal)) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' %s-arm hold refresh skipped: controller command "
          "reference is stale (> 1 s); measured anchoring would inject a step.",
          control.id.c_str(), side_label.c_str());
        return false;
      }
      auto goal = control_msgs::action::FollowJointTrajectory::Goal();
      auto & trajectory = goal.trajectory;
      trajectory.joint_names = joint_order;
      const std::size_t joint_count = joint_order.size();
      trajectory.points.resize(2);
      for (std::size_t point_index = 0; point_index < 2; ++point_index) {
        auto & point = trajectory.points[point_index];
        point.positions = terminal;
        point.velocities.assign(joint_count, 0.0);
        point.accelerations.assign(joint_count, 0.0);
        point.time_from_start.sec = static_cast<int32_t>(
          point_index == 0 ? 0 : kRodArmDwellSeconds);
        point.time_from_start.nanosec = 0;
      }
      goal.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);
      goal.goal_tolerance.resize(joint_count);
      for (std::size_t index = 0U; index < joint_count; ++index) {
        auto & tolerance = goal.goal_tolerance[index];
        tolerance.name = trajectory.joint_names[index];
        tolerance.position = 0.002;
        tolerance.velocity = 0.0;
        tolerance.acceleration = 0.0;
      }
      auto & slot = left_side ? bimanual_controller_handles_->left :
        bimanual_controller_handles_->right;
      try {
        check_cancel(goal_handle);
        const auto send_future = client->async_send_goal(goal);
        const auto accept_deadline = std::chrono::steady_clock::now() +
          std::chrono::seconds(5);
        while (send_future.wait_for(50ms) != std::future_status::ready) {
          check_cancel(goal_handle);
          if (std::chrono::steady_clock::now() >= accept_deadline) {
            RCLCPP_WARN(
              get_logger(),
              "Drawer '%s' %s-arm hold refresh goal was not accepted in time.",
              control.id.c_str(), side_label.c_str());
            return false;
          }
        }
        const auto accepted = send_future.get();
        if (!accepted) {
          RCLCPP_WARN(
            get_logger(),
            "Drawer '%s' %s-arm controller rejected the hold refresh goal.",
            control.id.c_str(), side_label.c_str());
          return false;
        }
        {
          std::lock_guard<std::mutex> lock(bimanual_controller_handles_->mutex);
          slot = accepted;
        }
        return true;
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' %s-arm hold refresh was not armed: %s",
          control.id.c_str(), side_label.c_str(), error.what());
        return false;
      }
    };
    double last_arm_hold_refresh_s = std::numeric_limits<double>::quiet_NaN();
    const auto refresh_arm_holds_if_due = [&]() {
      const double now_s = now_since_epoch();
      if (!std::isnan(last_arm_hold_refresh_s) &&
        now_s - last_arm_hold_refresh_s < kRodArmHoldRefreshSeconds)
      {
        return;
      }
      last_arm_hold_refresh_s = now_s;
      send_arm_hold_dwell(true);
      send_arm_hold_dwell(false);
    };

    // ---- (2026-09-05 AGENT doc §4.3 P1) phase-1 前置: home 物理基准 + 伸缩轴 ----
    // P1 把座封参考按「真实伸缩轴的有符号投影」计算(禁止欧氏范数: 欧氏会把横向
    // 抖动混入行程, doc §4.3)。home 基准必须在压贴前采样 —— 此刻 gripper 杆仍在
    // home(q≈0)、控制器 ref≈0、位置通道干净(~0 ± 0.1 mm); 压贴后位置通道在
    // load 下无意义(v33)。压贴结束后 phase 3 再取 p_seat 投影 extension =
    // dot(p_seat − p_home, axis), joint_ref = clamp(j_home + extension,
    // [0, 行程上限])(doc §4.3 公式)。
    constexpr std::size_t kRodPhysicsCaptureSamples = 8;
    constexpr double kRodPhysicsCaptureSpacingSeconds = 0.12;
    constexpr std::size_t kRodPhysicsMinSamples = 4;  // < 此数 → 捕获失败
    const auto median_world_point = [](
      std::vector<Eigen::Vector3d> & samples) -> Eigen::Vector3d {
      Eigen::Vector3d median;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        std::vector<double> values;
        values.reserve(samples.size());
        for (const auto & sample : samples) {
          values.push_back(sample[axis]);
        }
        const auto mid = values.begin() +
          static_cast<std::ptrdiff_t>(values.size() / 2);
        std::nth_element(values.begin(), mid, values.end());
        median[axis] = *mid;
      }
      return median;
    };
    const auto capture_gripper_end = [&](const BimanualToolProfile & tool,
      const std::string & side_label, const char * what)
      -> std::optional<Eigen::Vector3d> {
      std::vector<Eigen::Vector3d> samples;
      samples.reserve(kRodPhysicsCaptureSamples);
      for (std::size_t i = 0; i < kRodPhysicsCaptureSamples; ++i) {
        refresh_arm_holds_if_due();
        check_cancel(goal_handle);
        const auto end = drawer_physics_link_point(
          tool.gripper_contact_link, tool.gripper_contact_point_local);
        if (end.has_value()) {
          samples.push_back(*end);
        }
        if (i + 1 < kRodPhysicsCaptureSamples) {
          interruptible_hold(goal_handle, kRodPhysicsCaptureSpacingSeconds);
        }
      }
      if (samples.size() < kRodPhysicsMinSamples) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' hook %s physics capture (%s side) got only %zu/%zu "
          "clean samples.",
          control.id.c_str(), what, side_label.c_str(), samples.size(),
          kRodPhysicsCaptureSamples);
        return std::nullopt;
      }
      return median_world_point(samples);
    };
    struct HookHomeBase
    {
      double j_home{0.0};
      Eigen::Vector3d home_end{Eigen::Vector3d::Zero()};
      Eigen::Vector3d axis_world{Eigen::Vector3d(0.0, 0.0, 1.0)};
      bool ok{false};
    };
    const auto capture_home_base = [&](const BimanualToolProfile & tool,
      const std::string & side_label) -> HookHomeBase {
      HookHomeBase base;
      const auto home_end = capture_gripper_end(tool, side_label, "home");
      if (!home_end.has_value()) {
        return base;  // ok 保持 false → 调用方(phase 3)整阶段失败(doc §4.2)
      }
      const auto state = synchronized_current_robot_state(move_group);
      base.j_home = state->getVariablePosition(tool.gripper_joint);
      if (!std::isfinite(base.j_home)) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' hook home joint read (%s side) is not finite.",
          control.id.c_str(), side_label.c_str());
        return base;
      }
      base.home_end = *home_end;
      // 杆沿 local -Z 伸出; 轴 = 该连杆姿态旋转 (0,0,-1)。两臂在 seal 全程冻结,
      // 姿态恒定, 故轴只需取一次(home 时刻)。MoveIt FK 与 physics 的差异在位置
      // 域; 姿态差 ~deg 级, 对 ≤25 mm 行程的投影误差 ≤0.5 mm, 远低于行程门。
      base.axis_world = state->getGlobalLinkTransform(
        tool.gripper_contact_link).linear() *
        Eigen::Vector3d(0.0, 0.0, -1.0);
      base.ok = true;
      return base;
    };
    const HookHomeBase left_home_base = capture_home_base(left_tool, left_label);
    const HookHomeBase right_home_base =
      capture_home_base(right_tool, right_label);
    // ---- phase 1: 双侧同时低速压贴立板 ----
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' hook seal (AGENT §8.4 fix#3 v2): press probe, left/right "
      "ref %.4f / %.4f m (press-force floors %.1f / %.1f N).",
      control.id.c_str(), left_press_ref, right_press_ref,
      left_force_floor, right_force_floor);
    const std::vector<double> left_press_desired = drawer_rod_stage_desired(
      left_tool, control, move_group, "hook", left_press_ref);
    const std::vector<double> right_press_desired = drawer_rod_stage_desired(
      right_tool, control, move_group, "hook", right_press_ref);
    PendingRodGoal left_press = accept_drawer_rod_goal(
      two_cylinder_fjt_client_, left_label, goal_handle, move_group,
      left_tool.calibration_joint_names, left_press_desired,
      kRodSealGraceSeconds);
    PendingRodGoal right_press = accept_drawer_rod_goal(
      unlock_motor_fjt_client_, right_label, goal_handle, move_group,
      right_tool.calibration_joint_names, right_press_desired,
      kRodSealGraceSeconds);
    *operation_executed = true;
    double left_measured = std::numeric_limits<double>::quiet_NaN();
    double right_measured = std::numeric_limits<double>::quiet_NaN();
    double left_effort = 0.0;
    double right_effort = 0.0;
    double left_good_since = std::numeric_limits<double>::quiet_NaN();
    double right_good_since = std::numeric_limits<double>::quiet_NaN();
    double left_free_since = std::numeric_limits<double>::quiet_NaN();
    double right_free_since = std::numeric_limits<double>::quiet_NaN();
    bool press_sealed = false;
    double drawer_position = button_snapshot(control).position;
    const auto press_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(kRodSealPollBudget);
    while (std::chrono::steady_clock::now() < press_deadline) {
      refresh_arm_holds_if_due();
      check_cancel(goal_handle);
      interruptible_hold(goal_handle, 0.05);
      left_measured = read_real_joint_position(
        move_group, left_tool.gripper_joint, left_label);
      right_measured = read_real_joint_position(
        move_group, right_tool.gripper_joint, right_label);
      left_effort = read_real_joint_effort(
        move_group, left_tool.gripper_joint, left_label);
      right_effort = read_real_joint_effort(
        move_group, right_tool.gripper_joint, right_label);
      drawer_position = button_snapshot(control).position;
      if (drawer_position > kHookDrawerPopCeiling) {
        throw OperationError(
                PressCabinetButton::Result::EXECUTION_FAILED,
                "Drawer '" + control.id + "' hook press shoved the latched "
                "drawer to " + std::to_string(drawer_position) + " m (> " +
                std::to_string(kHookDrawerPopCeiling) + " m ceiling); the "
                "press depth needs re-sealing.");
      }
      const double now_s = now_since_epoch();
      // Pressed = a real blocked press: rod left home AND held joint effort is
      // at/above the force floor.  A rod hovering short in free air settles at
      // ~0 N; one that slips around the plate edge into free space collapses
      // to ~0 N too — both fail the force test (run2's 0.023 'pop-through').
      const bool left_pressed = left_measured >= kRodPressReachFloor &&
        left_effort >= left_force_floor;
      const bool right_pressed = right_measured >= kRodPressReachFloor &&
        right_effort >= right_force_floor;
      const bool left_good = sustained_since(
        left_good_since, now_s, left_pressed, kRodSealSustainSeconds);
      const bool right_good = sustained_since(
        right_good_since, now_s, right_pressed, kRodSealSustainSeconds);
      if (left_good && right_good) {
        press_sealed = true;
        break;
      }
      // Fail fast with a clean diagnosis instead of burning the poll budget: a
      // rod already at its press ref in free air with no force is hovering
      // short of the plate (pose too far / wrong aim).
      const bool left_free_hover = left_measured >= left_press_ref - 0.0005 &&
        left_effort < left_force_floor;
      const bool right_free_hover = right_measured >= right_press_ref - 0.0005 &&
        right_effort < right_force_floor;
      if (sustained_since(
            left_free_since, now_s, left_free_hover,
            kRodSealFreeHoverSeconds)) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id + "' left hook press reached its ref ("
                + std::to_string(left_press_ref) + " m) in free air with no "
                "contact force (" + std::to_string(left_effort) + " N < " +
                std::to_string(left_force_floor) + " N); the work pose is too "
                "far from the handle plate.");
      }
      if (sustained_since(
            right_free_since, now_s, right_free_hover,
            kRodSealFreeHoverSeconds)) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id + "' right hook press reached its ref ("
                + std::to_string(right_press_ref) + " m) in free air with no "
                "contact force (" + std::to_string(right_effort) + " N < " +
                std::to_string(right_force_floor) + " N); the work pose is too "
                "far from the handle plate.");
      }
    }
    if (!press_sealed) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' hook press force never held: left "
              "measured " + std::to_string(left_measured) + " m / " +
              std::to_string(left_effort) + " N (need >= " +
              std::to_string(kRodPressReachFloor) + " m and >= " +
              std::to_string(left_force_floor) + " N), right measured " +
              std::to_string(right_measured) + " m / " +
              std::to_string(right_effort) + " N (need >= " +
              std::to_string(kRodPressReachFloor) + " m and >= " +
              std::to_string(right_force_floor) + " N).");
    }
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' hook press sealed: left %.4f m @ %.1f N, right %.4f m @ "
      "%.1f N.",
      control.id.c_str(), left_measured, left_effort, right_measured,
      right_effort);

    // ---- phase 2: 保持压贴 3 s (doc §8.4「低速钩住把手并保持 3 s」cap2 门) ----
    // 旧「座封拖抽屉至 latch 止点」判据已删除: 关位抽屉被 latch 锁住、hook 只
    // 压贴不互锁(flush 立板无钩挂腔), 双侧同时回缩只会让杆失去接触、无法拖出;
    // 啮合的 1:1 拖动由其后 cap8 attach / cap9-11 抽拉阶段实测抽屉位移证明。
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' hook hold probe: keeping BOTH claws pressed for %.1f s.",
      control.id.c_str(), kHookSealHoldSeconds);
    const auto hold_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(kHookSealHoldSeconds);
    while (std::chrono::steady_clock::now() < hold_deadline) {
      refresh_arm_holds_if_due();
      check_cancel(goal_handle);
      interruptible_hold(goal_handle, 0.05);
      drawer_position = button_snapshot(control).position;
      left_effort = read_real_joint_effort(
        move_group, left_tool.gripper_joint, left_label);
      right_effort = read_real_joint_effort(
        move_group, right_tool.gripper_joint, right_label);
      if (drawer_position > kHookDrawerPopCeiling ||
        left_effort < left_force_floor ||
        right_effort < right_force_floor)
      {
        throw OperationError(
                PressCabinetButton::Result::EXECUTION_FAILED,
                "Drawer '" + control.id + "' hook press lost during the " +
                std::to_string(kHookSealHoldSeconds) + " s hold: left effort " +
                std::to_string(left_effort) + " N (floor " +
                std::to_string(left_force_floor) + "), right effort " +
                std::to_string(right_effort) + " N (floor " +
                std::to_string(right_force_floor) + "), drawer " +
                std::to_string(drawer_position) + " m (ceiling " +
                std::to_string(kHookDrawerPopCeiling) + ").");
      }
    }

    // ---- (2026-09-05 AGENT doc §4.1/§4.3 P1 + fix#15) phase 3: 原位座封接管 ----
    // 取代旧 fix#7 v33 的 deactivate/configure/activate → 回 home → 慢驱重碰 →
    // 90 s park/freeze 四连环(doc §1.3 定案: 该复杂度本身即 cap3 不稳定根因,
    // 属 §4.1 删除项)。phase 1/2 的压贴+3 s 保持已证明稳定(力证据 + 保持窗口);
    // fix#15(cap2 run A/B/C 取证, 见下): 接管绝不降参考 —— phase-1/2 的深压目标
    // 继续命令 press_ref, 座封只捕获/投影/复核, 不给杆制造「先失去接触再找回
    // 接触」的窗口。控制器复位只允许在异常恢复路径出现。
    if (!left_home_base.ok || !right_home_base.ok) {
      // doc §4.2/§4.5 (P1): 任一侧计算失败 = 整阶段失败。座封参考的 home 基准
      // （p_home/j_home/axis）必须在压贴前取得；physics 不可用（真机 / gazebo
      // 实体服务故障）时投影无从谈起。静默回退 legacy 静态 grasp 会让支撑/解锁
      // 阶段带着没有实测依据的钩爪深度前进 —— 这正是 §4.5「左右任一侧无效时不
      // 得进入后续阶段」禁止的。抛错 → 调用方统一收尾（电缸回位 + 退让 + 显式
      // 复闩）；此刻抽屉仍未解锁闩定，重试安全。控制器复位不在此路径。
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' hook home physics capture "
              "unavailable (left " +
              std::string(left_home_base.ok ? "ok" : "unavailable") +
              ", right " +
              std::string(right_home_base.ok ? "ok" : "unavailable") +
              "); cannot take over onto the seat without a pre-press home base "
              "(AGENT doc §4.2 whole-stage failure).");
    }
    const auto left_seat_end = capture_gripper_end(left_tool, left_label, "seat");
    const auto right_seat_end =
      capture_gripper_end(right_tool, right_label, "seat");
    if (!left_seat_end.has_value() || !right_seat_end.has_value()) {
      // doc §4.2/§4.5 (P1): 座封端点读回失败 = 整阶段失败 —— 两侧均已压贴保持
      // 3 s，但稳定后的真实端点没读到，就没有 joint_ref，绝不允许带病进入支撑/
      // 解锁（§4.5「左右任一侧无效时不得进入后续阶段」）。抛错 → 调用方统一
      // 收尾电缸回位；抽屉仍闩定，重试安全。
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' hook seat physics capture failed "
              "(left " +
              std::string(left_seat_end.has_value() ? "ok" : "unavailable") +
              ", right " +
              std::string(right_seat_end.has_value() ? "ok" : "unavailable") +
              "); cannot project the in-place seat takeover (AGENT doc §4.2 "
              "whole-stage failure).");
    }
    // 有符号投影(§4.3 公式): extension = dot(p_seat − p_home, axis);
    // joint_ref = clamp(j_home + extension, [0, 行程上限])。横向残差 =
    // |位移 − extension·axis|; 残差或行程超门 → 座封与 home 不一致(伪差/轴错/
    // 臂漂)即抛, 不静默带病前进。
    // 2026-09-05 fix#14 (cap2 run A 取证, §4.6 先取证再调参): 横向残差上限放宽
    // 到 3 mm。run A 左钩 1.716 mm 横向残差发生在"未加载 home → 压贴 3 s 保持
    // 后 seat"的跨载荷比较：10-23 N 密封力下臂/工具座有 ~1-2 mm 量级载荷挠度
    // （home 空载、seat 载荷状态不同），横向残差混入该挠度属正常密封形变，并非
    // 钩脱板。真正的脱钩/错板会先让压贴力塌穿 phase-1/2 力地板（≥5 mm 脱板即
    // 失力被拦），几何上也是 ≥5 mm 量级 —— 3 mm 仍拦得住，且 phase-1 力地板 +
    // 3 s 保持这些真安全断言（§8.2）本就不依赖本带。doc §4.3 公式本无横向带，
    // 本带只是防"明显不一致座封"带病进入支撑/解锁的自检。
    constexpr double kRodSeatTravelSanityMin = 0.0015;  // m: 行程合理性下界
    constexpr double kRodSeatTravelSanityMax = 0.025;   // m: 上界(远低于行程上限)
    // 2026-09-06 fix#18 (cap3 run C 取证): 3 mm 带低于当前测量噪声地板 —— 同
    // 一压贴健康密封时 self-center 实测 physics tip-spread 达 L 3.0 / R 3.3-4.0
    // mm, 而 run C 左钩在「0.0159 m @ 17.6 N / 0.0145 m @ 45.1 N 压贴 + 3 s 保持
    // 通过」的健康密封上投影横残 3.472 mm 触发误报。横向残差只对「垂直杆轴的横向
    // 不一致」敏感; 真实脱钩是沿轴回缩, 由行程带/接管下带 + phase-1/2 力地板拦截
    // （正交）。放宽到 5 mm 只把自检门抬到噪声之上, 不削弱轴向脱钩防护。
    constexpr double kRodSeatTransverseMax = 0.0050;    // m: 横向残差上限(fix#18 3→5 mm)
    constexpr double kRodSeatJointClampMax = 0.120;     // m: URDF finger-rod 上限
    const auto seat_joint_ref = [&](const HookHomeBase & home_base,
      const Eigen::Vector3d & seat_end,
      const std::string & side_label) -> double {
      // 有符号投影/裁剪逻辑在纯头 drawer_hook_seat_projection.hpp 内(doc §4.3:
      // 「端点位移转关节位移」只有一份实现, §4.5 纯逻辑单测覆盖正/负/横向扰动)。
      const auto projection = project_drawer_hook_seat(
        seat_end.x(), seat_end.y(), seat_end.z(),
        home_base.home_end.x(), home_base.home_end.y(),
        home_base.home_end.z(),
        home_base.axis_world.x(), home_base.axis_world.y(),
        home_base.axis_world.z(),
        home_base.j_home, 0.0, kRodSeatJointClampMax,
        kRodSeatTravelSanityMin, kRodSeatTravelSanityMax,
        kRodSeatTransverseMax);
      if (!projection.ok) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id + "' hook seat projection is out of "
                "range (" + side_label + ": extension " +
                std::to_string(projection.extension) + " m, expect " +
                std::to_string(kRodSeatTravelSanityMin) + "-" +
                std::to_string(kRodSeatTravelSanityMax) + " m; transverse "
                "residual " +
                std::to_string(projection.transverse_residual) + " m > " +
                std::to_string(kRodSeatTransverseMax) +
                " m) — the seat capture disagrees with the pre-press home; "
                "re-seal the hooks.");
      }
      return projection.joint_ref;
    };
    const double left_joint_ref = seat_joint_ref(
      left_home_base, *left_seat_end, left_label);
    const double right_joint_ref = seat_joint_ref(
      right_home_base, *right_seat_end, right_label);
    const Eigen::Vector3d left_disp = *left_seat_end - left_home_base.home_end;
    const Eigen::Vector3d right_disp =
      *right_seat_end - right_home_base.home_end;
    const double left_extension = left_disp.dot(left_home_base.axis_world);
    const double right_extension = right_disp.dot(right_home_base.axis_world);
    // fix#14: 座封相对 home 的横向残差也落到成功日志（取证累积）——旧代码只在
    // 超门抛错时打印，正常通过的每次密封没有横向证据。真实脱钩先塌 phase-1/2
    // 力地板、几何上 ≥5 mm；≤3 mm 为载荷挠度/读数噪声正常量级，不否决。
    const double left_transverse =
      (left_disp - left_extension * left_home_base.axis_world).norm();
    const double right_transverse =
      (right_disp - right_extension * right_home_base.axis_world).norm();
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' hook seat takeover (AGENT doc P1 fix#15 in-place, no ref "
      "drop): projected travel left %.4f m / right %.4f m (home joint %.4f / "
      "%.4f m) -> seat joint refs %.4f / %.4f m (record only); hooks KEEP the "
      "deep-press refs %.4f / %.4f m; transverse residual left %.4f m / right "
      "%.4f m (band %.4f m).",
      control.id.c_str(), left_extension, right_extension,
      left_home_base.j_home, right_home_base.j_home, left_joint_ref,
      right_joint_ref, left_press_ref, right_press_ref, left_transverse,
      right_transverse, kRodSeatTransverseMax);
    // ---- fix#15 (cap2 run A/B/C 取证, AGENT §4.3 P1 修正): 接管不降参考 ----
    // 旧相位在接管时把夹爪 FJT 参考从深压 ref(press_ref 0.030) 降到位面贴合座封
    // joint_ref(~0.014-0.016), 同时目标基本到位使积分退火、压贴力塌陷 ——
    // rod_trace_cap2*.log 三连取证: 接管瞬间 ref 0.030→0.016 且 effort 同步崩
    // (Lg 51→40 N, Rg 36→27 N); 而深压(0.030 顶死位面)此前已稳定 20+ s。参考
    // 回落后右杆先失贴(0.0156→0.0114→0.0077→0.0049, 穿 bore-guide 内回缩力带
    // 拖回 ~0.003-0.005 卡死), 左杆 0.0152→0.0129 靠 ~40 N 勉强回稳; 抬回 ref
    // 0.017 即恢复到 ~0.0146@30 N —— 证据链指向「降参考 = 松压 = 失贴」。
    // flush 立板无钩挂腔, 「座封」本无机械互锁 —— 保持只能靠正向压贴, 而压贴力
    // 又由「参考仍高于物理贴位」维持(位面把杆钉在实测触点, P+I 输出 ~36-51 N)。
    // 故 fix#15 不再下发降参考的接管目标: phase-1/2 的深压目标(kRodSealGraceSeconds
    // = 60 s 宽限, 从发送到 support 目标抢占远小于剩余宽限)继续命令 press_ref,
    // 杆被位面钉住 —— 不给任何「先松压再找回」窗口。座封投影(extension /
    // joint_ref)仅作记录与复核参照, 不再是新的命令参考。本阶段不发新目标。
    // 复核窗口(1.5 s): 不判到达(无新目标即无移动可到达), 只盯「没滑脱 / 没冲出」。
    // 单向带标定(2026-09-06 fix#19, 沿 fix#18 校准法): 本窗口统计的是 1.5 s 内
    // physics 触点重构位移的 min/max 包络(非逐样稳态)。cap2 run1 失败取证: 左/右
    // 杆全程正压(复核末 effort 仍 ≥ 力地板, 见下方正压复核), 但左包络 0.0072/
    // 0.0166 m 绕座封 0.0112 双侧摆动 ±4.7 mm、右 ±1.6 mm —— 与 fix#18 已定标的
    // 触点重构噪声地板(3-4 mm tip-spread)同源, 3 mm 单向带低于该包络而误报。
    // 下带 3→5 mm: 介于噪声包络(~±4.7 mm 实测上限)与真实内孔滑脱塌陷之间 ——
    // 真实滑脱把杆拖回 ~home(位移 ≥ 6-8 mm, 见 rod_trace 拖回 0.003-0.005 取证)
    // 仍会被 5 mm 下带拦下, 且滑脱不会在 1.5 s 内自愈; 上带 6→8 mm: 顶开/冲出是
    // 整杆外伸(≥ 十余 mm), 而本次噪声上摆 +5.4 mm 已逼近 6 mm 旧线, 留余量防
    // 误报。交接态的正压仍由窗口后的 effort 力地板 + 下游 hook-retain 复核把关。
    // 抽屉仍闩定不顶开; physics 读数偶发缺失时该侧按未验证处理 —— 不把 1.5 s 采样
    // 抖动做成新的脆弱冻结窗(doc §4.1)。此后进入支撑由调用方几何门把关(hook-retain
    // 复核 + slide 不可动门 + 本函数输出压贴 ref 的正压保持)。
    // 2026-09-06 fix#20 (cap2 r3 取证, 保留): 复核窗口内绝不触发 9 s 臂驻留刷新 ——
    // 避免把负载位形重钉回空载参考造成行程瞬时塌缩误报; 刷新一律移到窗口后。该修复
    // 单独验证不充分(r3/r2/rA 仍 FAIL, 右钩回缩依旧), 其「ref 未变时杆不可能自行回缩」
    // 推论被 rod_trace 证伪(见 fix#21)。
    // 2026-09-06 fix#21 (cap2 r3/r2/rA 取证, 本实现): 复核判定从「固定 1.5 s 原始
    // min/max 包络」改为「双侧连续在位确认」, 并改正 fix#20 的错误推论。rod_trace
    // (fix20 日志 t=75-118)显示: 右钩 Rg 深压保持期 ref 恒 0.030、窗口无任何杆/臂目标,
    // 但 Rg 实际关节仍周期性瞬时回缩 4-10 mm、0.1-0.7 s 内自愈 —— 接触颤振(回缩瞬间
    // effort 反升至 ~25 N, 控制器仍在顶压), 与 fix#15 真滑脱「ref 松 + effort 崩 +
    // 杆不回弹卡死 ~0.003-0.005」正交。左钩 Lg 同参 rock-steady(座封 0.018 恒定)。
    // 原始 min ≥ ext−5 mm 无法区分「瞬态自愈颤振」与「持续滑脱」, 于是把三连运行中
    // 2/3 的右钩颤振误判为滑脱(r3 0.0038 / r2 0.0052 / rA 0.0068 vs seat
    // 0.0115/0.0144/0.0137)。fix#21 判定改为观察「双侧连续在位」:
    //   (1) 最短被动观察 base 1.5 s(保留 fix#15「接管瞬间不滑」语义, 见上注释);
    //   (2) 此后双侧「各自最近一次连续在带内」≥ confirm 0.4 s 即确认接管 —— 颤振的
    //       带内间隔(实测 1-2 s)天然满足, 真滑脱/卡死永远凑不齐 0.4 s;
    //   (3) 某侧连续低于下带 > 1.0 s(> 实测最长颤振 0.7 s)或行程高于上带 = 立即失败;
    //   (4) 观察硬上限 2.6 s 内未确认 = 失败。
    // 窗口仍全程被动不发目标(fix#20); 2.6 s 上限在旧驻留(≤9 s 前发, 12 s + 2 s 容差)
    // 寿命内, 循环后照常补刷。抽屉闩定关闭、cap2 无可逃逸; 真滑脱另有窗口后 effort
    // 力地板 + 下游 hook-retain 复核兜底拦截。
    constexpr double kRodSeatTakeoverBaseSeconds = 1.5;  // s: 最短被动观察(fix#15 语义)
    constexpr double kRodSeatTakeoverMaxSeconds = 2.6;   // s: 观察硬上限(自愈留余量)
    constexpr double kRodSeatSlipDownBand = 0.005;  // m: 座封-5 mm 单向底线(fix#19)
    constexpr double kRodSeatSlipUpBand = 0.008;    // m: 座封+8 mm 单向顶线(fix#19)
    constexpr double kRodSeatConfirmSeconds = 0.4;  // s: 双侧连续在位确认时长(fix#21)
    constexpr double kRodSeatSlipMaxSeconds = 1.0;  // s: 连续低于下带=真滑脱时限(fix#21)
    // 2026-09-06 AGENT doc §5.1: fix#21 的时序判断抽到 SeatConfirmWindow 纯逻辑
    // （drawer_seat_confirm.hpp，含合成事件单测）。语义与本处注释保持一致:
    //   * 在位计时只由「双侧同帧新鲜在带样本」(配对帧)累计 —— 同步查询 / 单侧缺数
    //     的墙钟从不直接算在位时长; 断数超过采样预算(0.5 s)即打断连续计时;
    //   * 行程高于上带 = 超界, 新鲜样本即败; 连续低于下带 > slip 时限 = 持续回缩;
    //   * base/max 是墙钟预算(最短被动观察 / 安全硬上限), 不直接计入在位时长。
    const auto verify_start = std::chrono::steady_clock::now();
    SeatConfirmParams seat_params;
    seat_params.base_seconds = kRodSeatTakeoverBaseSeconds;
    seat_params.max_seconds = kRodSeatTakeoverMaxSeconds;
    seat_params.confirm_seconds = kRodSeatConfirmSeconds;
    seat_params.slip_max_seconds = kRodSeatSlipMaxSeconds;
    seat_params.down_band_m = kRodSeatSlipDownBand;
    seat_params.up_band_m = kRodSeatSlipUpBand;
    seat_params.fresh_budget_seconds = 0.5;  // 采样预算: 正常复核节拍远小于此
    SeatConfirmWindow seat_confirm(seat_params);
    seat_confirm.Reset(0.0);
    const auto mono_seconds_since_start = [&]() {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - verify_start).count();
    };
    bool seat_confirmed = false;
    while (mono_seconds_since_start() < kRodSeatTakeoverMaxSeconds) {
      // fix#20/fix#21: 复核全程被动观察 —— 不发任何臂/杆目标(见上注释), 只把每帧
      // 新鲜样本喂给 SeatConfirmWindow, 由它做配对在位计时与超界/回缩/缺数判别。
      check_cancel(goal_handle);
      interruptible_hold(goal_handle, 0.05);
      drawer_position = button_snapshot(control).position;
      if (drawer_position > kHookDrawerPopCeiling) {
        throw OperationError(
                PressCabinetButton::Result::EXECUTION_FAILED,
                "Drawer '" + control.id + "' seat takeover let the latched "
                "drawer drift to " + std::to_string(drawer_position) +
                " m (> " + std::to_string(kHookDrawerPopCeiling) + " m "
                "ceiling).");
      }
      left_effort = read_real_joint_effort(
        move_group, left_tool.gripper_joint, left_label);
      right_effort = read_real_joint_effort(
        move_group, right_tool.gripper_joint, right_label);
      SeatConfirmSideFrame left_frame;
      const auto left_end = drawer_physics_link_point(
        left_tool.gripper_contact_link, left_tool.gripper_contact_point_local);
      if (left_end.has_value()) {
        left_frame.present = true;
        left_frame.travel = (*left_end - left_home_base.home_end)
          .dot(left_home_base.axis_world);
        left_frame.seat = left_extension;
      }
      SeatConfirmSideFrame right_frame;
      const auto right_end = drawer_physics_link_point(
        right_tool.gripper_contact_link,
        right_tool.gripper_contact_point_local);
      if (right_end.has_value()) {
        right_frame.present = true;
        right_frame.travel = (*right_end - right_home_base.home_end)
          .dot(right_home_base.axis_world);
        right_frame.seat = right_extension;
      }
      // 配对在位确认 / 超界即时败 / 掉带超时败 / 缺数打断(>预算) 都在窗口内判定;
      // Describe() 明确区分「超界 / 持续回缩 / 持续缺数 / 采样过旧 / 机械振荡」。
      const SeatConfirmStatus seat_status = seat_confirm.Update(
        mono_seconds_since_start(), left_frame, right_frame);
      if (seat_status == SeatConfirmStatus::kConfirmed) {
        seat_confirmed = true;
        break;
      }
      if (seat_status != SeatConfirmStatus::kPending) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer '" + control.id +
                "' hook rod seat takeover did not verify (" +
                seat_confirm.Describe(mono_seconds_since_start()) + ").");
      }
    }
    if (!seat_confirmed) {
      // 硬上限内未确认(持续掉带、冲出或读数缺失): 用窗口诊断区分缺数/过旧/振荡。
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' hook rod could not be confirmed "
              "seated within " + std::to_string(kRodSeatTakeoverMaxSeconds) +
              " s during the in-place takeover (" +
              seat_confirm.Describe(mono_seconds_since_start()) + ").");
    }
    // fix#20: 观察结束立即续上 9 s 臂驻留 —— 旧驻留(观察前 ≤9 s 发送)寿命在观察期
    // (≤2.6 s)内已消耗, 此处刷新保证下一阶段(支撑)臂控目标不因 goal_time_tolerance
    // 超时脱手。刷新放在座封确认之后, 其重钉扰动不再污染判定。
    refresh_arm_holds_if_due();
    // 正向压贴力复核: flush 面上「仍贴住」的力信号。深压目标下位面把杆钉在实测
    // 触点, effort 应稳定在力底之上; 某侧跌穿 = 已失正压(脱板 / 滑脱先兆)。
    if (left_effort < left_force_floor || right_effort < right_force_floor) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id + "' hook lost its positive press right "
              "after the in-place takeover (left effort " +
              std::to_string(left_effort) + " N < " +
              std::to_string(left_force_floor) + " N floor, right effort " +
              std::to_string(right_effort) + " N < " +
              std::to_string(right_force_floor) + " N floor).");
    }
    drawer_position = button_snapshot(control).position;
    // fix#15: 向下游(support 的 hook_hold override)输出的保持参考 = 深压 ref,
    // 而不是随积分退火失力的座封位。flush 立板场景唯一稳定的保持态就是「参考仍
    // 高于物理贴位」的正压; support 阶段收到 press_ref 会再次顶紧(短暂重压后回到
    // 满压, 支撑杆收敛判据只盯支撑杆不受影响) —— 这正是 §4.4「钩爪保持」在该
    // 几何下的物理实现。钩爪实际贴位由位面决定(~座封 joint_ref), 不复核 ref。
    if (in_place_ok != nullptr) {
      *in_place_ok = true;
    }
    *measured_left = left_press_ref;
    *measured_right = right_press_ref;
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' hook seat takeover verified (AGENT doc P1 fix#15): left "
      "press ref %.4f m @ %.1f N, right press ref %.4f m @ %.1f N; hooks keep "
      "their deep-press hold with no reference drop / controller reset / "
      "re-home / re-chase / park in the normal path.",
      control.id.c_str(), left_press_ref, left_effort, right_press_ref,
      right_effort);
    // phase-1/2 压贴目标继续命令深压 ref, 由下一阶段(支撑)或收尾目标抢占 ——
    // 本阶段判定依赖 physics 投影钉位 + 压贴力, 从不依赖被抢占目标的完成; 电缸
    // 回位由各 cap 收尾 / 阶段 epilogue 负责, 无长期挂起驱动。
  }

  // 2026-09-03 AGENT §4.2/§6.1 (P4): hook stage.  Arms are at the single work
  // pose with the hooks still open; this closes both GRIPPER rods from open to
  // their §4.2 grasp positions so the real rod ends press the two handle
  // plates (latched-closed drawer: tips rest on the plate face; free drawer:
  // press-depth bite), then verifies arrival + stability.  Runs BEFORE the
  // support stage (the pre-P4 grasp-after-support order is gone), so the
  // support rods are still retracted here and need no keep-alive handling.
  // 2026-09-04 fix#3 v2: when the drawer_tools.<side>.gripper_hook_press_*
  // probe parameters (press_overshoot + press_force_floor) are sealed (> 0),
  // the force-confirmed press+hold seal path above replaces the legacy
  // arrival (which never engaged: cap2 run1/run2 showed the rigid handle
  // plate is not penetrable and position-past-face evidence is unreachable —
  // run2's 0.023 was a slip-around free-space false positive); otherwise the
  // legacy path below is kept unchanged for unsealed drawers.
  template<typename GoalHandleT>
  void drive_drawer_hook_stage(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    MoveGroupInterface & move_group,
    bool * operation_executed,
    DrawerHookHoldState * hook_hold)
  {
    // Hook gates are independent of the support stage: hooks close whenever
    // their own grasp positions are sealed, whether or not supports are.
    const bool left_grip = drawer_left_tool_.has_gripper_role &&
      drawer_left_tool_.gripper_grasp_position > 0.0;
    const bool right_grip = drawer_right_tool_.has_gripper_role &&
      drawer_right_tool_.gripper_grasp_position > 0.0;
    if (hook_hold != nullptr) {
      hook_hold->valid = false;
      hook_hold->left_joint_ref = 0.0;
      hook_hold->right_joint_ref = 0.0;
    }
    if (!left_grip || !right_grip) {
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' hook rod stage is not enabled (gripper grasp positions "
        "unsealed) — keeping the pose-only grasp.",
        control.id.c_str());
      return;
    }
    const bool probe_seal_configured =
      drawer_left_tool_.gripper_hook_press_overshoot > 0.0 &&
      drawer_left_tool_.gripper_hook_press_force_floor > 0.0 &&
      drawer_right_tool_.gripper_hook_press_overshoot > 0.0 &&
      drawer_right_tool_.gripper_hook_press_force_floor > 0.0;
    const double slide_before = button_snapshot(control).position;
    double measured_left = 0.0;
    double measured_right = 0.0;
    try {
      if (probe_seal_configured) {
        bool in_place_ok = false;
        seal_drawer_hooks_by_probe(
          goal_handle, control, move_group, &measured_left, &measured_right,
          &in_place_ok, operation_executed);
        // doc §4.4 + fix#15: seal 输出的保持参考（深压 press_ref）是下游
        // support 阶段必须携带的正压保持参考 —— 收到它, support 的 hook_hold
        // override 才会持续命令 press_ref 把钩压贴位面; 若传座封 joint_ref, 积分
        // 退火失力 → 复现 cap2 右杆回弹。seal 只有两种结局 —— 接管复核通过
        //（in_place_ok=true，上面已填 measured=press_ref），或抛错整阶段失败
        //（§4.2）。故此处 valid 在 sealed 路径恒为 true；false 只可能来自下面
        // legacy unsealed 分支（其他抽屉/场景零回归，用静态 gripper_grasp_position）。
        if (hook_hold != nullptr) {
          hook_hold->left_joint_ref = measured_left;
          hook_hold->right_joint_ref = measured_right;
          hook_hold->valid = in_place_ok;
        }
      } else {
        // legacy 到达路径（unsealed grasp）：无座封参考，hook_hold 保持 invalid，
        // 下游 support 用静态 gripper_grasp_position（其他抽屉/场景零回归）。
        measured_left = execute_drawer_rod_stage_side(
          goal_handle, control, move_group, drawer_left_tool_,
          two_cylinder_fjt_client_, "two-cylinder (left)", "hook",
          drawer_left_tool_.gripper_joint,
          drawer_left_tool_.gripper_grasp_position, false, operation_executed);
        measured_right = execute_drawer_rod_stage_side(
          goal_handle, control, move_group, drawer_right_tool_,
          unlock_motor_fjt_client_, "three-cylinder (right)", "hook",
          drawer_right_tool_.gripper_joint,
          drawer_right_tool_.gripper_grasp_position, false,
          operation_executed);
      }
    } catch (...) {
      best_effort_drawer_rods_home(goal_handle, control, move_group,
        operation_executed);
      throw;
    }
    const double slide_after = button_snapshot(control).position;
    double left_contact_distance = std::numeric_limits<double>::infinity();
    double right_contact_distance = std::numeric_limits<double>::infinity();
    if (control.has_left_handle_point) {
      left_contact_distance = measure_drawer_rod_contact(
        move_group, control, "hook",
        drawer_left_tool_.gripper_contact_link,
        drawer_left_tool_.gripper_contact_point_local,
        drawer_world_rail_point(
          control, control.left_handle_point, slide_after));
    }
    if (control.has_right_handle_point) {
      right_contact_distance = measure_drawer_rod_contact(
        move_group, control, "hook",
        drawer_right_tool_.gripper_contact_link,
        drawer_right_tool_.gripper_contact_point_local,
        drawer_world_rail_point(
          control, control.right_handle_point, slide_after));
    }
    if (left_contact_distance > drawer_gripper_contact_tolerance_ ||
      right_contact_distance > drawer_gripper_contact_tolerance_)
    {
      best_effort_drawer_rods_home(
        goal_handle, control, move_group, operation_executed);
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id +
              "' hook contact is outside tolerance (left " +
              std::to_string(left_contact_distance) + " m, right " +
              std::to_string(right_contact_distance) + " m, limit " +
              std::to_string(drawer_gripper_contact_tolerance_) + " m).");
    }
    // (P4) Old "support keepalive" re-check removed: the hook stage now runs
    // BEFORE the support stage and no arm motion separates them, so support
    // contact cannot be lost here; the mirror-image hook-retain re-check lives
    // in drive_drawer_support_stage (§6.3).
    const double slide_delta = slide_after - slide_before;
    RCLCPP_INFO(
      get_logger(),
      "Drawer '%s' hook rod stage: left hook measured %.4f m, right hook "
      "measured %.4f m; free drawer slide moved %.4f m during the close "
      "(immobility threshold sealed at AGENT §7.2 cap2).",
      control.id.c_str(), measured_left, measured_right, slide_delta);
    if (std::abs(slide_delta) > 0.02) {
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "Drawer '" + control.id +
              "' hook rod close shoved the free drawer by " +
              std::to_string(slide_delta) +
              " m (> 0.02 m); the grasp depth needs re-sealing.");
    }
  }

  // 2026-09-03 AGENT §3: 抽屉解锁服务门（向插件 set_cabinet_unlock 提交真实
  // 工具链接/点 + 解锁区点 + pressed 电机实位证据）。require_unlocked 打开时
  // 对 latch 是否真释放失败即抛 NOT_READY。
  template<typename GoalHandleT>
  void set_drawer_unlock(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    bool unlock,
    bool require_unlocked)
  {
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (!unlock_client_->wait_for_service(50ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Cabinet drawer unlock service is unavailable.");
      }
    }
    auto request = std::make_shared<
      xczs_inspection_robot_interfaces::srv::SetCabinetUnlock::Request>();
    request->control_id = control.id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (unlock &&
        (!operation_lease_held_.load() || operation_lease_lost_.load() ||
        operation_lease_id_.empty()))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Drawer unlock requires an active global operation lease.");
      }
      request->operation_lease_id = operation_lease_id_;
    }
    request->robot_model = robot_model_name_;
    if (!drawer_right_tool_.has_unlock_role ||
      drawer_right_tool_.unlock_joint != control.unlock_motor_joint)
    {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Right drawer unlock role is missing or does not match the "
              "control unlock_motor_joint.");
    }
    request->right_robot_link = drawer_right_tool_.unlock_contact_link;
    // The arm pose and plugin evidence must refer to the same physical rod.
    // Using the generic finger1 business point here allowed finger3 to extend
    // somewhere else while the unlock distance still passed.
    request->right_robot_grasp_point.x =
      drawer_right_tool_.unlock_contact_point_local.x();
    request->right_robot_grasp_point.y =
      drawer_right_tool_.unlock_contact_point_local.y();
    request->right_robot_grasp_point.z =
      drawer_right_tool_.unlock_contact_point_local.z();
    request->unlock_press_point.x = control.unlock_press_point.x();
    request->unlock_press_point.y = control.unlock_press_point.y();
    request->unlock_press_point.z = control.unlock_press_point.z();
    request->unlock = unlock;
    auto future = unlock_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Drawer unlock request timed out.");
      }
    }
    const auto response = future.get();
    if (!response->success) {
      // 2026-09-03 AGENT §3: the plugin reports motor-extension evidence —
      // report it so the failure is actionable (tip not in the unlock zone vs
      // the unlock motor not extended vs both).
      std::string evidence;
      if (response->unlock_mode == "simulated_linkage") {
        // 2026-09-06 AGENT doc §4.2: finger3 is never required at the button in
        // this mode, so the "tool tip left the zone" branch does not apply.
        evidence = "; simulated_linkage: unlock motor " +
          std::string(response->pressed ? "pressed" : "not pressed");
        if (response->pressed) {
          evidence += "; handles held left=" +
            std::string(response->left_handle_held ? "yes" : "no") + " (" +
            std::to_string(response->left_hold_distance) + " m), right=" +
            std::string(response->right_handle_held ? "yes" : "no") + " (" +
            std::to_string(response->right_hold_distance) + " m)";
        }
      } else if (!response->right_tool_contact && !response->pressed) {
        evidence = "; tool tip is not inside the unlock zone and the unlock "
          "motor is not extended";
      } else if (!response->pressed) {
        evidence = "; tool tip reached the zone but the unlock motor is not "
          "extended into [retracted+floor, ceiling)";
      } else if (!response->right_tool_contact) {
        evidence = "; unlock motor extended but the tool tip left the zone";
      }
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              response->message + " (distance " +
              std::to_string(response->distance) + " m" + evidence + ")");
    }
    if (require_unlocked && !response->drawer_unlocked) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Drawer unlock was acknowledged but the rail latch stayed "
              "engaged: " + response->message);
    }
    // 2026-09-06 AGENT doc §4.2: keep the plugin's acceptance mode so callers
    // can label cap/stage/Web evidence as simulated_linkage and never report a
    // sim grant as a real finger3 button press.
    last_unlock_acceptance_mode_ = response->unlock_mode.empty() ?
      "strict" : response->unlock_mode;
    last_unlock_simulated_ = response->simulation_acceptance;
    last_unlock_acceptance_message_ = response->message;
  }

  template<typename GoalHandleT>
  void set_drawer_bimanual_grasp(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    bool attach,
    bool base_free = false)
  {
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (!bimanual_grasp_client_->wait_for_service(50ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Cabinet bimanual grasp service is unavailable.");
      }
    }
    auto request = std::make_shared<
      xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp::Request>();
    request->control_id = control.id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (attach &&
        (!operation_lease_held_.load() || operation_lease_lost_.load() ||
        operation_lease_id_.empty()))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::GRASP_FAILED,
                "Bimanual drawer grasp attach requires an active global "
                "operation lease.");
      }
      request->operation_lease_id = operation_lease_id_;
    }
    request->robot_model = robot_model_name_;
    // 2026-09-03 AGENT §5.2: once the §4.2 gripper role is sealed (gripper
    // grasp position > 0), the plugin's per-side contact gate must be fed the
    // REAL gripper rod link and its measured rod-end local point — not the tool
    // base's abstract business point — so attach is gated on the actual gripper
    // rods touching the handles.  Before sealing it falls back to the proven
    // tool-base business-point contract (both tools' approach poses place their
    // tips on the handle points, so the distance check must receive the tip
    // offsets in the contact-link frame, not the link origin).
    const bool left_real_gripper = drawer_left_tool_.has_gripper_role &&
      drawer_left_tool_.gripper_grasp_position > 0.0;
    const bool right_real_gripper = drawer_right_tool_.has_gripper_role &&
      drawer_right_tool_.gripper_grasp_position > 0.0;
    if (left_real_gripper) {
      request->left_robot_link = drawer_left_tool_.gripper_contact_link;
      request->left_robot_grasp_point.x =
        drawer_left_tool_.gripper_contact_point_local.x();
      request->left_robot_grasp_point.y =
        drawer_left_tool_.gripper_contact_point_local.y();
      request->left_robot_grasp_point.z =
        drawer_left_tool_.gripper_contact_point_local.z();
    } else {
      request->left_robot_link = drawer_left_tool_.contact_tool_link;
      request->left_robot_grasp_point.x =
        drawer_left_tool_.tool_tip_position.x();
      request->left_robot_grasp_point.y =
        drawer_left_tool_.tool_tip_position.y();
      request->left_robot_grasp_point.z =
        drawer_left_tool_.tool_tip_position.z();
    }
    if (right_real_gripper) {
      request->right_robot_link = drawer_right_tool_.gripper_contact_link;
      request->right_robot_grasp_point.x =
        drawer_right_tool_.gripper_contact_point_local.x();
      request->right_robot_grasp_point.y =
        drawer_right_tool_.gripper_contact_point_local.y();
      request->right_robot_grasp_point.z =
        drawer_right_tool_.gripper_contact_point_local.z();
    } else {
      request->right_robot_link = drawer_right_tool_.contact_tool_link;
      request->right_robot_grasp_point.x =
        drawer_right_tool_.tool_tip_position.x();
      request->right_robot_grasp_point.y =
        drawer_right_tool_.tool_tip_position.y();
      request->right_robot_grasp_point.z =
        drawer_right_tool_.tool_tip_position.z();
    }
    request->left_handle_point.x = control.left_handle_point.x();
    request->left_handle_point.y = control.left_handle_point.y();
    request->left_handle_point.z = control.left_handle_point.z();
    request->right_handle_point.x = control.right_handle_point.x();
    request->right_handle_point.y = control.right_handle_point.y();
    request->right_handle_point.z = control.right_handle_point.z();
    request->robot_base_link = grasp_brake_link_;
    request->attach = attach;
    // P3-8 grab-and-drive: the drawer pull translates the base along the
    // drawer axis, so the attach must release the chassis brake (base_free).
    request->base_free = base_free;
    auto future = bimanual_grasp_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Bimanual drawer grasp request timed out.");
      }
    }
    const auto response = future.get();
    if (!response->success) {
      // 2026-09-02: surface the per-side contact evidence so a hovering attach
      // is reported as such (the plugin now rejects it).
      std::string contact;
      if (attach) {
        contact = "; left_tool_contact=" +
          std::to_string(response->left_tool_contact) +
          ", right_tool_contact=" +
          std::to_string(response->right_tool_contact);
      }
      throw GenericOperationError(
              attach ? OperateCabinetControl::Result::GRASP_FAILED :
              OperateCabinetControl::Result::RELEASE_FAILED,
              response->message + " (left " +
              std::to_string(response->left_distance) + " m, right " +
              std::to_string(response->right_distance) + " m" + contact + ")");
    }
    if (attach && (!response->left_tool_contact ||
      !response->right_tool_contact))
    {
      // The plugin's attach gate already requires both contacts, but refuse a
      // hover pull even if the plugin ever relaxes: both tools must genuinely
      // touch their handles before the drawer is dragged.
      throw GenericOperationError(
              OperateCabinetControl::Result::GRASP_FAILED,
              "Bimanual drawer grasp attached without confirmed tool contact "
              "(left_tool_contact=" +
              std::to_string(response->left_tool_contact) +
              ", right_tool_contact=" +
              std::to_string(response->right_tool_contact) + "); refusing to "
              "pull in empty air.");
    }
  }

  // 2026-09-03 AGENT §6.3 / §13.2（用户定夺）：正式抽拉/推回 = 双臂同步
  // waypoint 拉拽。attach 带 base_free=false（插件创建底盘固定制动关节，拉拽
  // 反力落在地面，底盘全程保持 docked）；两侧工具尖端在整段拉距内保持压入
  // 手柄立板（waypoint 接触偏移与 grasp 位一致，耦合 keepalive 需要持续接触），
  // 插件弹簧-阻尼耦合让抽屉 1:1 跟随双侧尖端。双臂每段共享同一时间基准
  // （plan → 同步时长 → 带 watchdog 执行，见 execute_bimanual_segmented_
  // cartesian_path）；某段失败时按已完成 waypoint 折算实测到达位置，随异常
  // 响亮上报（现场封顶阶段 8/9 的判据）。P3-8 的 pull_drawer_by_base_
  // translation 保留在本文件下方（不再被调用），作未来兜底/恢复参考。
  template<typename GoalHandleT>
  void pull_drawer_by_arm_waypoints(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double target_position,
    bool * operation_executed,
    double already_at_tolerance = -1.0)
  {
    // 2026-09-03 AGENT §7.2 (cap7 run2 fix): 跳过判据的容差可覆盖。默认沿用
    // slider_position_tolerance_ (0.03) —— 对常规 0.30 全开/全关足够；但封顶
    // 拉距 0.03 恰好落在其内，cap7 曾整段被短路（|0.03−0.0033|=0.0267≤0.03
    // → "already at the requested position"，抽屉从未移动）。封顶流程传
    // capped_pull_already_at_tolerance_ (0.002)，保证短拉距真正执行。
    const double effective_skip_tolerance = already_at_tolerance < 0.0 ?
      slider_position_tolerance_ : already_at_tolerance;
    const double drawer_start = button_snapshot(control).position;
    const double travel = target_position - drawer_start;
    if (std::abs(travel) <= effective_skip_tolerance) {
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' is already at the requested position (%.4f m; skip "
        "tolerance %.4f m); no arm pull.",
        control.id.c_str(), drawer_start, effective_skip_tolerance);
      return;
    }
    std::vector<geometry_msgs::msg::Pose> left_waypoints;
    std::vector<geometry_msgs::msg::Pose> right_waypoints;
    // 拉拽 waypoints 必须按抽屉实测位置重算：解锁/支撑段可能把自由抽屉带开
    // 数十 mm（run 实测 0→35mm），分支入口按操作起始位置算好的链已过期，
    // 第一段就会与把手失配。尖端压入偏移与 grasp 位一致（耦合 keepalive
    // 需要持续接触）。
    calculate_drawer_bimanual_waypoints(
      control, drawer_start, target_position, left_waypoints, right_waypoints);
    RCLCPP_INFO(
      get_logger(),
      "Pulling drawer '%s' by synchronized bimanual waypoints: %.4f -> "
      "%.4f m (%zu matched waypoint pairs; chassis brake held, "
      "base_free=false).",
      control.id.c_str(), drawer_start, target_position,
      left_waypoints.size());
    std::size_t completed_waypoints = 0U;
    try {
      execute_bimanual_segmented_cartesian_path(
        left_group, right_group, goal_handle,
        left_waypoints, right_waypoints,
        door_cartesian_segment_waypoints_,
        cartesian_velocity_scale_ * 0.5,
        cartesian_acceleration_scale_ * 0.5,
        completed_waypoints, operation_executed);
    } catch (const OperationError & error) {
      // 失败也要留下"拉到哪里"的证据：完成 waypoint 数折算实测到达位置。
      // PLANNING_FAILED 与 EXECUTION_FAILED 原样上抛，仅补充距离取证。
      const double fraction = left_waypoints.empty() ? 0.0 :
        static_cast<double>(completed_waypoints) /
        static_cast<double>(left_waypoints.size());
      throw OperationError(
              error.error_code,
              std::string("Drawer bimanual arm pull stopped at an estimated "
                          "rail position of ") +
              std::to_string(drawer_start + travel * fraction) +
              " m after " + std::to_string(completed_waypoints) + "/" +
              std::to_string(left_waypoints.size()) +
              " waypoints en route to " + std::to_string(target_position) +
              " m: " + error.what());
    }
    // 全部 waypoint 完成只证明双臂到达目标位姿；抽屉是否真实到位由调用方
    // 的 wait_for_slider_detent / 位置轮询判定（耦合弹簧的 1:1 跟随在此处只
    // 保证抽屉已跟到尖端附近）。
  }

  // 2026-09-03 AGENT §7.2: 封顶阶段收尾（caps 1/3/4/5/6/7/8 共用），与正常
  // 尾段同序的安全序列：解除真实附着（若仍挂着）→ 电缸回位 → 双臂退让并
  // 收起 → 显式重新闩定轨道闩（cap≥3 的抽屉可能仍处释放态；解锁=false 在
  // 插件侧恒安全、无需会话权威）。全部完成后抛 DebugStageCapReached 把本次
  // 任务按成功交付 —— 调用后不返回。
  template<typename GoalHandleT>
  void finish_capped_drawer_stage(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    int stage,
    const std::string & evidence,
    bool attached,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    if (attached) {
      set_drawer_bimanual_grasp(goal_handle, control, false);
      result->grasp_released = true;
    }
    if (right_group) {
      best_effort_drawer_rods_home(
        goal_handle, control, *right_group, &result->operation_executed);
    }
    publish_operate_feedback(
      goal_handle,
      OperateCabinetControl::Feedback::RETREATING,
      0.94F, 0.0,
      "Retreating both arms before stowing (capped stage).");
    best_effort_bimanual_retreat_and_stow(
      left_group, right_group, control, &result->operation_executed);
    // 轨道闩显式重新闩定：cap≥3 的抽屉此刻处于释放自由态（未打开过则插件
    // 不会自动复闩），重锁让下一次封顶/全流程从干净的"关闭+闩定"出发。cap1
    // 未解锁，重复闩定恒无害。重锁失败只告警（抽屉保持自由，仍可被下一次
    // 解锁流程幂等接管）。
    try {
      set_drawer_unlock(goal_handle, control, false, false);
    } catch (const std::exception & relock_error) {
      RCLCPP_WARN(
        get_logger(),
        "Drawer '%s' capped stage %d re-lock failed; the rail latch stays "
        "released: %s",
        control.id.c_str(), stage, relock_error.what());
    }
    throw DebugStageCapReached(
      stage,
      std::string("capped at AGENT §7.2 stage ") + std::to_string(stage) +
      ": " + evidence);
  }

  // P3-8 grab-and-drive drawer pull.  The right arm's forearm self-collides
  // with the chassis beyond p≈0.15 and no dock shift / dock yaw / tool roll /
  // west-side push clears the full pull to the 0.3 m open detent (exhaustive
  // real-seed IK sweep).  So the pull does NOT fold the arms: it holds the
  // grasp config and translates the BASE along the drawer axis, and the
  // plugin's linear-drag coupling opens/closes the drawer 1:1 with the tools'
  // world projection.  The arm joint config never changes during the drive,
  // so no self-collision can develop.  Requires the attach to have been made
  // with base_free=true (the chassis brake was skipped so the base can move).
  // 2026-09-03 AGENT §6.3/§13.2: 该函数保留作兜底参考，正式路径已切换到
  // pull_drawer_by_arm_waypoints（双臂拉拽 + 支撑电缸锚点）。
  template<typename GoalHandleT>
  void pull_drawer_by_base_translation(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double target_position,
    bool * operation_executed)
  {
    const double drawer_start = button_snapshot(control).position;
    const double travel = target_position - drawer_start;
    if (std::abs(travel) <= slider_position_tolerance_) {
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' is already at the requested detent (%.4f); no base drive.",
        control.id.c_str(), drawer_start);
      return;
    }
    geometry_msgs::msg::TransformStamped reference_transform;
    try {
      reference_transform = transform_buffer_->lookupTransform(
        planning_frame_, docking_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Could not read the robot odometry before the drawer base "
              "drive: " + std::string(error.what()));
    }
    tf2::Quaternion reference_rotation;
    tf2::fromMsg(reference_transform.transform.rotation, reference_rotation);
    reference_rotation.normalize();
    const double reference_yaw = tf2::getYaw(reference_rotation);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(drawer_base_drive_timeout_);
    auto last_feedback = std::chrono::steady_clock::time_point{};
    bool base_commanded = false;
    try {
      while (std::chrono::steady_clock::now() < deadline) {
        check_cancel(goal_handle);
        const double drawer_now = button_snapshot(control).position;
        const double remaining = target_position - drawer_now;
        if (std::abs(remaining) <= slider_position_tolerance_) {
          publish_manual_base_stop();
          if (base_commanded && operation_executed) {
            *operation_executed = true;
          }
          return;
        }
        geometry_msgs::msg::TransformStamped current_transform;
        try {
          current_transform = transform_buffer_->lookupTransform(
            planning_frame_, docking_base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Could not read the robot odometry during the drawer base "
                  "drive: " + std::string(error.what()));
        }
        tf2::Quaternion current_rotation;
        tf2::fromMsg(current_transform.transform.rotation, current_rotation);
        const double current_x = current_transform.transform.translation.x;
        const double current_y = current_transform.transform.translation.y;
        if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
          !std::isfinite(current_rotation.length2()) ||
          current_rotation.length2() <= 1.0e-12)
        {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Robot odometry is invalid during the drawer base drive.");
        }
        current_rotation.normalize();
        const double current_yaw = tf2::getYaw(current_rotation);
        const int direction = remaining > 0.0 ? 1 : -1;
        const double world_speed = std::min(
          drawer_base_drive_max_speed_,
          drawer_base_drive_gain_ * std::abs(remaining));
        // Manual base twist is in the robot body frame (the router forwards
        // /xczs/manual_cmd_vel unmodified).  Transform the pure east/west
        // world velocity into the body frame and hold yaw against the yaw at
        // drive start so the tool tips stay on the handles.
        geometry_msgs::msg::Twist command;
        command.linear.x = std::cos(current_yaw) * world_speed *
          static_cast<double>(direction);
        command.linear.y = -std::sin(current_yaw) * world_speed *
          static_cast<double>(direction);
        command.angular.z = std::clamp(
          docking_angular_gain_ *
            std::atan2(
              std::sin(reference_yaw - current_yaw),
              std::cos(reference_yaw - current_yaw)),
          -docking_max_angular_speed_,
          docking_max_angular_speed_);
        manual_base_publisher_->publish(command);
        base_commanded = true;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_feedback >= 500ms) {
          last_feedback = now;
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.62F, target_position,
            "Driving the base along the drawer axis: remaining " +
            std::to_string(std::abs(remaining)) + " m.");
        }
        std::this_thread::sleep_for(50ms);
      }
    } catch (...) {
      publish_manual_base_stop();
      throw;
    }
    publish_manual_base_stop();
    throw OperationError(
            PressCabinetButton::Result::EXECUTION_FAILED,
            "The drawer base drive did not reach the requested detent within "
            "the timeout.");
  }

  void release_drawer_bimanual_grasp_noexcept(
    const ButtonSpec & control) noexcept
  {
    try {
      if (!bimanual_grasp_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(
          get_logger(),
          "Emergency bimanual grasp release service is unavailable for '%s'.",
          control.id.c_str());
        return;
      }
      constexpr int kReleaseAttempts = 2;
      for (int attempt = 1; attempt <= kReleaseAttempts; ++attempt) {
        auto request = std::make_shared<
          xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp::
          Request>();
        request->control_id = control.id;
        {
          std::lock_guard<std::mutex> lock(operation_lease_mutex_);
          request->operation_lease_id = operation_lease_id_;
        }
        request->robot_model = robot_model_name_;
        request->left_robot_link = drawer_left_tool_.contact_tool_link;
        request->right_robot_link = drawer_right_tool_.contact_tool_link;
        request->left_robot_grasp_point.x = drawer_left_tool_.tool_tip_position.x();
        request->left_robot_grasp_point.y = drawer_left_tool_.tool_tip_position.y();
        request->left_robot_grasp_point.z = drawer_left_tool_.tool_tip_position.z();
        request->right_robot_grasp_point.x = drawer_right_tool_.tool_tip_position.x();
        request->right_robot_grasp_point.y = drawer_right_tool_.tool_tip_position.y();
        request->right_robot_grasp_point.z = drawer_right_tool_.tool_tip_position.z();
        request->left_handle_point.x = control.left_handle_point.x();
        request->left_handle_point.y = control.left_handle_point.y();
        request->left_handle_point.z = control.left_handle_point.z();
        request->right_handle_point.x = control.right_handle_point.x();
        request->right_handle_point.y = control.right_handle_point.y();
        request->right_handle_point.z = control.right_handle_point.z();
        request->robot_base_link = grasp_brake_link_;
        request->attach = false;
        auto future = bimanual_grasp_client_->async_send_request(request);
        if (future.wait_for(2s) != std::future_status::ready) {
          RCLCPP_ERROR(
            get_logger(),
            "Emergency bimanual release timed out for '%s' (attempt %d/%d).",
            control.id.c_str(), attempt, kReleaseAttempts);
          continue;
        }
        const auto response = future.get();
        if (response->success) {
          return;
        }
        RCLCPP_ERROR(
          get_logger(),
          "Emergency bimanual release failed for '%s' (attempt %d/%d): %s",
          control.id.c_str(), attempt, kReleaseAttempts,
          response->message.c_str());
      }
      RCLCPP_ERROR(
        get_logger(),
        "Emergency bimanual release exhausted all attempts for '%s'.",
        control.id.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Emergency bimanual release failed: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  void wait_for_pregrasp_controls_stable(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & target_control,
    const std::vector<ControlStabilityReference> & references,
    std::chrono::steady_clock::time_point boundary_started_at)
  {
    if (references.empty()) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "No initial physical-state reference is available for '" +
              target_control.id + "'.");
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      const auto now = std::chrono::steady_clock::now();
      bool all_stable = true;
      auto oldest_sample = std::chrono::steady_clock::time_point::max();
      auto newest_sample = std::chrono::steady_clock::time_point::min();
      for (const auto & reference : references) {
        if (!reference.control) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::NOT_READY,
                  "A pre-grasp physical-state reference is unavailable for '" +
                  target_control.id + "'.");
        }
        const auto state = button_snapshot(*reference.control);
        const auto status = classify_pregrasp_stability_sample(
          state.structured_received,
          state.valid,
          state.structured_received_at > boundary_started_at,
          std::chrono::duration<double>(
            now - state.structured_received_at).count() <=
          button_state_timeout_,
          state.state_id == reference.state_id,
          state.in_motion,
          state.position,
          reference.position,
          state.velocity,
          target_tolerance_,
          stable_velocity_tolerance_);
        if (status ==
          PregraspStabilitySampleStatus::REFERENCE_CHANGED)
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::NOT_READY,
                  "Control '" + reference.control->id +
                  "' changed from its initial state or position before "
                  "grasp; the operation was stopped.");
        }
        if (status != PregraspStabilitySampleStatus::STABLE) {
          all_stable = false;
          break;
        }
        oldest_sample = std::min(
          oldest_sample, state.structured_received_at);
        newest_sample = std::max(
          newest_sample, state.structured_received_at);
      }
      if (all_stable) {
        if (stable_since == std::chrono::steady_clock::time_point{}) {
          // Start only once every stream has supplied a stable sample after
          // this boundary.  Requiring each stream's latest sample to advance
          // past this common point prevents a single cached sample from
          // satisfying the continuous-stability interval.
          stable_since = newest_sample;
        }
        if (oldest_sample >= stable_since &&
          std::chrono::duration<double>(
            oldest_sample - stable_since).count() >=
          stable_state_duration_)
        {
          return;
        }
      } else {
        stable_since = std::chrono::steady_clock::time_point{};
      }
      std::this_thread::sleep_for(50ms);
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::NOT_READY,
            "Control '" + target_control.id +
            "' and its articulated parents did not provide a continuous "
            "fresh, valid, stationary pre-grasp state.");
  }

  template<typename GoalHandleT>
  void wait_for_parent_controls_stable(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    const std::unordered_map<std::string, std::string> & expected_states)
  {
    const auto ancestors = control_ancestors(control);
    if (ancestors.empty()) {
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      const auto now = std::chrono::steady_clock::now();
      bool all_stable = true;
      auto oldest_sample = std::chrono::steady_clock::time_point::max();
      for (const auto * ancestor : ancestors) {
        const auto state = button_snapshot(*ancestor);
        const auto expected = expected_states.find(ancestor->id);
        const bool fresh = structured_control_state_is_usable(
          state.structured_received, state.valid, true,
          std::chrono::duration<double>(
            now - state.structured_received_at).count() <=
          button_state_timeout_);
        if (expected == expected_states.end()) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::INTERNAL_ERROR,
                  "Missing expected parent state for '" + ancestor->id +
                  "'.");
        }
        if (!fresh || state.state_id != expected->second || state.in_motion ||
          std::abs(state.velocity) > stable_velocity_tolerance_)
        {
          all_stable = false;
          break;
        }
        oldest_sample = std::min(
          oldest_sample, state.structured_received_at);
      }
      if (all_stable) {
        if (stable_since == std::chrono::steady_clock::time_point{}) {
          stable_since = oldest_sample;
        }
        if (std::chrono::duration<double>(
            oldest_sample - stable_since).count() >=
          stable_state_duration_)
        {
          return;
        }
      } else {
        stable_since = std::chrono::steady_clock::time_point{};
      }
      std::this_thread::sleep_for(50ms);
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::NOT_READY,
            "Parent controls for '" + control.id +
            "' did not return to a stable physical detent.");
  }

  template<typename GoalHandleT>
  void wait_for_door_release_position(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double initial_position,
    double release_position,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const double direction = release_position - initial_position;
    if (std::abs(direction) <= target_tolerance_) {
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(door_release_position_timeout_);
    const double detent_midpoint =
      0.5 * (control.min_position + control.max_position);
    const double safe_release_position = detent_midpoint +
      std::copysign(
      door_detent_hysteresis_ + door_release_position_margin_, direction);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      std::unique_lock<std::mutex> lock(control.runtime->mutex);
      const auto now = std::chrono::steady_clock::now();
      const auto & state = control.runtime->state;
      const bool fresh = structured_control_state_is_usable(
        state.structured_received, state.valid,
        state.structured_received_at > minimum_received_at,
        std::chrono::duration<double>(
          now - state.structured_received_at).count() <=
        button_state_timeout_);
      const bool crossed_release_position = direction > 0.0 ?
        state.position >= safe_release_position :
        state.position <= safe_release_position;
      if (fresh && crossed_release_position) {
        return;
      }
      control.runtime->condition.wait_for(lock, 50ms);
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::TARGET_NOT_REACHED,
            control.id + " did not physically cross its safe release "
            "position while the grasp was attached.");
  }

  template<typename GoalHandleT>
  void wait_for_knob_release_position(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    const std::string & target_state,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const double direction = target_position - initial_position;
    if (std::abs(direction) <= target_tolerance_) {
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    const double safe_release_position =
      0.5 * (initial_position + target_position) +
      std::copysign(
      door_detent_hysteresis_ + door_release_position_margin_, direction);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      std::unique_lock<std::mutex> lock(control.runtime->mutex);
      const auto now = std::chrono::steady_clock::now();
      const auto & state = control.runtime->state;
      const bool fresh = structured_control_state_is_usable(
        state.structured_received, state.valid,
        state.structured_received_at > minimum_received_at,
        std::chrono::duration<double>(
          now - state.structured_received_at).count() <=
        button_state_timeout_);
      const bool crossed_release_position = direction > 0.0 ?
        state.position >= safe_release_position :
        state.position <= safe_release_position;
      if (fresh && crossed_release_position && state.state_id == target_state) {
        return;
      }
      control.runtime->condition.wait_for(lock, 50ms);
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::TARGET_NOT_REACHED,
            control.id + " did not physically latch the requested adjacent "
            "detent while the grasp was attached.");
  }

  template<typename GoalHandleT>
  void wait_for_target_stable(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double target_position,
    const std::string & target_state,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const double settle_timeout =
      control.control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR ?
      door_settle_timeout_ : system_wait_timeout_;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(settle_timeout);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(control.runtime->mutex);
        const auto now = std::chrono::steady_clock::now();
        const auto & state = control.runtime->state;
        const bool fresh = structured_control_state_is_usable(
          state.structured_received, state.valid,
          state.structured_received_at > minimum_received_at,
          std::chrono::duration<double>(
            now - state.structured_received_at).count() <=
          button_state_timeout_);
        const bool at_position = fresh &&
          std::abs(state.position - target_position) <= target_tolerance_;
        const bool state_matches = target_state.empty() ||
          state.state_id == target_state;
        const bool stopped = !state.in_motion &&
          std::abs(state.velocity) <= stable_velocity_tolerance_;
        if (at_position && state_matches && stopped) {
          if (stable_since == std::chrono::steady_clock::time_point{}) {
            stable_since = state.structured_received_at;
          }
          if (std::chrono::duration<double>(
              state.structured_received_at - stable_since).count() >=
            stable_state_duration_)
          {
            return;
          }
        } else {
          stable_since = std::chrono::steady_clock::time_point{};
        }
        control.runtime->condition.wait_for(lock, 50ms);
      }
    }
    const auto final_state = button_snapshot(control);
    throw GenericOperationError(
            OperateCabinetControl::Result::TARGET_NOT_REACHED,
            control.id + " did not settle at the requested physical state "
            "(state='" + final_state.state_id + "', position=" +
            std::to_string(final_state.position) + ", velocity=" +
            std::to_string(final_state.velocity) + ").");
  }

  static std::uint8_t map_legacy_error_code(std::uint8_t error_code)
  {
    switch (error_code) {
      case PressCabinetButton::Result::INVALID_BUTTON:
        return OperateCabinetControl::Result::INVALID_CONTROL;
      case PressCabinetButton::Result::NOT_READY:
        return OperateCabinetControl::Result::NOT_READY;
      case PressCabinetButton::Result::NAVIGATION_FAILED:
        return OperateCabinetControl::Result::NAVIGATION_FAILED;
      case PressCabinetButton::Result::PLANNING_FAILED:
        return OperateCabinetControl::Result::PLANNING_FAILED;
      case PressCabinetButton::Result::EXECUTION_FAILED:
        return OperateCabinetControl::Result::EXECUTION_FAILED;
      case PressCabinetButton::Result::PRESS_NOT_DETECTED:
        return OperateCabinetControl::Result::TARGET_NOT_REACHED;
      case PressCabinetButton::Result::RELEASE_NOT_DETECTED:
        return OperateCabinetControl::Result::RELEASE_FAILED;
      case PressCabinetButton::Result::CANCELED:
        return OperateCabinetControl::Result::CANCELED;
      case PressCabinetButton::Result::RESOURCE_BUSY:
        return OperateCabinetControl::Result::RESOURCE_BUSY;
      case PressCabinetButton::Result::LEASE_LOST:
        return OperateCabinetControl::Result::LEASE_LOST;
      case PressCabinetButton::Result::TOOLSET_MISMATCH:
        return OperateCabinetControl::Result::TOOLSET_MISMATCH;
      case PressCabinetButton::Result::ADAPTER_NOT_VALIDATED:
        return OperateCabinetControl::Result::UNREACHABLE;
      default:
        return OperateCabinetControl::Result::INTERNAL_ERROR;
    }
  }

  bool is_operate_goal_canceling(
    const std::shared_ptr<OperateGoalHandle> & goal_handle) const noexcept
  {
    return goal_canceling_after_request(goal_handle);
  }

  bool finish_operate_goal_noexcept(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    bool request_success) noexcept
  {
    const auto finalize = [&]() {
        if (result) {
          if (request_success) {
            result->failure_reason.clear();
          } else {
            result->physical_outcome_confirmed = false;
            result->final_state_verified = false;
            result->transport_succeeded = false;
            result->recovery_succeeded = false;
            result->grasp_released = false;
            if (result->failure_reason.empty()) {
              result->failure_reason = result->message.empty() ?
                "Cabinet operation failed without a diagnostic message." :
                result->message;
            }
          }
        }
        return apply_goal_terminal_disposition(
          goal_terminal_disposition(
            request_success, is_operate_goal_canceling(goal_handle)),
          [&]() {goal_handle->succeed(result);},
          [&]() {
            result->success = false;
            result->error_code = OperateCabinetControl::Result::CANCELED;
            result->message = "Cabinet operation was canceled.";
            result->failure_reason = result->message;
            goal_handle->canceled(result);
          },
          [&]() {goal_handle->abort(result);});
      };
    try {
      if (!goal_handle || !goal_handle->is_active()) {
        RCLCPP_WARN(
          get_logger(), "Cabinet operation goal was already terminal.");
        return false;
      }
      return finalize();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finalize cabinet operation: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finalize cabinet operation.");
    }

    // A cancel request can race the first terminal-state transition.
    try {
      if (goal_handle && goal_handle->is_active()) {
        return finalize();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the cabinet operation terminal state: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the cabinet operation terminal state.");
    }
    return false;
  }

  void snapshot_planning_only_result(
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const std::shared_ptr<ButtonSpec> & control) noexcept
  {
    // This cleanup path is deliberately observation-only.  In particular it
    // must not call the ordinary recovery helper: recovery is allowed to
    // execute retreat/stow trajectories and release a physical grasp.
    if (!result || !control) {
      return;
    }
    const auto state = button_snapshot(*control);
    result->final_position = state.position;
    result->peak_position = std::abs(state.position);
    result->final_state = state.state_id;
    if (control->control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
    {
      result->estimated_force =
        state.position * control->spring_stiffness;
      result->button_triggered = state.pressed;
    }
  }

  void recover_failed_operate_goal(
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const std::shared_ptr<ButtonSpec> & control,
    const std::shared_ptr<MoveGroupInterface> & move_group,
    bool should_attempt_retreat,
    bool grasp_attached,
    const DoorArcProgress & door_arc_progress,
    double rotary_tool_roll_offset,
    bool canceled,
    bool drawer_bimanual_attached = false,
    const std::shared_ptr<MoveGroupInterface> & drawer_left_move_group =
      std::shared_ptr<MoveGroupInterface>(),
    const std::shared_ptr<OperateGoalHandle> & goal_handle =
      std::shared_ptr<OperateGoalHandle>()) noexcept
  {
    const bool physical_recovery_required = physical_recovery_is_required(
      result->operation_executed, should_attempt_retreat, grasp_attached);
    const bool motion_recovery_allowed =
      physical_recovery_required && !operation_lease_lost_.load();
    if (!physical_recovery_required) {
      RCLCPP_INFO(
        get_logger(),
        "Skipping arm/grasp recovery after '%s': the operation failed "
        "before any manipulator command, grasp, or required retreat.",
        result->diagnostic_stage.c_str());
    }
    // Bimanual drawer recovery: release the two-sided fixed constraint FIRST
    // (the drawer is still rigidly held while it is attached -- retreating
    // with the grasp on would drag the drawer), home every rod actuator
    // (AGENT §6.5), then retreat and stow BOTH arms.  The single-arm
    // retreat/stow and grasp-release paths below are for door/knob/slider/
    // button only.
    if (control && is_drawer_type(control->control_type)) {
      if (physical_recovery_required &&
        (drawer_bimanual_attached || control->requires_grasp))
      {
        result->operation_executed = true;
        release_drawer_bimanual_grasp_noexcept(*control);
      }
      // 2026-09-03 AGENT §6.5: 退让/收起前先把所有 rod 电缸（含右三缸解锁
      // 电机 r3）尽力收回 —— 否则机械臂退让会把仍伸出的杆端拖过抽屉前脸/
      // 把手。不受 motion_recovery_allowed 约束：任何阶段失败后电缸都可能
      // 仍处伸出（hook/support/unlock/attach），只要节点还能发目标就必须
      // 回位；best_effort_drawer_rods_home 内部吞错不抛，仅在此包裹兜底。
      if (goal_handle && move_group && rclcpp::ok()) {
        try {
          best_effort_drawer_rods_home(
            goal_handle, *control, *move_group, &result->operation_executed);
        } catch (const std::exception & rod_home_error) {
          RCLCPP_ERROR(
            get_logger(),
            "Drawer recovery could not home the rod actuators: %s",
            rod_home_error.what());
        }
      }
      if (motion_recovery_allowed && drawer_left_move_group &&
        move_group && rclcpp::ok())
      {
        best_effort_bimanual_retreat_and_stow(
          drawer_left_move_group, move_group, *control,
          &result->operation_executed);
      }
      // 2026-09-03 AGENT §6.5: 近关位（|实测| ≤ 0.05 m）显式重新闩定 ——
      // 抽屉从未被插件开过（解锁前失败/cap1-5）时插件不会自动复闩，必须
      // 显式 unlock=false；已打开并推回关位的由插件 auto-latch，重复闩定
      // 恒无害。中途轨位（自由滑块）跳过并告警：退让不收拢抽屉，强行拖回
      // 关位可能在导轨中途冻结，属 §6.5 明令避免的情形。
      if (goal_handle && rclcpp::ok()) {
        try {
          const double recovery_position =
            button_snapshot(*control).position;
          if (std::abs(recovery_position) <= 0.05) {
            set_drawer_unlock(goal_handle, *control, false, false);
            RCLCPP_INFO(
              get_logger(),
              "Drawer '%s' recovery re-latched the rail at the closed "
              "detent (measured %.4f m).",
              control->id.c_str(), recovery_position);
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Drawer '%s' recovery skipped re-latching: the drawer sits "
              "mid-rail at %.4f m (|p| > 0.05 m); recovery must not drag a "
              "free drawer to the closed detent.",
              control->id.c_str(), recovery_position);
          }
        } catch (const std::exception & relock_error) {
          RCLCPP_WARN(
            get_logger(),
            "Drawer '%s' recovery re-latch failed; the rail latch stays "
            "released: %s",
            control->id.c_str(), relock_error.what());
        }
      }
      if (control) {
        const auto final_state = button_snapshot(*control);
        result->final_position = final_state.position;
        result->peak_position = final_state.peak_position;
        result->final_state = final_state.state_id;
      }
      if (operation_lease_lost_.load() && !canceled) {
        result->success = false;
        result->error_code = OperateCabinetControl::Result::LEASE_LOST;
        result->message =
          "The global robot operation lease was lost; all motion was stopped.";
      } else if (canceled) {
        result->success = false;
        result->error_code = OperateCabinetControl::Result::CANCELED;
        result->message = "Cabinet operation was canceled.";
      }
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }
    const bool is_door = control && control->control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
    bool rollback_and_retreat_succeeded = false;
    if (motion_recovery_allowed && is_door && grasp_attached &&
      door_arc_progress.in_progress &&
      move_group && rclcpp::ok())
    {
      rollback_and_retreat_succeeded = best_effort_rollback_door_arc(
        *move_group, *control, door_arc_progress,
        &result->operation_executed);
    }
    if (motion_recovery_allowed && is_door && grasp_attached &&
      should_attempt_retreat && move_group && rclcpp::ok() &&
      !rollback_and_retreat_succeeded)
    {
      try {
        if (door_arc_progress.in_progress) {
          RCLCPP_WARN(
            get_logger(),
            "Door arc rollback did not complete; using the outward safety "
            "retreat fallback while the grasp remains attached.");
        }
        const auto state = button_snapshot(*control);
        const auto retreat_pose = calculate_rotary_tool_pose(
          *control, state.position, door_release_clearance_, false,
          rotary_tool_roll_offset);
        best_effort_retreat(
          *move_group, retreat_pose, &result->operation_executed);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Door pre-release safety retreat failed: %s",
          error.what());
      }
    }
    if (physical_recovery_required && control &&
      (grasp_attached || control->requires_grasp))
    {
      result->operation_executed = true;
      release_control_grasp_noexcept(control->id);
    }
    if (motion_recovery_allowed && control && should_attempt_retreat &&
      move_group && rclcpp::ok())
    {
      try {
        const auto state = button_snapshot(*control);
        const bool is_button = control->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
        const bool is_slider = is_slider_type(control->control_type);
        const auto retreat_pose = is_button ?
          calculate_operation_poses(
          *control, press_depth_).prepress_pose :
          (is_slider ?
           calculate_slider_operation_poses(
           *control, state.position, state.position).prepress_pose :
           calculate_rotary_tool_pose(
           *control, state.position,
           is_door ? door_release_clearance_ : prepress_distance_, false,
           rotary_tool_roll_offset));
        best_effort_retreat(
          *move_group, retreat_pose, &result->operation_executed);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Generic safety retreat failed: %s", error.what());
      }
    }
    if (motion_recovery_allowed && move_group && rclcpp::ok()) {
      best_effort_stow(*move_group, &result->operation_executed);
    }
    if (control) {
      const auto final_state = button_snapshot(*control);
      result->final_position = final_state.position;
      result->peak_position = final_state.peak_position;
      result->final_state = final_state.state_id;
      if (control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
      {
        result->estimated_force =
          final_state.peak_position * control->spring_stiffness;
        result->button_triggered =
          result->button_triggered ||
          final_state.peak_position + 1.0e-9 >= control->press_threshold;
      }
    }
    if (operation_lease_lost_.load() && !canceled) {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::LEASE_LOST;
      result->message =
        "The global robot operation lease was lost; all motion was stopped.";
    } else if (canceled) {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::CANCELED;
      result->message = "Cabinet operation was canceled.";
    }
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
  }

  void receive_button_joint_state(
    const ButtonSpec & button,
    const sensor_msgs::msg::JointState & message)
  {
    const auto iterator = std::find(
      message.name.begin(), message.name.end(), button.joint_name);
    if (iterator == message.name.end()) {
      return;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(message.name.begin(), iterator));
    if (index >= message.position.size() ||
      !std::isfinite(message.position[index]))
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(button.runtime->mutex);
      button.runtime->state.received = true;
      button.runtime->state.position = message.position[index];
      button.runtime->state.peak_position = update_absolute_position_peak(
        button.runtime->state.peak_position,
        button.runtime->state.position);
      button.runtime->state.received_at = std::chrono::steady_clock::now();
    }
    button.runtime->condition.notify_all();
  }

  void receive_button_pressed(
    const ButtonSpec & button,
    const std_msgs::msg::Bool & message)
  {
    {
      std::lock_guard<std::mutex> lock(button.runtime->mutex);
      if (!button.runtime->state.pressed_received ||
        button.runtime->state.pressed != message.data)
      {
        ++button.runtime->state.pressed_transition_sequence;
      }
      button.runtime->state.pressed_received = true;
      button.runtime->state.pressed = message.data;
      button.runtime->state.pressed_received_at =
        std::chrono::steady_clock::now();
    }
    button.runtime->condition.notify_all();
  }

  void receive_control_state(
    const ButtonSpec & control,
    const xczs_inspection_robot_interfaces::msg::CabinetControlState & message)
  {
    const auto update = classify_structured_control_state(
      message.control_id == control.id, message.valid, message.position,
      message.velocity, message.effort);
    if (update == StructuredControlStateUpdate::IGNORE) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(control.runtime->mutex);
      auto & state = control.runtime->state;
      const auto received_at = std::chrono::steady_clock::now();
      state.structured_received = true;
      state.structured_received_at = received_at;
      if (update == StructuredControlStateUpdate::INVALIDATE) {
        state.valid = false;
        control.runtime->condition.notify_all();
        return;
      }
      state.received = true;
      state.valid = true;
      state.position = message.position;
      state.velocity = message.velocity;
      state.effort = message.effort;
      state.peak_position = update_absolute_position_peak(
        state.peak_position, state.position);
      state.in_motion = message.in_motion;
      state.state_id = message.state_id;
      state.received_at = received_at;
      state.pressed_received = true;
      state.pressed = message.activated;
      state.pressed_received_at = state.received_at;
      state.pressed_transition_sequence = message.transition_sequence;
    }
    control.runtime->condition.notify_all();
  }

  ButtonSnapshot button_snapshot(const ButtonSpec & button) const
  {
    std::lock_guard<std::mutex> lock(button.runtime->mutex);
    return button.runtime->state;
  }

  void reset_peak_position(const ButtonSpec & button)
  {
    std::lock_guard<std::mutex> lock(button.runtime->mutex);
    button.runtime->state.peak_position = update_absolute_position_peak(
      0.0, button.runtime->state.position);
  }

  template<typename GoalHandleT>
  void wait_for_fresh_button_state(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button.runtime->mutex);
        const auto now = std::chrono::steady_clock::now();
        const bool joint_state_fresh = button.runtime->state.received &&
          button.runtime->state.received_at > minimum_received_at &&
          std::chrono::duration<double>(
          now - button.runtime->state.received_at).count() <=
          button_state_timeout_;
        const bool pressed_state_fresh =
          button.runtime->state.pressed_received &&
          button.runtime->state.pressed_received_at > minimum_received_at &&
          std::chrono::duration<double>(
          now - button.runtime->state.pressed_received_at).count() <=
          button_state_timeout_;
        const bool structured_state_fresh =
          structured_control_state_is_usable(
          button.runtime->state.structured_received,
          button.runtime->state.valid,
          button.runtime->state.structured_received_at > minimum_received_at,
          std::chrono::duration<double>(
            now - button.runtime->state.structured_received_at).count() <=
          button_state_timeout_);
        if (joint_state_fresh && pressed_state_fresh &&
          structured_state_fresh)
        {
          if (button.control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON &&
            button.runtime->state.pressed)
          {
            throw OperationError(
                    PressCabinetButton::Result::NOT_READY,
                    button.id + " is already pressed; release it before " +
                    "starting an operation.");
          }
          return;
        }
        button.runtime->condition.wait_for(lock, 100ms);
      }
    }
    throw OperationError(
            PressCabinetButton::Result::NOT_READY,
            "No fresh state was received from " + button.id +
            ". Check the cabinet model plugin and joint-state topic.");
  }

  template<typename GoalHandleT>
  bool wait_for_pressed_state(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    bool expected_pressed,
    double timeout_seconds,
    std::uint64_t previous_transition_sequence)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button.runtime->mutex);
        if (button.runtime->state.pressed_received &&
          button.runtime->state.pressed == expected_pressed &&
          button.runtime->state.pressed_transition_sequence >
          previous_transition_sequence)
        {
          return true;
        }
        button.runtime->condition.wait_for(lock, 50ms);
      }
    }
    return false;
  }

  /**
   * Wait until a slider panel has reached its target detent.
   *
   * The plugin reports the nearest-detent state, so the state-id comparison is
   * the primary signal (robust to the panel settling short of the exact detent
   * under friction).  A small position bound around the target detent backs it
   * up so a half-latched panel that only crossed the midpoint cannot pass.
   */
  template<typename GoalHandleT>
  bool wait_for_slider_detent(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    const std::string & target_state,
    double target_position,
    double timeout_seconds,
    double position_tolerance = -1.0,
    double * measured_position_out = nullptr)
  {
    // 2026-09-03 AGENT §7.2 (cap7 run2 fix): 默认沿用滑块大容差
    // (slider_position_tolerance_，状态翻转是主信号，位置只作兜底)；封顶
    // 拉距取证必须传紧容差（debug_capped_pull_tolerance_）—— 空状态名退化
    // 为纯位置判据时，0.03 的大容差会让 0.03 拉距在抽屉从未移动时也通过
    // （run2 假 PASS）。measured_position_out 把通过/超时时刻的实测位置回传，
    // 供调用方写进证据或失败消息。
    const double effective_tolerance = position_tolerance < 0.0 ?
      slider_position_tolerance_ : position_tolerance;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button.runtime->mutex);
        if (button.runtime->state.structured_received &&
          (button.runtime->state.state_id == target_state ||
           std::abs(button.runtime->state.position - target_position) <=
           effective_tolerance))
        {
          if (measured_position_out != nullptr) {
            *measured_position_out = button.runtime->state.position;
          }
          return true;
        }
        button.runtime->condition.wait_for(lock, 50ms);
      }
    }
    if (measured_position_out != nullptr) {
      *measured_position_out = button_snapshot(button).position;
    }
    return false;
  }

  /**
   * Drive the slider ready (prepress) -> contact -> press chain, retrying the
   * ready planning on a fresh OMPL IK branch when the press cannot be planned.
   *
   * A 7-DOF arm has several IK families for the same prepress tool pose, and
   * the randomized ready planner lands on one each run.  Whether the following
   * 0->target Cartesian push is plannable at the configured fraction depends
   * on that branch (boot9: a bad branch gave 14%; the B3a button probe: a good
   * branch gave >=95% at the SAME dock, same poses).  PLANNING_FAILED is
   * thrown before any physical motion, so re-planning the ready on a fresh
   * branch can never double-push.  Each attempt re-plans the ready pose from
   * the current state (the arm sits at contact after a failed press), which
   * re-samples the prepress IK and therefore the contact config and the press
   * path.
   */
  template<typename GoalHandleT>
  void execute_slider_ready_approach_press(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const OperationPoses & slider_poses,
    bool closing,
    double target_position,
    const ButtonSpec & control,
    bool * operation_executed)
  {
    constexpr int kSliderReadyRetryAttempts = 3;
    for (int attempt = 0; attempt < kSliderReadyRetryAttempts; ++attempt) {
      try {
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          attempt == 0 ?
          "Planning the probe to the slider ready pose." :
          "The slider press was not plannable on the previous IK branch; "
          "re-planning the ready pose on a fresh branch.");
        plan_and_execute_pose(
          move_group, goal_handle, slider_poses.prepress_pose,
          contact_tool_link_, operation_executed, &control);
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.47F, target_position,
          "Approaching the slider panel along its travel axis.");
        execute_cartesian_path(
          move_group, goal_handle, {slider_poses.contact_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, operation_executed);
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          closing ? 0.58F : 0.65F, target_position,
          closing ?
          "Pushing the open panel past its detent to trip the push-push "
          "release." :
          "Pushing the slider panel to the target detent.");
        execute_cartesian_path(
          move_group, goal_handle, {slider_poses.pressed_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          button_press_minimum_cartesian_fraction_, operation_executed);
        return;
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          attempt + 1 >= kSliderReadyRetryAttempts)
        {
          throw;
        }
        RCLCPP_WARN(
          get_logger(),
          "Slider ready/approach/press branch %d was not fully plannable "
          "(%s); re-planning the ready pose on a fresh IK branch.",
          attempt, error.what());
      }
    }
  }

  /**
   * Converge a slider panel to its target detent by re-pushing from the
   * PHYSICAL panel position, mirroring the button type's track_button_force.
   *
   * After a successful Cartesian press the panel may still sit short of the
   * detent (95% of the push leaves the tool at face+0.285 for a 0.3 target).
   * Each iteration measures the panel position, computes the remaining
   * deficit, and re-pushes from the CURRENT face
   * (calculate_slider_operation_poses) to the target.  The re-push is
   * deliberately lenient (5% minimum) so a short convergence push that cannot
   * be fully planned does not fail the operation -- the final detent
   * verification decides.  The compensation is clamped by
   * button_force_tracking_limit so it never reaches the push-push release
   * position.
   */
  template<typename GoalHandleT>
  void track_slider_force(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    double target_position,
    bool * operation_executed = nullptr)
  {
    double commanded_position = clamp_button_press_depth(
      target_position, button.max_position);
    const double maximum_position = button_force_tracking_limit(
      target_position, force_tracking_max_compensation_, button.max_position);
    for (int attempt = 0; attempt < force_tracking_attempts_; ++attempt) {
      interruptible_hold(goal_handle, force_tracking_settle_seconds_);
      const auto measured = button_snapshot(button);
      const double deficit = target_position - measured.position;
      if (deficit <= force_tracking_tolerance_) {
        return;
      }
      const double next_position = clamp_button_press_depth(
        std::min(
          maximum_position,
          commanded_position + deficit + force_tracking_tolerance_),
        button.max_position);
      if (next_position <= commanded_position + 1.0e-6) {
        return;
      }
      commanded_position = next_position;
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::MANIPULATING,
        0.70F, target_position,
        "Correcting the physical slider travel to reach the requested detent.");
      const auto corrected_poses = calculate_slider_operation_poses(
        button, measured.position, commanded_position);
      try {
        execute_cartesian_path(
          move_group, goal_handle, {corrected_poses.pressed_pose},
          cartesian_velocity_scale_ * 0.35,
          cartesian_acceleration_scale_ * 0.35,
          0.05, operation_executed);
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED) {
          throw;
        }
        RCLCPP_WARN(
          get_logger(),
          "Slider convergence push (%s) could not be planned; relying on the "
          "final detent verification.", error.what());
      }
    }
    interruptible_hold(goal_handle, force_tracking_settle_seconds_);
  }

  template<typename GoalHandleT>
  void track_button_force(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    double target_travel,
    bool * operation_executed = nullptr)
  {
    double commanded_depth = clamp_button_press_depth(
      target_travel, button.max_position);
    const double maximum_depth = button_force_tracking_limit(
      target_travel, force_tracking_max_compensation_, button.max_position);
    for (int attempt = 0; attempt < force_tracking_attempts_; ++attempt) {
      interruptible_hold(goal_handle, force_tracking_settle_seconds_);
      const auto measured = button_snapshot(button);
      const double deficit = target_travel - measured.position;
      if (deficit <= force_tracking_tolerance_) {
        return;
      }

      const double next_depth = clamp_button_press_depth(
        std::min(
          maximum_depth,
          commanded_depth + deficit + force_tracking_tolerance_),
        button.max_position);
      if (next_depth <= commanded_depth + 1.0e-6) {
        return;
      }
      commanded_depth = next_depth;
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::MANIPULATING,
        0.70F, target_travel,
        "Correcting the physical button travel to track the requested force.");
      const auto corrected_poses = calculate_operation_poses(
        button, commanded_depth);
      execute_cartesian_path(
        move_group, goal_handle, {corrected_poses.pressed_pose},
        cartesian_velocity_scale_ * 0.35,
        cartesian_acceleration_scale_ * 0.35,
        button_press_minimum_cartesian_fraction_, operation_executed);
    }
    interruptible_hold(goal_handle, force_tracking_settle_seconds_);
  }

  OperationPoses make_face_press_poses(
    const ButtonSpec & button,
    const tf2::Vector3 & button_face,
    double press_depth)
  {
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const tf2::Quaternion cabinet_rotation = cabinet_transform.getRotation();
    const auto geometry = resolve_control_geometry(button);
    tf2::Vector3 outward = tf2::quatRotate(
      cabinet_rotation, geometry.approach_normal);
    outward.normalize();
    const tf2::Vector3 inward = -outward;

    // At zero roll keep tool +X vertical and tool -Z aligned with the press
    // direction.  A calibrated local-Z roll may then clear the passive sibling
    // tool without changing either the working direction or business point.
    tf2::Vector3 tool_z = outward;
    tf2::Vector3 tool_x(0.0, 0.0, 1.0);
    tool_x -= tool_z * tool_z.dot(tool_x);
    if (tool_x.length2() < 1.0e-6) {
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Cabinet panel normal is parallel to the world vertical axis.");
    }
    tool_x.normalize();
    tf2::Vector3 tool_y = tool_z.cross(tool_x);
    tool_y.normalize();
    const tf2::Matrix3x3 tool_basis(
      tool_x.x(), tool_y.x(), tool_z.x(),
      tool_x.y(), tool_y.y(), tool_z.y(),
      tool_x.z(), tool_y.z(), tool_z.z());
    tf2::Quaternion tool_rotation;
    tool_basis.getRotation(tool_rotation);
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(
      tf2::Vector3(0.0, 0.0, 1.0), button.tool_roll_offset);
    tool_roll.normalize();
    tool_rotation *= tool_roll;
    tool_rotation.normalize();
    const auto make_tool_pose =
      [&](double tip_offset) -> geometry_msgs::msg::Pose {
        const tf2::Vector3 desired_tip_position =
          button_face + inward * tip_offset;
        const tf2::Vector3 link_position = desired_tip_position -
          tf2::quatRotate(tool_rotation, tool_tip_position_);
        geometry_msgs::msg::Pose pose;
        pose.position.x = link_position.x();
        pose.position.y = link_position.y();
        pose.position.z = link_position.z();
        pose.orientation = to_message(tool_rotation);
        return pose;
      };

    OperationPoses poses;
    poses.prepress_pose = make_tool_pose(-prepress_distance_);
    poses.contact_pose = make_tool_pose(-contact_clearance_);
    poses.pressed_pose = make_tool_pose(press_depth);

    return poses;
  }

  OperationPoses calculate_operation_poses(
    const ButtonSpec & button,
    double press_depth)
  {
    const double bounded_press_depth = clamp_button_press_depth(
      press_depth, button.max_position);
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(button);
    const tf2::Vector3 button_face =
      cabinet_transform * geometry.grasp_zero;
    return make_face_press_poses(button, button_face, bounded_press_depth);
  }

  OperationPoses calculate_slider_operation_poses(
    const ButtonSpec & button,
    double current_position,
    double target_position)
  {
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(button);
    // The panel's press face slides with the joint, so the contact adapts to
    // the CURRENT slide position.  A signed press then lets the same front
    // approach both open the panel (target > current: the tip pushes into the
    // panel) and close it again (target < current: the tip keeps contact and
    // drags the panel back to the closed detent).
    const tf2::Vector3 face_current =
      cabinet_transform * geometry.grasp_zero +
      geometry.axis * current_position;
    const double press_depth = target_position - current_position;
    return make_face_press_poses(button, face_current, press_depth);
  }

  // Configure a MoveIt group for Cartesian tool motion.  The end-effector link
  // is normally the active tool's contact link (contact_tool_link_); bimanual
  // drawer flow configures the LEFT arm group with the LEFT tool's contact
  // link, so the caller can override the EEF per side.  The default keeps the
  // single-tool call sites unchanged.
  void configure_move_group(
    MoveGroupInterface & move_group,
    const std::string & end_effector_link = std::string())
  {
    const auto robot_model = move_group.getRobotModel();
    std::vector<std::string> profile_move_groups;
    profile_move_groups.reserve(tool_profiles_.size() + 1U);
    profile_move_groups.push_back(move_group_name_);
    for (const auto & entry : tool_profiles_) {
      profile_move_groups.push_back(entry.second.move_group);
    }
    const auto kinematics_error = tool_profile_kinematics_validation_error(
      robot_model, std::move(profile_move_groups));
    if (kinematics_error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              *kinematics_error);
    }
    const std::string & configured_end_effector_link =
      end_effector_link.empty() ? contact_tool_link_ : end_effector_link;
    if (!robot_model->getLinkModel(configured_end_effector_link)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The robot adapter tool profile for MoveIt JointModelGroup '" +
              move_group_name_ + "' references missing contact tool link '" +
              configured_end_effector_link + "'.");
    }
    const auto named_targets = move_group.getNamedTargets();
    if (std::find(
        named_targets.begin(), named_targets.end(),
        transport_named_target_) == named_targets.end())
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The robot adapter transport target '" +
              transport_named_target_ + "' does not exist in the SRDF.");
    }
    if (!move_group.setEndEffectorLink(configured_end_effector_link)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt cannot use configured contact tool link '" +
              configured_end_effector_link + "'.");
    }
    move_group.setPoseReferenceFrame(planning_frame_);
    move_group.setPlanningTime(planning_time_);
    move_group.setNumPlanningAttempts(planning_attempts_);
    move_group.setMaxVelocityScalingFactor(planning_velocity_scale_);
    move_group.setMaxAccelerationScalingFactor(planning_acceleration_scale_);
    move_group.setGoalPositionTolerance(goal_position_tolerance_);
    move_group.setGoalOrientationTolerance(goal_orientation_tolerance_);
    move_group.setGoalJointTolerance(goal_joint_tolerance_);
    move_group.allowReplanning(allow_replanning_);
  }

  moveit::core::RobotStatePtr synchronized_current_robot_state(
    MoveGroupInterface & move_group)
  {
    // -----------------------------------------------------------------------
    // AGENT §8.4 fix#10 (v20 取证根因, 2026-09-05): 真实关节状态优先叠盖。
    //
    // v20 决定性矛盾: verify 报 R 臂到达通过(≤5.3 mm)时, R 钩杆端在物理里仍
    // 在轨道外 1.2 m —— 证明本函数的信源是 move_group 客户端 belief: 其
    // current-state monitor 订阅根命名空间 joint_states(该话题不存在), 真实
    // 100 Hz 状态只在 /xczs/joint_states(fix#3 订阅)。client planning-scene
    // 自启动起从未更新, 任何 getCurrentState 都是"幻影": verify/measure 的
    // FK、plan 起点全读垃圾, 在 plan 目标构型附近立即"通过"(≤4.2-6.9 mm =
    // 该构型相对位姿目标的 IK 残差)却与物理脱节。fix#3 comment (2928-2938)
    // 已诊断同一 belief-staleness(rod measured 也读垃圾)。
    //
    // 修复: 每次调用把新鲜(≤1.0 s)real 缓存叠盖到 getCurrentState 骨架上 ——
    // 缓存覆盖全部真实关节(100 Hz 23 关节), 未缓存变量保留 scene 值保完整;
    // 缓存缺失/超龄才整段回退旧行为(栈外环境/断流兜底, 旧路径本身即 v20
    // 幻影路径, 回退时打节流 WARN 留证)。下方 TF 根关节覆盖逻辑不变。
    auto state = move_group.getCurrentState(system_wait_timeout_);
    if (!state) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt did not receive the current robot state.");
    }
    const auto robot_model = move_group.getRobotModel();
    if (robot_model != nullptr) {
      // fix#3 同款新鲜度门限（须与 cached_real_joint_position 中同值同语义:
      // 以接收时刻 steady clock 判定, 不用消息 stamp 减本地 ros 时间）。
      constexpr double kRealJointStateMaxAgeSeconds = 1.0;
      std::vector<std::pair<std::string, double>> fresh_samples;
      {
        std::lock_guard<std::mutex> lock(real_joint_states_mutex_);
        const double age_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() -
          real_joint_states_receipt_).count();
        if (!real_joint_positions_.empty() && age_seconds >= 0.0 &&
            age_seconds <= kRealJointStateMaxAgeSeconds)
        {
          fresh_samples.assign(
            real_joint_positions_.begin(), real_joint_positions_.end());
        }
      }
      if (fresh_samples.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "REAL joint-state cache unavailable or stale; falling back to the "
          "MoveIt client scene (phantom-belief risk, fix#10).");
      } else {
        const auto real_state =
          std::make_shared<moveit::core::RobotState>(*state);
        // 模型变量集成员判定：缓存键是 joint 名（单变量关节 joint 名 == 变量
        // 名），floating/planar 根关节等多变量关节不在 /xczs/joint_states 里，
        // 不会误叠盖；未知名经 setVariablePosition 会越界索引，先查变量表。
        const auto & model_variables = robot_model->getVariableNames();
        for (const auto & sample : fresh_samples) {
          if (std::find(
                model_variables.begin(), model_variables.end(),
                sample.first) != model_variables.end())
          {
            real_state->setVariablePosition(sample.first, sample.second);
          }
        }
        real_state->update();
        state = real_state;
      }
    }
    const auto * root_joint =
      robot_model == nullptr ? nullptr : robot_model->getRootJoint();
    if (root_joint == nullptr || root_joint->getVariableCount() == 0U) {
      state->update();
      return state;
    }

    try {
      const auto transform = transform_buffer_->lookupTransform(
        robot_model->getModelFrame(), robot_model->getRootLinkName(),
        tf2::TimePointZero,
        tf2::durationFromSec(system_wait_timeout_));
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      Eigen::Quaterniond quaternion(
        rotation.w, rotation.x, rotation.y, rotation.z);
      quaternion.normalize();
      Eigen::Isometry3d root_transform = Eigen::Isometry3d::Identity();
      root_transform.linear() = quaternion.toRotationMatrix();
      root_transform.translation() = Eigen::Vector3d(
        translation.x, translation.y, translation.z);
      state->setJointPositions(root_joint, root_transform);
      state->update();
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt mobile-base transform '" +
              robot_model->getModelFrame() + "' -> '" +
              robot_model->getRootLinkName() +
              "' is unavailable: " + error.what());
    }
    return state;
  }

  static void update_validation_diagnostic(
    OperateCabinetControl::Result & result,
    const std::string & stage,
    double path_fraction,
    double required_fraction,
    std::int32_t moveit_error_code)
  {
    result.diagnostic_stage = stage;
    result.path_fraction = std::isfinite(path_fraction) ?
      std::max(0.0, std::min(1.0, path_fraction)) : 0.0;
    result.required_fraction = std::isfinite(required_fraction) ?
      std::max(0.0, std::min(1.0, required_fraction)) : 0.0;
    result.moveit_error_code = moveit_error_code;
  }

  static std::string planning_validation_failure_message(
    const OperateCabinetControl::Result & result,
    const std::string & detail)
  {
    std::ostringstream message;
    message << "实时 MoveIt 安全规划验证失败 [" << result.diagnostic_stage
            << "]：路径完成 " << result.path_fraction * 100.0
            << "% ，要求 " << result.required_fraction * 100.0 << "%";
    if (result.moveit_error_code != 0) {
      const moveit::core::MoveItErrorCode error(result.moveit_error_code);
      message << "，MoveIt=" << moveit::core::error_code_to_string(error)
              << "(" << result.moveit_error_code << ")";
    }
    if (!detail.empty()) {
      message << "；" << detail;
    }
    if (!result.policy_reason.empty()) {
      message << "；适配层历史限制：" << result.policy_reason;
    }
    return message.str();
  }

  moveit::core::RobotState validation_trajectory_end_state(
    MoveGroupInterface & move_group,
    const moveit::core::RobotState & start_state,
    const moveit_msgs::msg::RobotTrajectory & trajectory_message) const
  {
    robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), move_group_name_);
    trajectory.setRobotTrajectoryMsg(start_state, trajectory_message);
    if (trajectory.getWayPointCount() == 0U) {
      // MoveIt may represent an already-satisfied target as an empty plan.
      return start_state;
    }
    return trajectory.getLastWayPoint();
  }

  bool set_pose_target_with_calibrated_ik_seed(
    MoveGroupInterface & move_group,
    const moveit::core::RobotState & start_state,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    const ButtonSpec * control,
    const std::vector<double> * seed_positions_override = nullptr)
  {
    if (control == nullptr || control->ready_joint_seed_names.empty()) {
      return move_group.setPoseTarget(target, tool_link);
    }

    const auto robot_model = move_group.getRobotModel();
    const auto * joint_model_group =
      robot_model == nullptr ? nullptr :
      robot_model->getJointModelGroup(move_group_name_);
    if (joint_model_group == nullptr) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "MoveIt group '" + move_group_name_ +
              "' is unavailable while applying the calibrated IK seed.");
    }
    const auto & group_variable_names =
      joint_model_group->getVariableNames();
    std::unordered_set<std::string> configured_names(
      control->ready_joint_seed_names.begin(),
      control->ready_joint_seed_names.end());
    if (configured_names.size() != group_variable_names.size() ||
      std::any_of(
        group_variable_names.begin(), group_variable_names.end(),
        [&configured_names](const std::string & name) {
          return configured_names.count(name) == 0U;
        }))
    {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Control '" + control->id +
              "' ready_joint_seed must name every variable of MoveIt group '" +
              move_group_name_ + "' exactly once.");
    }
    const auto & seed_positions = seed_positions_override != nullptr ?
      *seed_positions_override : control->ready_joint_seed_positions;
    if (seed_positions.size() != control->ready_joint_seed_names.size()) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Control '" + control->id +
              "' runtime branch seed must name every variable of MoveIt "
              "group '" + move_group_name_ + "' exactly once.");
    }

    moveit::core::RobotState seeded_goal(start_state);
    seeded_goal.setVariablePositions(
      control->ready_joint_seed_names,
      seed_positions);
    seeded_goal.update();
    if (!seeded_goal.satisfiesBounds(joint_model_group)) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Control '" + control->id +
              "' ready_joint_seed violates the configured joint limits.");
    }
    if (!seeded_goal.setFromIK(
        joint_model_group, target, tool_link,
        std::min(1.0, planning_time_)))
    {
      return false;
    }
    seeded_goal.update();
    return seeded_goal.satisfiesBounds(joint_model_group) &&
           move_group.setJointValueTarget(seeded_goal);
  }

  // Choose the IK branch for a rotary control's ready/pregrasp plans so the
  // whole Cartesian chain (50 mm approach + the manipulation arc) is feasible
  // from the REAL dock TF.  A 7-DOF arm has several IK families for one tool
  // pose; the calibrated seed is only a starting point and can fail the arc
  // (r_arm_0<->r_arm_2 self-collision near a detent, or a wrist joint pinned
  // against its limit).  Candidate branches are the configured seed and its
  // mirror (the first group joint negated).  Each candidate is dry-run with
  // computeCartesianPath -- the same path the operator will execute -- and the
  // first candidate that clears the full chain is returned.  This is a branch
  // PICKER, not a gate: if nothing clears, the configured seed is returned and
  // the real execution reports the failure honestly.
  std::vector<double> select_rotary_branch_seed(
    MoveGroupInterface & move_group,
    const ButtonSpec & control,
    const geometry_msgs::msg::Pose & pregrasp_pose,
    const geometry_msgs::msg::Pose & grasp_pose,
    const std::vector<geometry_msgs::msg::Pose> & arc_waypoints,
    const geometry_msgs::msg::Pose & retreat_pose,
    const std::string & tool_link)
  {
    if (control.ready_joint_seed_names.empty() || arc_waypoints.empty()) {
      return {};
    }
    const auto robot_model = move_group.getRobotModel();
    const auto * joint_model_group =
      robot_model == nullptr ? nullptr :
      robot_model->getJointModelGroup(move_group_name_);
    if (joint_model_group == nullptr) {
      return {};
    }
    const auto & group_variable_names = joint_model_group->getVariableNames();
    if (group_variable_names.empty() ||
      control.ready_joint_seed_names[0] != group_variable_names.front())
    {
      return {};
    }

    // A 7-DOF arm has a one-parameter family of IK branches for the same
    // pregrasp pose.  The configured seed is only one sample; different
    // shoulder (r_arm_0) seeds converge to different branches that trade off
    // clean approach/arc against a retreat with room to clear the blade.
    // Sample the branch space broadly so the operation is robust to the few
    // millimetres of docking drift between branch validation and execution.
    std::vector<std::vector<double>> candidates;
    candidates.push_back(control.ready_joint_seed_positions);
    for (const double shoulder :
      { -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5 })
    {
      auto variant = control.ready_joint_seed_positions;
      variant[0] = shoulder;
      candidates.push_back(std::move(variant));
    }

    constexpr double kRequired = 0.99;
    double best_margin = -1.0;
    std::vector<double> best_margin_seed;
    double best_fraction = -1.0;
    std::vector<double> best_fraction_seed;
    for (const auto & seed : candidates) {
      try {
        const auto current_state =
          synchronized_current_robot_state(move_group);
        moveit::core::RobotState branch_state(*current_state);
        branch_state.setVariablePositions(
          control.ready_joint_seed_names, seed);
        if (!branch_state.setFromIK(
            joint_model_group, pregrasp_pose, tool_link,
            std::min(1.0, planning_time_)))
        {
          continue;
        }
        branch_state.update();
        if (!branch_state.satisfiesBounds(joint_model_group)) {
          continue;
        }

        moveit_msgs::msg::RobotTrajectory approach;
        move_group.setStartState(branch_state);
        const double f_approach = move_group.computeCartesianPath(
          {grasp_pose}, 0.002, cartesian_jump_threshold_, approach, true);
        if (f_approach < kRequired ||
          approach.joint_trajectory.points.empty())
        {
          continue;
        }

        robot_trajectory::RobotTrajectory approach_trajectory(
          move_group.getRobotModel(), move_group_name_);
        approach_trajectory.setRobotTrajectoryMsg(branch_state, approach);
        moveit::core::RobotState arc_start =
          approach_trajectory.getLastWayPoint();
        moveit_msgs::msg::RobotTrajectory arc;
        move_group.setStartState(arc_start);
        const double f_arc = move_group.computeCartesianPath(
          arc_waypoints, 0.002, cartesian_jump_threshold_, arc, true);
        if (f_arc < kRequired || arc.joint_trajectory.points.empty()) {
          continue;
        }

        // The post-release retreat is part of the same physical chain: the
        // arm must be able to pull the tool straight out of the knob at the
        // arc-end angle.  A branch can clear approach + arc and still pin a
        // joint against its limit during the retreat, so validate it too.
        robot_trajectory::RobotTrajectory arc_trajectory(
          move_group.getRobotModel(), move_group_name_);
        arc_trajectory.setRobotTrajectoryMsg(arc_start, arc);
        moveit::core::RobotState retreat_start =
          arc_trajectory.getLastWayPoint();
        moveit_msgs::msg::RobotTrajectory retreat;
        move_group.setStartState(retreat_start);
        const double f_retreat = move_group.computeCartesianPath(
          {retreat_pose}, 0.002, cartesian_jump_threshold_, retreat, true);

        const double fraction = std::min(
          {f_approach, f_arc, f_retreat});
        if (fraction > best_fraction) {
          best_fraction = fraction;
          best_fraction_seed = seed;
        }
        if (f_retreat < kRequired) {
          continue;
        }

        // Among branches that clear the whole chain, prefer the one whose
        // arc-end state sits farthest from every joint limit: it keeps the
        // retreat feasible against the docking drift that can appear between
        // this dry-run and the physical arc execution.
        const double margin = joint_limit_margin(
          retreat_start, joint_model_group);
        if (margin > best_margin) {
          best_margin = margin;
          best_margin_seed = seed;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Rotary branch validation for '%s' failed: %s",
          control.id.c_str(), error.what());
      }
    }
    if (!best_margin_seed.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Rotary branch selected for '%s': approach+arc+retreat clear "
        "(best arc-end joint-limit margin=%.3f).",
        control.id.c_str(), best_margin);
      return best_margin_seed;
    }
    RCLCPP_WARN(
      get_logger(),
      "No rotary branch cleared approach+arc+retreat for '%s' "
      "(best min fraction=%.3f); using the best candidate. The execution "
      "reports any failure honestly.",
      control.id.c_str(), best_fraction > 0.0 ? best_fraction : 0.0);
    return best_fraction_seed.empty() ?
      control.ready_joint_seed_positions : best_fraction_seed;
  }

  double joint_limit_margin(
    const moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * joint_model_group)
  {
    double min_fraction = 1.0;
    for (const auto * joint : joint_model_group->getActiveJointModels()) {
      for (const auto & variable : joint->getVariableNames()) {
        const auto & bounds = joint->getVariableBounds(variable);
        if (!bounds.position_bounded_ || bounds.min_position_ >= bounds.max_position_) {
          continue;
        }
        const double low = bounds.min_position_;
        const double high = bounds.max_position_;
        const double q = state.getVariablePosition(variable);
        const double fraction = std::min(
          (q - low) / (high - low), (high - q) / (high - low));
        min_fraction = std::min(min_fraction, fraction);
      }
    }
    return min_fraction;
  }

  // P3-8: 双手抽拉抽屉的 ready 位姿分支选择器。七轴臂对同一个 ready 工具位姿
  // 存在多组 IK 分支；run7 实测 OMPL 对右臂 ready 随机落到 r5=+2.96（腕部贴近
  // +π 限位），解锁后下探 5cm 到支撑位时 IK 在 84.4% 处耗尽（r5 无法再负转），
  // 支撑段规划失败。这里与 select_rotary_branch_seed 同构：以当前状态的 IK 分支
  // （"自然分支"，接近 home、无甩动）为基准，扫描肩部/腕部种子变体，逐个用
  // computeCartesianPath 干跑整条后续 Cartesian 链条（右臂：解锁→支撑→预抓→
  // 抓取→拽拉；左臂：支撑→预抓→抓取→拽拉），选一条整链 ≥99% 且相对自然分支
  // 甩动最小的分支。选不出则返回空，ready 走普通位姿目标（如实报告失败）。
  template<typename GoalHandleT>
  std::vector<double> select_drawer_branch_seed(
    MoveGroupInterface & move_group,
    const std::string & group_name,
    const std::string & tool_link,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & ready_pose,
    const std::vector<std::vector<geometry_msgs::msg::Pose>> & chain,
    const std::string & description,
    std::size_t reseat_rebase_segment =
      std::numeric_limits<std::size_t>::max())
  {
    (void)goal_handle;
    if (chain.empty()) {
      return {};
    }
    const auto robot_model = move_group.getRobotModel();
    const auto * joint_model_group =
      robot_model == nullptr ? nullptr :
      robot_model->getJointModelGroup(group_name);
    if (joint_model_group == nullptr) {
      return {};
    }
    const auto & variable_names = joint_model_group->getVariableNames();
    if (variable_names.size() != 7U) {
      return {};
    }
    const double kPi = std::acos(-1.0);

    const auto current_state = synchronized_current_robot_state(move_group);
    // 自然分支：以当前状态（home）为种子解 ready 位姿，即 OMPL 无种子时会落的
    // 分支（接近 home、无甩动）。它是所有候选变体的基准。
    moveit::core::RobotState natural(*current_state);
    if (!natural.setFromIK(
        joint_model_group, ready_pose, tool_link,
        std::min(1.0, planning_time_)))
    {
      return {};
    }
    natural.update();
    if (!natural.satisfiesBounds(joint_model_group)) {
      return {};
    }
    const auto read_positions = [&](const moveit::core::RobotState & state) {
      std::vector<double> positions;
      positions.reserve(variable_names.size());
      for (const auto & name : variable_names) {
        positions.push_back(state.getVariablePosition(name));
      }
      return positions;
    };
    const auto natural_positions = read_positions(natural);

    // 候选种子：自然分支 + 肩部(r_arm_0/l_arm_0)与腕部(r_arm_5/l_arm_5)扫描。
    // 保持其余关节为自然值，让 setFromIK 收敛到种子附近的 IK 分支。
    std::vector<std::vector<double>> candidates;
    candidates.reserve(1U + 11U + 11U);
    candidates.push_back(natural_positions);
    for (const double shoulder :
      { -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5 })
    {
      auto variant = natural_positions;
      variant[0] = shoulder;
      candidates.push_back(std::move(variant));
    }
    for (const double wrist :
      { -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5 })
    {
      auto variant = natural_positions;
      variant[5] = wrist;
      candidates.push_back(std::move(variant));
    }

    // 候选分支与自然分支的最大关节差（含角度回绕）。ready 用 setJointValueTarget
    // 钉到该分支；分支离自然越远，home→ready 的关节空间甩动越大（run6 的远侧
    // 解锁分支就是这种甩动的自锁来源），故只接受有限甩动且优先最小甩动。
    const double kSwingHardLimit = 5.0;
    const double kRequired = 0.99;
    const auto swing_vs_natural = [&](const std::vector<double> & values) {
      double max_swing = 0.0;
      for (std::size_t index = 0U; index < values.size(); ++index) {
        double delta = values[index] - natural_positions[index];
        delta -= std::round(delta / (2.0 * kPi)) * (2.0 * kPi);
        max_swing = std::max(max_swing, std::abs(delta));
      }
      return max_swing;
    };

    double best_fraction = -1.0;
    double best_swing = std::numeric_limits<double>::infinity();
    std::vector<double> best_clear;
    const auto run_chain = [&](const moveit::core::RobotState & start) {
      moveit::core::RobotState state(start);
      double min_fraction = 1.0;
      std::size_t segment_index = 0U;
      for (const auto & segment : chain) {
        if (segment_index == reseat_rebase_segment) {
          // 干跑镜像实跑 reseat 的终点：实跑用 OMPL 关节空间钉回分支 ready
          // 构型（终态恰为分支起点构型），干跑不跑那条末点 2 mm 内 IK 耗尽
          // 的直线，直接把段后状态置回分支起点继续验证支撑/预抓/抓取段。
          state = start;
          ++segment_index;
          continue;
        }
        move_group.setStartState(state);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = -1.0;
        try {
          fraction = move_group.computeCartesianPath(
            segment, 0.002, cartesian_jump_threshold_, trajectory, true);
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            get_logger(),
            "Drawer '%s' branch chain Cartesian threw: %s",
            description.c_str(), error.what());
        }
        if (fraction < min_fraction) {
          min_fraction = fraction;
        }
        if (fraction < kRequired || trajectory.joint_trajectory.points.empty()) {
          break;
        }
        robot_trajectory::RobotTrajectory segment_trajectory(
          move_group.getRobotModel(), group_name);
        segment_trajectory.setRobotTrajectoryMsg(state, trajectory);
        state = segment_trajectory.getLastWayPoint();
        ++segment_index;
      }
      return min_fraction;
    };

    for (const auto & seed : candidates) {
      try {
        moveit::core::RobotState branch(*current_state);
        branch.setVariablePositions(variable_names, seed);
        if (!branch.setFromIK(
            joint_model_group, ready_pose, tool_link,
            std::min(1.0, planning_time_)))
        {
          continue;
        }
        branch.update();
        if (!branch.satisfiesBounds(joint_model_group)) {
          continue;
        }
        const auto branch_positions = read_positions(branch);
        const double swing = swing_vs_natural(branch_positions);
        if (swing > kSwingHardLimit) {
          continue;
        }
        const double fraction = run_chain(branch);
        if (fraction > best_fraction) {
          best_fraction = fraction;
        }
        if (fraction >= kRequired) {
          if (swing < best_swing) {
            best_swing = swing;
            best_clear = branch_positions;
          }
        } else {
          RCLCPP_DEBUG(
            get_logger(),
            "Drawer '%s' branch swing=%.3f min_fraction=%.3f.",
            description.c_str(), swing, fraction);
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' branch validation failed: %s",
          description.c_str(), error.what());
      }
    }

    if (!best_clear.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' ready branch selected: full Cartesian chain clears "
        "(swing=%.3f rad vs natural).",
        description.c_str(), best_swing);
      return best_clear;
    }
    RCLCPP_WARN(
      get_logger(),
      "No drawer '%s' ready branch cleared the full Cartesian chain "
      "(best min fraction=%.3f); ready will pin the natural branch and "
      "report any downstream failure honestly.",
      description.c_str(), best_fraction > 0.0 ? best_fraction : 0.0);
    // 2026-09-03 cap4 run2: 空返回会让 ready 回退 OMPL 任意采样——解锁/支撑等
    // 下游 Cartesian 段从随机构型起跑，IK 是否跟得上完全靠抽签（run7 的
    // r5=+2.96 腕位 84.4% 截断、cap4 run2 的 unlock-approach 21.9% 截断都出自
    // 随机 ready）。即使无分支清整链，也钉回自然分支（home 邻域、最小甩动）：
    // 它的解锁四拍干跑 100%，下游若仍截断则是确定性的、可复现的失败。
    return natural_positions;
  }

  moveit::core::RobotState validate_pose_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const moveit::core::RobotState & start_state,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    const std::string & stage,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const ButtonSpec * control = nullptr)
  {
    check_cancel(goal_handle);
    update_validation_diagnostic(
      *result, stage, 0.0, 1.0, 0);
    RCLCPP_DEBUG(
      get_logger(),
      "Planning-only target [%s] link=%s frame=%s "
      "position=(%.9f, %.9f, %.9f) orientation=(%.9f, %.9f, %.9f, %.9f)",
      stage.c_str(), tool_link.c_str(), planning_frame_.c_str(),
      target.position.x, target.position.y, target.position.z,
      target.orientation.x, target.orientation.y,
      target.orientation.z, target.orientation.w);
    move_group.setStartState(start_state);
    // A calibrated seed pins OMPL to one IK branch.  When that branch is
    // unreachable or in collision, fall back to a pure pose target so the
    // validation still certifies the collision-free pose itself.
    MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result =
      moveit::core::MoveItErrorCode::FAILURE;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      move_group.setStartState(start_state);
      // Re-apply the calibrated seed on every attempt: setJointValueTarget
      // pins one IK branch, and clearPoseTargets() at the end of each
      // iteration wipes that target, so a const seed computed once would plan
      // to an empty goal (and never re-enter the seed branch) on later tries.
      const bool seed_ok = set_pose_target_with_calibrated_ik_seed(
        move_group, start_state, target, tool_link, control);
      if (seed_ok) {
        planning_result = move_group.plan(plan);
        move_group.clearPoseTargets();
        if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
          planned = true;
          break;
        }
      }
      move_group.setStartState(start_state);
      move_group.setPoseTarget(target, tool_link);
      planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "Planning-only validation stage '%s' failed (attempt %d/%d, "
        "MoveIt=%d).",
        stage.c_str(), attempt, motion_planning_attempts_, planning_result.val);
    }
    move_group.clearPoseTargets();
    if (!planned) {
      update_validation_diagnostic(
        *result, stage, 0.0, 1.0, planning_result.val);
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "无法生成无碰撞目标位姿轨迹。"));
    }
    update_validation_diagnostic(
      *result, stage, 1.0, 1.0,
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    return validation_trajectory_end_state(
      move_group, start_state, plan.trajectory_);
  }

  moveit::core::RobotState validate_named_target_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const moveit::core::RobotState & start_state,
    const std::string & target_name,
    const std::string & stage,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    check_cancel(goal_handle);
    update_validation_diagnostic(
      *result, stage, 0.0, 1.0, 0);
    move_group.setStartState(start_state);
    if (!move_group.setNamedTarget(target_name)) {
      update_validation_diagnostic(
        *result, stage, 0.0, 1.0,
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "MoveIt 不包含指定的安全命名位姿。"));
    }

    MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      move_group.setStartState(start_state);
      planning_result = move_group.plan(plan);
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
    }
    if (!planned) {
      update_validation_diagnostic(
        *result, stage, 0.0, 1.0, planning_result.val);
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "无法规划回到安全运输位姿。"));
    }
    update_validation_diagnostic(
      *result, stage, 1.0, 1.0,
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    return validation_trajectory_end_state(
      move_group, start_state, plan.trajectory_);
  }

  moveit::core::RobotState validate_cartesian_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const moveit::core::RobotState & start_state,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double minimum_fraction,
    const std::string & stage,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    check_cancel(goal_handle);
    update_validation_diagnostic(
      *result, stage, 0.0, minimum_fraction, 0);
    if (waypoints.empty()) {
      update_validation_diagnostic(
        *result, stage, 1.0, minimum_fraction,
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      return start_state;
    }
    moveit_msgs::msg::RobotTrajectory best_trajectory;
    moveit_msgs::msg::MoveItErrorCodes best_error;
    double best_fraction = -1.0;
    for (int attempt = 0; attempt < cartesian_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      move_group.setStartState(start_state);
      moveit_msgs::msg::RobotTrajectory candidate;
      moveit_msgs::msg::MoveItErrorCodes error;
      const double fraction = move_group.computeCartesianPath(
        waypoints, 0.002, cartesian_jump_threshold_, candidate, true, &error);
      if (fraction > best_fraction) {
        best_fraction = fraction;
        best_trajectory = std::move(candidate);
        best_error = error;
      }
      if (best_fraction >= minimum_fraction) {
        break;
      }
    }
    const double reported_fraction = std::max(0.0, best_fraction);
    update_validation_diagnostic(
      *result, stage, reported_fraction, minimum_fraction, best_error.val);
    if (best_fraction < minimum_fraction) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "笛卡尔路径没有达到当前执行路径使用的安全门槛。"));
    }

    robot_trajectory::RobotTrajectory timed_trajectory(
      move_group.getRobotModel(), move_group_name_);
    timed_trajectory.setRobotTrajectoryMsg(start_state, best_trajectory);
    if (timed_trajectory.getWayPointCount() > 0U) {
      trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterizer;
      if (!time_parameterizer.computeTimeStamps(
          timed_trajectory,
          cartesian_velocity_scale_,
          cartesian_acceleration_scale_))
      {
        update_validation_diagnostic(
          *result, stage, reported_fraction, minimum_fraction,
          moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
        throw OperationError(
                PressCabinetButton::Result::PLANNING_FAILED,
                planning_validation_failure_message(
                  *result, "笛卡尔候选轨迹无法生成安全时间参数。"));
      }
      update_validation_diagnostic(
        *result, stage, reported_fraction, minimum_fraction,
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      return timed_trajectory.getLastWayPoint();
    }
    update_validation_diagnostic(
      *result, stage, reported_fraction, minimum_fraction,
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    return start_state;
  }

  moveit::core::RobotState validate_segmented_cartesian_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    moveit::core::RobotState virtual_state,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    std::size_t maximum_segment_waypoints,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    if (maximum_segment_waypoints == 0U) {
      throw std::invalid_argument(
              "Planning-only Cartesian segment size must be positive.");
    }
    for (std::size_t begin = 0U; begin < waypoints.size();
      begin += maximum_segment_waypoints)
    {
      const auto end = std::min(
        begin + maximum_segment_waypoints, waypoints.size());
      const std::vector<geometry_msgs::msg::Pose> segment(
        waypoints.begin() + begin, waypoints.begin() + end);
      try {
        virtual_state = validate_cartesian_plan_only(
          move_group, goal_handle, virtual_state, segment, 0.99,
          "manipulation", result);
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          segment.size() == 1U)
        {
          throw;
        }
        // Match the real door executor's safe fallback without executing a
        // partial segment: retry each predicted waypoint from the preceding
        // virtual trajectory endpoint.
        for (const auto & waypoint : segment) {
          virtual_state = validate_cartesian_plan_only(
            move_group, goal_handle, virtual_state, {waypoint}, 0.99,
            "manipulation", result);
        }
      }
    }
    return virtual_state;
  }

  void validate_inoperable_control_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const ButtonSpec & control,
    const ButtonSnapshot & initial_state,
    double target_position,
    const OperationPoses & button_poses,
    const RotaryOperationPoses & rotary_poses,
    double rotary_tool_roll_offset,
    const moveit::core::RobotState & current_robot_state,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    // SAFETY CONTRACT: this function and every helper it calls are planning
    // only.  They may query current Gazebo/TF state and the MoveIt planning
    // scene, but they must never execute a trajectory, navigate, dock, attach
    // a grasp, wait for a physical transition, or write a cabinet joint.
    moveit::core::RobotState virtual_state(current_robot_state);
    const bool is_button = control.control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON ||
      control.control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SLIDER;
    if (is_button) {
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MOVING_TO_READY,
        0.25F, target_position,
        "正在用实时 MoveIt 场景验证控件预备位姿（不会执行轨迹）。");
      virtual_state = validate_pose_plan_only(
        move_group, goal_handle, virtual_state,
        button_poses.prepress_pose, contact_tool_link_, "ready_pose", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::APPROACHING,
        0.47F, target_position,
        "正在验证控件接近路径（不会移动机械臂）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.contact_pose}, 0.99, "approach", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MANIPULATING,
        0.65F, target_position,
        "正在验证请求对应的按压路径（不会接触控件）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.pressed_pose}, button_press_minimum_cartesian_fraction_,
        "manipulation", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::RETREATING,
        0.86F, 0.0,
        "正在验证控件撤回路径（不会执行轨迹）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.contact_pose, button_poses.prepress_pose},
        0.99, "retreat", result);
    } else {
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MOVING_TO_READY,
        0.25F, target_position,
        "正在用实时 MoveIt 场景验证控件预备位姿（不会执行轨迹）。");
      virtual_state = validate_pose_plan_only(
        move_group, goal_handle, virtual_state,
        rotary_poses.ready_pose, contact_tool_link_, "ready_pose", result,
        &control);
      if (control.control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR ||
          control.control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
      {
        virtual_state = validate_pose_plan_only(
          move_group, goal_handle, virtual_state,
          rotary_poses.pregrasp_pose, contact_tool_link_,
          "pregrasp", result);
      }
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::APPROACHING,
        0.43F, target_position,
        "正在验证抓取接近路径（不会建立物理抓取）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {rotary_poses.grasp_pose}, 0.99, "approach", result);

      const bool is_door = control.control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
      const double manipulation_position = rotary_manipulation_position(
        control, initial_state.position, target_position);
      const auto waypoints = calculate_rotation_waypoints(
        control, initial_state.position, manipulation_position,
        rotary_tool_roll_offset);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MANIPULATING,
        0.62F, target_position,
        "正在验证控件操作圆弧（不会转动或抓取控件）。");
      if (is_door) {
        virtual_state = validate_segmented_cartesian_plan_only(
          move_group, goal_handle, virtual_state, waypoints,
          static_cast<std::size_t>(door_cartesian_segment_waypoints_), result);
      } else {
        virtual_state = validate_cartesian_plan_only(
          move_group, goal_handle, virtual_state, waypoints,
          0.99, "manipulation", result);
      }
      const double release_clearance = is_door ?
        door_release_clearance_ : prepress_distance_;
      const auto target_ready = calculate_rotary_tool_pose(
        control, manipulation_position, release_clearance, false,
        rotary_tool_roll_offset);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::RETREATING,
        0.84F, target_position,
        "正在验证控件撤离路径（不会释放不存在的抓取）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state, {target_ready},
        0.99, "retreat", result);
    }

    publish_operate_feedback(
      goal_handle, OperateCabinetControl::Feedback::VERIFYING,
      0.96F, target_position,
      "正在验证从预测撤回终点返回安全运输位姿。");
    (void)validate_named_target_plan_only(
      move_group, goal_handle, virtual_state,
      transport_named_target_, "transport", result);
  }

  template<typename GoalHandleT>
  void plan_and_execute_pose(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    bool * operation_executed = nullptr,
    const ButtonSpec * control = nullptr,
    const std::vector<double> * seed_positions_override = nullptr)
  {
    check_cancel(goal_handle);
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      // Prefer the calibrated IK seed so the planned branch matches the pose
      // family the seed was calibrated for.  When the seed-derived branch is
      // unreachable or in collision (e.g. dock calibration drift), fall back
      // to a pure pose target and let OMPL sample any collision-free branch.
      const bool seed_ok = set_pose_target_with_calibrated_ik_seed(
        move_group, *current_state, target, tool_link, control,
        seed_positions_override);
      moveit::core::MoveItErrorCode planning_result =
        moveit::core::MoveItErrorCode::FAILURE;
      if (seed_ok) {
        planning_result = move_group.plan(plan);
        move_group.clearPoseTargets();
        if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
          planned = true;
          break;
        }
      }
      move_group.setStartState(*current_state);
      move_group.setPoseTarget(target, tool_link);
      const auto pose_planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
      if (pose_planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "MoveIt planning to the cabinet pose for '%s' failed "
        "(attempt %d/%d, seed=%d pose=%d).",
        tool_link.c_str(), attempt, motion_planning_attempts_,
        seed_ok ? planning_result.val : -1, pose_planning_result.val);
    }
    if (!planned) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not plan to the prepress pose after " +
              std::to_string(motion_planning_attempts_) +
              " attempts. If navigation was skipped, first park the base "
              "in front of the cabinet.");
    }
    check_cancel(goal_handle);
    if (operation_executed) {
      *operation_executed = true;
    }
    check_cancel(goal_handle);
    const auto prepress_code = execute_motion_bounded(
      [&move_group, &plan]() { return move_group.execute(plan); },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), "prepress pose");
    if (prepress_code != moveit::core::MoveItErrorCode::SUCCESS) {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt failed to execute the prepress trajectory.");
    }
    check_cancel(goal_handle);
  }

  template<typename GoalHandleT>
  void plan_and_execute_named_target(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::string & target_name,
    bool * operation_executed = nullptr,
    bool physical_outcome_committed = false)
  {
    const auto check_stop = [this, &goal_handle,
        physical_outcome_committed]() {
        if (physical_outcome_committed) {
          check_safety_transport_stop(goal_handle);
        } else {
          check_cancel(goal_handle);
        }
      };
    check_stop();
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_stop();
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      if (!move_group.setNamedTarget(target_name)) {
        throw OperationError(
                PressCabinetButton::Result::PLANNING_FAILED,
                "MoveIt does not contain the arm target '" + target_name +
                "'.");
      }
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "MoveIt planning to named target '%s' failed (attempt %d/%d).",
        target_name.c_str(), attempt, motion_planning_attempts_);
    }
    if (!planned) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not plan the arm to the safe '" + target_name +
              "' target after " +
              std::to_string(motion_planning_attempts_) +
              " attempts.");
    }
    check_stop();
    if (operation_executed) {
      *operation_executed = true;
    }
    const auto stow_code = execute_motion_bounded(
      [&move_group, &plan]() { return move_group.execute(plan); },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), "named target '" + target_name + "'");
    if (stow_code != moveit::core::MoveItErrorCode::SUCCESS) {
      check_stop();
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt could not return the arm to the safe '" + target_name +
              "' target.");
    }
    check_stop();
  }

  template<typename GoalHandleT>
  void execute_segmented_cartesian_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    std::size_t maximum_segment_waypoints,
    double velocity_scale,
    double acceleration_scale,
    std::size_t & completed_waypoints,
    bool * operation_executed = nullptr)
  {
    if (maximum_segment_waypoints == 0U) {
      throw std::invalid_argument(
              "Cartesian segment size must be greater than zero.");
    }
    completed_waypoints = 0U;
    const std::size_t segment_count =
      (waypoints.size() + maximum_segment_waypoints - 1U) /
      maximum_segment_waypoints;
    std::size_t segment_index = 0U;
    for (std::size_t begin = 0; begin < waypoints.size();
      begin += maximum_segment_waypoints)
    {
      ++segment_index;
      check_cancel(goal_handle);
      const auto end = std::min(
        begin + maximum_segment_waypoints, waypoints.size());
      const std::vector<geometry_msgs::msg::Pose> segment(
        waypoints.begin() + begin, waypoints.begin() + end);
      RCLCPP_INFO(
        get_logger(),
        "Starting door Cartesian segment %zu/%zu [%zu, %zu); "
        "%zu/%zu waypoints already executed.",
        segment_index, segment_count, begin, end,
        completed_waypoints, waypoints.size());
      try {
        execute_cartesian_path(
          move_group, goal_handle, segment,
          velocity_scale, acceleration_scale, 0.99, operation_executed);
        completed_waypoints = end;
        RCLCPP_INFO(
          get_logger(),
          "Door Cartesian segment %zu/%zu [%zu, %zu) completed; "
          "%zu/%zu waypoints executed.",
          segment_index, segment_count, begin, end,
          completed_waypoints, waypoints.size());
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          segment.size() == 1U)
        {
          throw;
        }

        // Recomputing each waypoint from the measured joint state can escape
        // an IK/state-validity dead end without ever executing a partial
        // Cartesian plan. The fallback also reduces the maximum swept-door
        // update interval from four degrees to two degrees.
        RCLCPP_WARN(
          get_logger(),
          "Door Cartesian segment %zu/%zu [%zu, %zu) was not fully "
          "plannable after %zu/%zu completed waypoints; retrying it one "
          "waypoint at a time.",
          segment_index, segment_count, begin, end,
          completed_waypoints, waypoints.size());
        for (std::size_t offset = 0U; offset < segment.size(); ++offset) {
          execute_cartesian_path(
            move_group, goal_handle, {segment[offset]},
            velocity_scale, acceleration_scale, 0.99,
            operation_executed);
          completed_waypoints = begin + offset + 1U;
          RCLCPP_INFO(
            get_logger(),
            "Door Cartesian fallback waypoint %zu/%zu completed; "
            "%zu/%zu waypoints executed.",
            offset + 1U, segment.size(), completed_waypoints,
            waypoints.size());
        }
      }
    }
  }

  template<typename GoalHandleT>
  void execute_cartesian_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale,
    double minimum_fraction = 0.99,
    bool * operation_executed = nullptr)
  {
    check_cancel(goal_handle);
    moveit_msgs::msg::RobotTrajectory trajectory_message;
    double best_fraction = -1.0;
    for (int attempt = 0; attempt < cartesian_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      moveit_msgs::msg::RobotTrajectory candidate;
      const double fraction = move_group.computeCartesianPath(
        waypoints,
        0.002,
        cartesian_jump_threshold_,
        candidate,
        true);
      if (fraction > best_fraction) {
        best_fraction = fraction;
        trajectory_message = std::move(candidate);
      }
      if (best_fraction >= minimum_fraction) {
        break;
      }
    }
    if (best_fraction < minimum_fraction) {
      // Diagnostic: report the approach start state and the tool link pose at
      // the last collision-free waypoint so a Cartesian clearance regression
      // can be reproduced and fixed from the config layer alone.
      try {
        const auto start_state = synchronized_current_robot_state(move_group);
        std::ostringstream oss;
        oss << "Cartesian fraction=" << best_fraction;
        oss << " start_joints=";
        for (const auto & name : start_state->getVariableNames()) {
          if (name.find("r_arm") == 0 || name.find("l_arm") == 0 ||
            name.find("cyl") != std::string::npos ||
            name.find("r_three") == 0)
          {
            oss << name << ":" << start_state->getVariablePosition(name)
              << " ";
          }
        }
        if (!trajectory_message.joint_trajectory.points.empty()) {
          const auto & last = trajectory_message.joint_trajectory.points.back();
          const auto & names = trajectory_message.joint_trajectory.joint_names;
          moveit::core::RobotState waypoint_state(move_group.getRobotModel());
          waypoint_state.setToDefaultValues();
          for (std::size_t i = 0; i < names.size() && i < last.positions.size();
            ++i)
          {
            waypoint_state.setVariablePosition(names[i], last.positions[i]);
          }
          waypoint_state.update();
          const Eigen::Isometry3d & link_pose =
            waypoint_state.getGlobalLinkTransform(contact_tool_link_);
          const Eigen::Vector3d t = link_pose.translation();
          oss << " last_waypoint_tool_pose=(" << t.x() << "," << t.y()
            << "," << t.z() << ")";
          oss << " waypoint_joints=";
          for (std::size_t i = 0; i < names.size() && i < last.positions.size();
            ++i)
          {
            if (names[i].find("r_arm") == 0 || names[i].find("cyl") !=
              std::string::npos || names[i].find("r_three") == 0)
            {
              oss << names[i] << ":" << last.positions[i] << " ";
            }
          }
        }
        RCLCPP_ERROR(get_logger(), "%s", oss.str().c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(), "Cartesian failure diagnostic unavailable: %s",
          e.what());
      }
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt completed only " +
              std::to_string(std::max(0.0, best_fraction) * 100.0) +
              "% of the Cartesian control path; this segment requires " +
              std::to_string(minimum_fraction * 100.0) + "% after " +
              std::to_string(cartesian_planning_attempts_) + " attempts.");
    }
    retime_cartesian_trajectory(
      move_group,
      trajectory_message,
      velocity_scale,
      acceleration_scale);
    check_cancel(goal_handle);
    if (operation_executed) {
      *operation_executed = true;
    }
    const auto cartesian_code = execute_motion_bounded(
      [&move_group, &trajectory_message]() {
        return move_group.execute(trajectory_message);
      },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), "Cartesian button path");
    if (cartesian_code != moveit::core::MoveItErrorCode::SUCCESS) {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt failed to execute a Cartesian button trajectory.");
    }
    check_cancel(goal_handle);
  }

  void retime_cartesian_trajectory(
    MoveGroupInterface & move_group,
    moveit_msgs::msg::RobotTrajectory & trajectory_message,
    double velocity_scale,
    double acceleration_scale)
  {
    const auto current_state = synchronized_current_robot_state(move_group);
    robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), move_group_name_);
    trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterizer;
    if (!time_parameterizer.computeTimeStamps(
        trajectory,
        velocity_scale,
        acceleration_scale))
    {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not generate a low-speed Cartesian trajectory.");
    }
    trajectory.getRobotTrajectoryMsg(trajectory_message);
  }

  // MoveIt2 Humble's move_group.execute() busy-waits an internal done flag with
  // no timeout.  If the controller's ExecuteTrajectory result is lost, the call
  // never returns and the operator permanently blocks, holding operation_active_
  // true so every later goal returns RESOURCE_BUSY ("一直失败") until the whole
  // stack is restarted.  This wrapper runs the blocking motion on a worker
  // thread, polls a cancel predicate and a hard deadline, and when either fires
  // first asks stop_active_motion() to cancel the controller goal and then, if
  // the worker still will not return, abandons it (detach) and reports failure
  // so the operation fails cleanly instead of wedging the operator forever.
  //
  // should_stop may be null (best-effort safety retreat/stow have no goal
  // handle): then only the hard deadline applies, so a lost controller result
  // can never block a safety motion indefinitely either.  goal_should_stop on a
  // null handle would return true immediately, so callers must pass null here
  // for those noexcept best-effort paths.
  //
  // Safety properties of the wedge path (detach):
  //  - The worker never throws: motion() is wrapped so a throwing action-client
  //    cannot escape the thread and call std::terminate.
  //  - The worker holds shared ownership of the motion std::function and the
  //    result promise, so the function object survives the caller's stack; the
  //    motion's by-reference captures are only read while execute() serializes
  //    the goal, which happened before any wedge was detected.
  //  - A second execute on the same MoveGroupInterface cannot race the wedged
  //    in-flight one: every bounded motion first takes a serialization lock
  //    (motion_execute_serial_->execute_mutex), and the wedge transfers that
  //    lock into the shared serialization state until the wedged worker
  //    eventually returns.  Retreat/stow after a wedge therefore fail fast with
  //    FAILURE instead of launching a concurrent move_group.execute().  A fresh
  //    MoveGroupInterface for a later goal (acquire_active_move_group) clears
  //    the wedge lock so the new, independent instance is not bricked forever.
  moveit::core::MoveItErrorCode execute_motion_bounded(
    const std::function<moveit::core::MoveItErrorCode()> & motion,
    const std::function<bool()> & should_stop,
    const std::chrono::steady_clock::duration & hard_deadline,
    const std::string & description)
  {
    const auto serial = motion_execute_serial_;
    // Refuse immediately (no blocking) if a prior wedge still holds the
    // execute serialization: launching a second execute() on the same
    // MoveGroupInterface while the wedged worker is still inside the first
    // one would be a data race on a non-thread-safe object.
    std::unique_lock<std::mutex> execute_serial(
      serial->execute_mutex, std::try_to_lock);
    if (!execute_serial.owns_lock()) {
      RCLCPP_ERROR(
        get_logger(),
        "Bounded '%s' refused: a previous bounded motion is still in flight "
        "on this move group (wedged backend); refusing a concurrent execute.",
        description.c_str());
      return moveit::core::MoveItErrorCode::FAILURE;
    }
    const auto result_holder =
      std::make_shared<std::promise<moveit::core::MoveItErrorCode>>();
    std::future<moveit::core::MoveItErrorCode> future =
      result_holder->get_future();
    // Heap-shared copy of the motion closure: a detached worker outlives this
    // call and must not touch the caller's stack-owned std::function.
    const auto motion_holder =
      std::make_shared<std::function<moveit::core::MoveItErrorCode()>>(
        motion);
    const rclcpp::Logger bounded_logger = get_logger();
    std::thread worker([result_holder, motion_holder, serial,
        bounded_logger]() {
      try {
        result_holder->set_value((*motion_holder)());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          bounded_logger,
          "Bounded motion threw inside its worker thread: %s",
          error.what());
        try {
          result_holder->set_value(moveit::core::MoveItErrorCode::FAILURE);
        } catch (...) {
        }
      } catch (...) {
        try {
          result_holder->set_value(moveit::core::MoveItErrorCode::FAILURE);
        } catch (...) {
        }
      }
      // If this worker was detached on the wedge path, its in-flight
      // execute() transferred the serialization lock into serial->wedged_lock;
      // clear it now that the execute has actually returned so later bounded
      // motions can run again.
      std::lock_guard<std::mutex> lock(serial->guard_mutex);
      serial->wedged_lock.reset();
    });
    const auto deadline_time =
      std::chrono::steady_clock::now() + hard_deadline;
    moveit::core::MoveItErrorCode code =
      moveit::core::MoveItErrorCode::FAILURE;
    bool worker_detached = false;
    while (true) {
      if (future.wait_for(std::chrono::milliseconds(50)) ==
        std::future_status::ready) {
        code = future.get();
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      const bool stop_requested =
        (should_stop && should_stop()) || now >= deadline_time;
      if (stop_requested) {
        RCLCPP_WARN(
          get_logger(),
          "Bounded '%s' hit its deadline or was canceled; stopping motion.",
          description.c_str());
        stop_active_motion();
        const auto grace_deadline =
          now + std::chrono::milliseconds(2500);
        while (future.wait_for(std::chrono::milliseconds(50)) !=
          std::future_status::ready) {
          if (std::chrono::steady_clock::now() >= grace_deadline) {
            break;
          }
        }
        if (future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
          code = future.get();
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "Bounded '%s' is permanently wedged (lost controller result); "
            "abandoning the motion worker and failing the operation.",
            description.c_str());
          // Keep the execute serialization held until the wedged worker
          // returns (worker clears it) or a fresh move group replaces this
          // one, so no retreat/stow on this instance races the in-flight
          // execute() with a concurrent move_group.execute().
          {
            std::lock_guard<std::mutex> lock(serial->guard_mutex);
            serial->wedged_lock.emplace(std::move(execute_serial));
          }
          worker.detach();
          worker_detached = true;
          code = moveit::core::MoveItErrorCode::FAILURE;
        }
        break;
      }
    }
    if (!worker_detached && worker.joinable()) {
      worker.join();
    }
    return code;
  }

  void retime_cartesian_trajectory_for_group(
    MoveGroupInterface & move_group,
    const std::string & group_name,
    moveit_msgs::msg::RobotTrajectory & trajectory_message,
    double velocity_scale,
    double acceleration_scale)
  {
    const auto current_state = synchronized_current_robot_state(move_group);
    robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), group_name);
    trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterizer;
    if (!time_parameterizer.computeTimeStamps(
        trajectory,
        velocity_scale,
        acceleration_scale))
    {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not generate a low-speed Cartesian trajectory "
              "for drawer group '" + group_name + "'.");
    }
    trajectory.getRobotTrajectoryMsg(trajectory_message);
  }

  struct BimanualExecutionResult
  {
    bool left_started{false};
    bool right_started{false};
    moveit::core::MoveItErrorCode left_code{
      moveit::core::MoveItErrorCode::FAILURE};
    moveit::core::MoveItErrorCode right_code{
      moveit::core::MoveItErrorCode::FAILURE};
  };

  // 把 move_group 规划好的关节轨迹直发 follow_joint_trajectory 控制器并等待
  // 结果。左右各由独立 worker 调用，因此两条臂真正并行执行（不再经过
  // move_group 的 execute_trajectory action——该 action 在单线程执行器上串行
  // 处理目标，先执行的那条臂会被另一条臂对抽屉的焊接固定物理锁死，最终以
  // GOAL_TOLERANCE_VIOLATED 超时）。超时/取消由调用方的有界执行循环经
  // stop_active_motion() 撤销控制器目标驱动本函数返回。本函数不触碰 this，
  // detach 的楔死 worker 在 operator 销毁后仍可安全运行。
  // 嵌套类型声明于类成员区（参数签名需要先于完整定义前向声明；函数体在
  // complete-class 上下文，仍可见 10461 处的完整定义）。
  struct BimanualControllerHandles;
  static moveit::core::MoveItErrorCode execute_controller_trajectory(
    const rclcpp::Logger & logger,
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory> & client,
    const std::shared_ptr<BimanualControllerHandles> & handles,
    const trajectory_msgs::msg::JointTrajectory & joint_trajectory,
    bool left_side) noexcept
  {
    if (joint_trajectory.points.empty()) {
      return moveit::core::MoveItErrorCode::SUCCESS;
    }
    if (!client.wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(
        logger,
        "Bimanual %s-arm controller action server is unavailable.",
        left_side ? "left" : "right");
      return moveit::core::MoveItErrorCode::FAILURE;
    }
    auto & goal_slot = left_side ? handles->left : handles->right;
    auto goal = control_msgs::action::FollowJointTrajectory::Goal();
    goal.trajectory = joint_trajectory;
    // 到达确定性：不带逐关节 goal_tolerance 时，ros2_control 在轨迹名义
    // 终点即宣告 Goal reached，而 PID 可能仍在收敛（cap1/cap2 实测首样本
    // 距目标 ~8 mm / 稳定带内漂移 ~3 mm）。与解锁电机 FJT（6310-6375，
    // 故障注入 4/4 PASS）同一语义：逐关节位置容差 + 名义终点后的宽限窗。
    // 窗口期内控制器持续保持末点指令并复查容差，真正收敛后才 SUCCESS；
    // 收敛不了的关节在窗口耗尽后报 GOAL_TOLERANCE_VIOLATED（可见失败，
    // 由调用方 120 s 有界截止兜底），不再静默带漂移到达。
    const auto & joints = joint_trajectory.joint_names;
    goal.goal_tolerance.resize(joints.size());
    for (std::size_t index = 0U; index < joints.size(); ++index) {
      auto & tolerance = goal.goal_tolerance[index];
      tolerance.name = joints[index];
      // 双臂静态保持实测可达 ~0.003 rad 以内；0.005 留足 PID 余量，
      // 收敛后的几何精度仍由调用方稳定带 verify（0.008/0.02/0.003）把关。
      tolerance.position = 0.005;
      tolerance.velocity = 0.0;
      tolerance.acceleration = 0.0;
    }
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(10.0);
    try {
      const auto send_future = client.async_send_goal(goal);
      const auto send_result = send_future.get();
      if (!send_result) {
        RCLCPP_ERROR(
          logger, "Bimanual %s-arm controller goal was rejected.",
          left_side ? "left" : "right");
        return moveit::core::MoveItErrorCode::FAILURE;
      }
      {
        std::lock_guard<std::mutex> lock(handles->mutex);
        goal_slot = send_result;
      }
      const auto result_future = client.async_get_result(send_result);
      while (result_future.wait_for(std::chrono::milliseconds(50)) !=
        std::future_status::ready) {
      }
      {
        std::lock_guard<std::mutex> lock(handles->mutex);
        goal_slot.reset();
      }
      const auto wrapped_result = result_future.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result->error_code ==
          control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
      {
        return moveit::core::MoveItErrorCode::SUCCESS;
      }
      RCLCPP_WARN(
        logger,
        "Bimanual %s-arm controller execution ended with status %d and "
        "error code %d.",
        left_side ? "left" : "right",
        static_cast<int>(wrapped_result.code),
        wrapped_result.result->error_code);
      return moveit::core::MoveItErrorCode::CONTROL_FAILED;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        logger, "Bimanual %s-arm direct controller execution threw: %s",
        left_side ? "left" : "right", error.what());
      {
        std::lock_guard<std::mutex> lock(handles->mutex);
        goal_slot.reset();
      }
      return moveit::core::MoveItErrorCode::FAILURE;
    }
  }

  // Concurrent two-arm execution primitive for the bimanual drawer.
  //
  // The left and right arms are driven by SEPARATE FollowJointTrajectory
  // controllers through distinct MoveGroupInterface instances, so both
  // execute() calls can genuinely run in parallel.  The single-arm
  // execute_motion_bounded serializes every execute() on one shared mutex,
  // which would defeat the synchronization; this primitive instead runs both
  // workers under ONE hold of that serialization (both plans belong to the
  // same drawer operation), and stops BOTH arms on cancel/deadline by way of
  // stop_active_motion(), which now also cancels bimanual_active_move_groups_.
  //
  // The wedge path mirrors execute_motion_bounded: on a lost controller
  // result both workers are detached and the serialization lock is transferred
  // into motion_execute_serial_->wedged_lock.  Only the LAST worker to return
  // clears that wedge (shared atomic counter), so a normally-returning right
  // worker cannot clear the wedge while the wedged left worker is still inside
  // execute() on the left group.
  BimanualExecutionResult execute_bimanual_trajectories_bounded(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const moveit_msgs::msg::RobotTrajectory & left_trajectory,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const moveit_msgs::msg::RobotTrajectory & right_trajectory,
    const std::function<bool()> & should_stop,
    const std::chrono::steady_clock::duration & hard_deadline,
    const std::string & description)
  {
    BimanualExecutionResult result;
    const auto serial = motion_execute_serial_;
    std::unique_lock<std::mutex> execute_serial(
      serial->execute_mutex, std::try_to_lock);
    if (!execute_serial.owns_lock()) {
      RCLCPP_ERROR(
        get_logger(),
        "Bimanual '%s' refused: a previous bounded motion is still in flight "
        "(wedged backend).",
        description.c_str());
      return result;
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      bimanual_active_move_groups_ = {left_group, right_group};
    }
    struct BimanualWorkerState
    {
      std::mutex guard_mutex;
      std::atomic<int> remaining_workers{2};
      bool cleared_wedge{false};
    };
    const auto worker_state = std::make_shared<BimanualWorkerState>();
    const auto left_promise = std::make_shared<
      std::promise<moveit::core::MoveItErrorCode>>();
    auto left_future = left_promise->get_future();
    const auto right_promise = std::make_shared<
      std::promise<moveit::core::MoveItErrorCode>>();
    auto right_future = right_promise->get_future();
    const rclcpp::Logger bounded_logger = get_logger();
    // 轨迹按值拷贝进 worker：楔死 detach 后 worker 可能晚于调用方返回，
    // 不能引用调用方栈上的 left_trajectory。控制器 client 与 goal-handle 池
    // 按值捕获，detach worker 全程不触碰 this；group 由调用方在
    // bimanual_active_move_groups_ 中保持存活（stop_active_motion 用）。
    const auto run_one = [bounded_logger, serial, worker_state,
        handles = bimanual_controller_handles_,
        left_client = left_fjt_client_,
        right_client = right_fjt_client_](
        bool left_side,
        const moveit_msgs::msg::RobotTrajectory trajectory,
        const std::shared_ptr<std::promise<moveit::core::MoveItErrorCode>> &
        promise) {
        moveit::core::MoveItErrorCode code =
          moveit::core::MoveItErrorCode::FAILURE;
        try {
          auto & client = left_side ? *left_client : *right_client;
          code = execute_controller_trajectory(
            bounded_logger, client, handles, trajectory.joint_trajectory,
            left_side);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            bounded_logger, "Bimanual worker threw: %s", error.what());
        } catch (...) {
        }
        try {
          promise->set_value(code);
        } catch (...) {
        }
        if (worker_state->remaining_workers.fetch_sub(1) == 1) {
          std::lock_guard<std::mutex> lock(serial->guard_mutex);
          serial->wedged_lock.reset();
        }
      };
    std::thread left_worker(run_one, true, left_trajectory, left_promise);
    std::thread right_worker(run_one, false, right_trajectory, right_promise);
    result.left_started = true;
    result.right_started = true;

    const auto deadline_time =
      std::chrono::steady_clock::now() + hard_deadline;
    bool left_wedged = false;
    bool right_wedged = false;
    bool worker_detached = false;
    while (true) {
      const auto left_ready = left_future.wait_for(
        std::chrono::milliseconds(50)) == std::future_status::ready;
      const auto right_ready = right_future.wait_for(
        std::chrono::milliseconds(0)) == std::future_status::ready;
      if (left_ready && right_ready) {
        result.left_code = left_future.get();
        result.right_code = right_future.get();
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      const bool stop_requested =
        (should_stop && should_stop()) || now >= deadline_time;
      if (!stop_requested) {
        continue;
      }
      RCLCPP_WARN(
        get_logger(),
        "Bimanual '%s' hit its deadline or was canceled; stopping both arms.",
        description.c_str());
      stop_active_motion();
      const auto grace_deadline = now + std::chrono::milliseconds(2500);
      while (true) {
        const auto grace_left_ready = left_future.wait_for(
          std::chrono::milliseconds(50)) == std::future_status::ready;
        const auto grace_right_ready = right_future.wait_for(
          std::chrono::milliseconds(0)) == std::future_status::ready;
        if (grace_left_ready && grace_right_ready) {
          break;
        }
        if (std::chrono::steady_clock::now() >= grace_deadline) {
          break;
        }
      }
      if (left_future.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        result.left_code = left_future.get();
      } else {
        left_wedged = true;
      }
      if (right_future.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        result.right_code = right_future.get();
      } else {
        right_wedged = true;
      }
      if (left_wedged || right_wedged) {
        RCLCPP_ERROR(
          get_logger(),
          "Bimanual '%s' is permanently wedged on %s%s%s; abandoning both "
          "motion workers and failing the operation.",
          description.c_str(),
          left_wedged ? "left" : "",
          left_wedged && right_wedged ? "+" : "",
          right_wedged ? "right" : "");
        {
          std::lock_guard<std::mutex> lock(serial->guard_mutex);
          serial->wedged_lock.emplace(std::move(execute_serial));
        }
        worker_detached = true;
        left_worker.detach();
        right_worker.detach();
      }
      break;
    }
    if (!worker_detached) {
      if (left_worker.joinable()) {
        left_worker.join();
      }
      if (right_worker.joinable()) {
        right_worker.join();
      }
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      bimanual_active_move_groups_ = {};
    }
    return result;
  }

  // Plan one arm's Cartesian segment from its measured state and retime it.
  // Used by the bimanual segmented path so BOTH arm plans exist before either
  // executes (a one-sided execute would yank the drawer).
  template<typename GoalHandleT>
  moveit_msgs::msg::RobotTrajectory plan_drawer_cartesian_segment(
    MoveGroupInterface & move_group,
    const std::string & group_name,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale,
    const std::string & description,
    double minimum_fraction = 0.99)
  {
    check_cancel(goal_handle);
    moveit_msgs::msg::RobotTrajectory trajectory_message;
    double best_fraction = -1.0;
    for (int attempt = 0; attempt < cartesian_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      const auto current_state = synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      moveit_msgs::msg::RobotTrajectory candidate;
      double fraction = -1.0;
      try {
        fraction = move_group.computeCartesianPath(
          waypoints, 0.002, cartesian_jump_threshold_, candidate, true);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' Cartesian planning threw on attempt %d: %s",
          description.c_str(), attempt + 1, error.what());
      }
      if (fraction > best_fraction) {
        best_fraction = fraction;
        trajectory_message = std::move(candidate);
      }
      if (best_fraction >= minimum_fraction) {
        break;
      }
    }
    if (best_fraction < minimum_fraction) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "Drawer '" + description + "' Cartesian segment completed only " +
              std::to_string(std::max(0.0, best_fraction) * 100.0) + "% of "
              "the path (needs " +
              std::to_string(minimum_fraction * 100.0) + "%).");
    }
    retime_cartesian_trajectory_for_group(
      move_group, group_name, trajectory_message,
      velocity_scale, acceleration_scale);
    check_cancel(goal_handle);
    return trajectory_message;
  }

  // P3-8: 双臂焊接后共享同一导轨，任一时刻两把工具在导轨上的位置必须一致。
  // 两条臂各自 TOTG 重定时（关节速度上限不同）时长可能不同；时长差会让较快
  // 一条臂在慢臂到达前被固定关节顶住，ODE 闭环自锁 → 控制器 GOAL_TOLERANCE
  // 超时。把两条轨迹按较慢一条统一线性拉伸到同一末时刻，保持空间-时间比例，
  // 使双臂沿导轨严格同步。
  static void synchronize_bimanual_trajectory_durations(
    moveit_msgs::msg::RobotTrajectory & left,
    moveit_msgs::msg::RobotTrajectory & right)
  {
    const auto last_duration = [](
      const moveit_msgs::msg::RobotTrajectory & trajectory) {
      const auto & points = trajectory.joint_trajectory.points;
      if (points.empty()) {
        return 0.0;
      }
      return rclcpp::Duration(points.back().time_from_start).seconds();
    };
    const double target = std::max(
      last_duration(left), last_duration(right));
    if (target <= 0.0) {
      return;
    }
    const auto stretch_to_target = [target](
      moveit_msgs::msg::RobotTrajectory & trajectory) {
      auto & points = trajectory.joint_trajectory.points;
      if (points.empty()) {
        return;
      }
      const double duration =
        rclcpp::Duration(points.back().time_from_start).seconds();
      if (duration <= 0.0) {
        return;
      }
      const double scale = target / duration;
      for (auto & point : points) {
        point.time_from_start = rclcpp::Duration::from_seconds(
          rclcpp::Duration(point.time_from_start).seconds() * scale);
      }
    };
    stretch_to_target(left);
    stretch_to_target(right);
  }

  // Segmented, matched, concurrent drawer pull/push.  Each segment is planned
  // for BOTH arms from their measured states and then executed concurrently;
  // the i-th left/right waypoint always command the same drawer rail position.
  // A segment that cannot be planned on either arm is retried one waypoint at
  // a time; a single unplannable waypoint fails the operation (the caller
  // stops, releases the bimanual grasp and retreats both arms).
  template<typename GoalHandleT>
  void execute_bimanual_segmented_cartesian_path(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & left_waypoints,
    const std::vector<geometry_msgs::msg::Pose> & right_waypoints,
    std::size_t maximum_segment_waypoints,
    double velocity_scale,
    double acceleration_scale,
    std::size_t & completed_waypoints,
    bool * operation_executed = nullptr)
  {
    if (left_waypoints.size() != right_waypoints.size()) {
      throw std::invalid_argument(
              "Bimanual drawer waypoint vectors must have equal length.");
    }
    if (maximum_segment_waypoints == 0U) {
      throw std::invalid_argument(
              "Bimanual drawer Cartesian segment size must be greater than "
              "zero.");
    }
    completed_waypoints = 0U;
    const auto run_segment = [&](
        std::size_t end,
        const std::vector<geometry_msgs::msg::Pose> & left_segment,
        const std::vector<geometry_msgs::msg::Pose> & right_segment) {
        check_cancel(goal_handle);
        auto left_trajectory = plan_drawer_cartesian_segment(
          *left_group, drawer_left_tool_.move_group, goal_handle,
          left_segment, velocity_scale, acceleration_scale, "left");
        auto right_trajectory = plan_drawer_cartesian_segment(
          *right_group, drawer_right_tool_.move_group, goal_handle,
          right_segment, velocity_scale, acceleration_scale, "right");
        // P3-8: 两条臂各自 TOTG 重定时时长不同，焊接闭环自锁；统一到同一
        // 末时刻再执行（见 synchronize_bimanual_trajectory_durations）。
        synchronize_bimanual_trajectory_durations(
          left_trajectory, right_trajectory);
        const auto execution = execute_bimanual_trajectories_bounded(
          left_group, left_trajectory,
          right_group, right_trajectory,
          [this, &goal_handle]() { return goal_should_stop(goal_handle); },
          std::chrono::seconds(120), "drawer segment");
        if (execution.left_code != moveit::core::MoveItErrorCode::SUCCESS ||
          execution.right_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
          check_cancel(goal_handle);
          throw OperationError(
                  PressCabinetButton::Result::EXECUTION_FAILED,
                  "Drawer bimanual segment execution failed (left=" +
                  std::to_string(execution.left_code.val) + " right=" +
                  std::to_string(execution.right_code.val) + ").");
        }
        if (operation_executed) {
          *operation_executed = true;
        }
        verify_drawer_arm_pose_stable(
          *left_group, goal_handle, left_segment.back(),
          drawer_left_tool_.contact_tool_link, "pull/push segment left");
        verify_drawer_arm_pose_stable(
          *right_group, goal_handle, right_segment.back(),
          drawer_right_tool_.contact_tool_link, "pull/push segment right");
        completed_waypoints = end;
      };
    const std::size_t segment_count =
      (left_waypoints.size() + maximum_segment_waypoints - 1U) /
      maximum_segment_waypoints;
    std::size_t segment_index = 0U;
    for (std::size_t begin = 0U; begin < left_waypoints.size();
      begin += maximum_segment_waypoints)
    {
      ++segment_index;
      const auto end = std::min(
        begin + maximum_segment_waypoints, left_waypoints.size());
      const std::vector<geometry_msgs::msg::Pose> left_segment(
        left_waypoints.begin() + begin, left_waypoints.begin() + end);
      const std::vector<geometry_msgs::msg::Pose> right_segment(
        right_waypoints.begin() + begin, right_waypoints.begin() + end);
      RCLCPP_INFO(
        get_logger(),
        "Starting drawer bimanual segment %zu/%zu [%zu, %zu); "
        "%zu/%zu waypoints already executed.",
        segment_index, segment_count, begin, end,
        completed_waypoints, left_waypoints.size());
      try {
        run_segment(end, left_segment, right_segment);
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          left_segment.size() == 1U)
        {
          throw;
        }
        RCLCPP_WARN(
          get_logger(),
          "Drawer bimanual segment %zu/%zu was not fully plannable on both "
          "arms; retrying one waypoint at a time.",
          segment_index, segment_count);
        for (std::size_t offset = 0U; offset < left_segment.size(); ++offset) {
          run_segment(
            begin + offset + 1U,
            {left_segment[offset]}, {right_segment[offset]});
          RCLCPP_INFO(
            get_logger(),
            "Drawer bimanual fallback waypoint %zu/%zu completed; "
            "%zu/%zu waypoints executed.",
            offset + 1U, left_segment.size(), completed_waypoints,
            left_waypoints.size());
        }
      }
    }
  }

  bool best_effort_cartesian_move(
    MoveGroupInterface & move_group,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale,
    const char * description,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      move_group.stop();
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      moveit_msgs::msg::RobotTrajectory trajectory_message;
      const double fraction = move_group.computeCartesianPath(
        waypoints,
        0.002,
        cartesian_jump_threshold_,
        trajectory_message,
        true);
      if (fraction < 0.99) {
        RCLCPP_ERROR(
          get_logger(), "%s path was only %.1f%% complete.",
          description, fraction * 100.0);
        return false;
      }
      retime_cartesian_trajectory(
        move_group,
        trajectory_message,
        velocity_scale,
        acceleration_scale);
      if (operation_executed) {
        *operation_executed = true;
      }
      const auto best_effort_code = execute_motion_bounded(
        [&move_group, &trajectory_message]() {
          return move_group.execute(trajectory_message);
        },
        std::function<bool()>(), std::chrono::seconds(60),
        description);
      if (best_effort_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "%s execution failed.", description);
        return false;
      }
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "%s failed: %s", description, error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "%s failed with an unknown error.", description);
    }
    return false;
  }

  bool best_effort_rollback_door_arc(
    MoveGroupInterface & move_group,
    const ButtonSpec & control,
    const DoorArcProgress & progress,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      const auto current_state = button_snapshot(control);
      if (!std::isfinite(current_state.position)) {
        RCLCPP_ERROR(
          get_logger(),
          "Door arc rollback cannot start from a non-finite position.");
        return false;
      }

      std::vector<geometry_msgs::msg::Pose> rollback_waypoints;
      if (std::abs(current_state.position - progress.initial_position) >
        target_tolerance_)
      {
        rollback_waypoints = calculate_rotation_waypoints(
          control, current_state.position, progress.initial_position);
      }
      RCLCPP_WARN(
        get_logger(),
        "Door arc failed after %zu/%zu executed waypoints; rolling the "
        "grasped door back from %.4f rad to %.4f rad using %zu waypoints.",
        progress.completed_waypoints, progress.total_waypoints,
        current_state.position, progress.initial_position,
        rollback_waypoints.size());

      for (std::size_t index = 0U; index < rollback_waypoints.size(); ++index) {
        if (!best_effort_cartesian_move(
            move_group, {rollback_waypoints[index]},
            cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            "Door arc rollback waypoint", operation_executed))
        {
          RCLCPP_ERROR(
            get_logger(),
            "Door arc rollback stopped after %zu/%zu waypoints.",
            index, rollback_waypoints.size());
          return false;
        }
        RCLCPP_INFO(
          get_logger(), "Door arc rollback waypoint %zu/%zu completed.",
          index + 1U, rollback_waypoints.size());
      }

      const auto retreat_pose = calculate_rotary_tool_pose(
        control, progress.initial_position, door_release_clearance_, false);
      if (!best_effort_cartesian_move(
          move_group, {retreat_pose}, cartesian_velocity_scale_,
          cartesian_acceleration_scale_, "Door post-rollback safety retreat",
          operation_executed))
      {
        return false;
      }
      RCLCPP_INFO(
        get_logger(),
        "Door arc rollback and outward safety retreat completed while the "
        "grasp remained attached.");
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Door arc rollback failed: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Door arc rollback failed with an unknown error.");
    }
    return false;
  }

  void best_effort_retreat(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & prepress_pose,
    bool * operation_executed = nullptr) noexcept
  {
    (void)best_effort_cartesian_move(
      move_group, {prepress_pose}, cartesian_velocity_scale_,
      cartesian_acceleration_scale_, "Safety retreat", operation_executed);
  }

  void best_effort_stow(
    MoveGroupInterface & move_group,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      move_group.stop();
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      if (!move_group.setNamedTarget(transport_named_target_)) {
        RCLCPP_ERROR(
          get_logger(), "Safety target '%s' is not configured.",
          transport_named_target_.c_str());
        return;
      }
      MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Safety stow planning failed.");
        return;
      }
      if (operation_executed) {
        *operation_executed = true;
      }
      const auto stow_code = execute_motion_bounded(
        [&move_group, &plan]() { return move_group.execute(plan); },
        std::function<bool()>(), std::chrono::seconds(60), "safety stow");
      if (stow_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Safety stow execution failed.");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Safety stow failed: %s", error.what());
    }
  }

  void best_effort_stow_named_target(
    MoveGroupInterface & move_group,
    const std::string & target_name,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      move_group.stop();
      const auto current_state = synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      if (!move_group.setNamedTarget(target_name)) {
        RCLCPP_ERROR(
          get_logger(), "Safety target '%s' is not configured.",
          target_name.c_str());
        return;
      }
      MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Safety stow planning failed for '%s'.",
          target_name.c_str());
        return;
      }
      if (operation_executed) {
        *operation_executed = true;
      }
      const auto stow_code = execute_motion_bounded(
        [&move_group, &plan]() { return move_group.execute(plan); },
        std::function<bool()>(), std::chrono::seconds(60),
        "safety stow '" + target_name + "'");
      if (stow_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(), "Safety stow execution failed for '%s'.",
          target_name.c_str());
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Safety stow failed: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  MoveGroupInterface::Plan plan_arm_pose(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    const std::string & description)
  {
    check_cancel(goal_handle);
    MoveGroupInterface::Plan plan;
    bool planned = false;
    moveit::core::RobotStatePtr start_state;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      start_state = synchronized_current_robot_state(move_group);
      move_group.setStartState(*start_state);
      move_group.setPoseTarget(target, tool_link);
      const auto planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "Drawer '%s' pose planning failed (attempt %d/%d).",
        description.c_str(), attempt, motion_planning_attempts_);
    }
    if (!planned) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not plan drawer '" + description +
              "' to its target pose after " +
              std::to_string(motion_planning_attempts_) + " attempts.");
    }
    check_cancel(goal_handle);
    return plan;
  }

  // 到达稳定性失败的结构化报告：报错值从"抛出瞬间单样本"升级为"整窗最差 +
  // 末样本逐轴误差 + 姿态轴角"，让几何缺陷（重力下垂/伸距不足/姿态偏转）
  // 可一次取证，而不是只给一个模长。
  static std::string drawer_arm_pose_inaccuracy_message(
    const std::string & description,
    const Eigen::Vector3d & actual_position,
    const Eigen::Vector3d & desired_position,
    const Eigen::Quaterniond & actual_orientation,
    const Eigen::Quaterniond & desired_orientation,
    double position_error, double orientation_error, double motion,
    double worst_position_error, double worst_orientation_error,
    double worst_motion)
  {
    const Eigen::Vector3d delta = actual_position - desired_position;
    const Eigen::Quaterniond error_quaternion =
      (desired_orientation.conjugate() * actual_orientation).normalized();
    const double error_angle =
      2.0 * std::atan2(error_quaternion.vec().norm(),
                       std::abs(error_quaternion.w()));
    std::string message = "Drawer '" + description + "' pose is inaccurate "
      "or unstable (position error " +
      std::to_string(position_error) + " m, orientation error " +
      std::to_string(orientation_error) + " rad, motion " +
      std::to_string(motion) + " m; window worst " +
      std::to_string(worst_position_error) + " m / " +
      std::to_string(worst_orientation_error) + " rad / " +
      std::to_string(worst_motion) + " m; delta (x,y,z) = (" +
      std::to_string(delta.x()) + ", " + std::to_string(delta.y()) + ", " +
      std::to_string(delta.z()) + ") m, orientation error " +
      std::to_string(error_angle) + " rad).";
    return message;
  }

  template<typename GoalHandleT>
  void verify_drawer_arm_pose_stable(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    const std::string & description)
  {
    const Eigen::Vector3d desired_position(
      target.position.x, target.position.y, target.position.z);
    Eigen::Quaterniond desired_orientation(
      target.orientation.w, target.orientation.x,
      target.orientation.y, target.orientation.z);
    desired_orientation.normalize();
    // 到达验证 = 收敛段 + 稳定带段，几何门限值（drawer_arm_pose_tolerance_ /
    // drawer_arm_orientation_tolerance_ / drawer_arm_stability_tolerance_）不变：
    // 直接 FJT 执行"末点时间到达即报控制器成功"，末段追踪/静止沉降可滞后
    // 数十到数百毫秒 —— 旧实现"窗口内任一样本超差即抛"（首样本即判）会把
    // 仍在收敛的臂误杀。2026-09-05 fix#10 (v20 取证): FJT action SUCCESS 可
    // 早于 desired 扫掠结束 ~6-10 s（v20 R action 6285.44 SUCCESS、desired
    // 继续扫到 6291.2），且到达驻留目标在旧轨迹结束前只排队不打断 —— 真实
    // 到达可比 execute 返回晚数十秒；fix#10 后本函数读真实关节状态（Edit 1），
    // 原 2.0 s 收敛余量（kArrivalSettleSeconds）在慢到达下必然误杀。现改为：
    //  1) 收敛总预算 kArrivalCompletionBudgetSeconds（60 s）：期限内任一样本
    //     进入误差带即锚定带起点开窗；整段超预算仍未进带 → stop + 逐轴证据
    //     抛错（真静态偏差/目标不可达/无源持续漂移仍照常失败）；
    //  2) 带内破带策略：进带不足 kArrivalBandMinimumSeconds（1.0 s）即破带
    //     （仍在沉降/驻留阶跃修正尾部，见 12731-12748 注释）→ 视为未收敛，
    //     关带重来（收敛预算兜底，不误杀仍在收敛的臂）；进带已足该时长才破
    //     带（dwell 过期后无源漂移等）→ stop + 照实抛错；
    //  3) 通过条件不变：整 stable_state_duration_ 窗连续带内且相对带锚点
    //     位移 ≤ drawer_arm_stability_tolerance_（与本文件 rod/pull 的
    //     stable-band 验证同构；报错仍给出逐轴证据）。
    constexpr double kArrivalCompletionBudgetSeconds = 60.0;
    constexpr double kArrivalBandMinimumSeconds = 1.0;
    using DoubleTimePoint = std::chrono::time_point<
      std::chrono::steady_clock, std::chrono::duration<double>>;
    const auto completion_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(kArrivalCompletionBudgetSeconds);
    bool in_band = false;
    Eigen::Vector3d band_anchor = Eigen::Vector3d::Zero();
    DoubleTimePoint band_started{};
    DoubleTimePoint band_deadline{};
    double worst_position_error = 0.0;
    double worst_orientation_error = 0.0;
    double worst_motion = 0.0;
    double last_position_error = 0.0;
    double last_orientation_error = 0.0;
    Eigen::Vector3d last_actual_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond last_actual_orientation =
      Eigen::Quaterniond::Identity();
    for (;;) {
      check_cancel(goal_handle);
      const auto state = synchronized_current_robot_state(move_group);
      const auto * link_model = state->getLinkModel(tool_link);
      if (link_model == nullptr) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Drawer pose verification cannot find tool link '" +
                tool_link + "'.");
      }
      const Eigen::Isometry3d & actual =
        state->getGlobalLinkTransform(link_model);
      last_actual_position = actual.translation();
      Eigen::Quaterniond actual_orientation(actual.rotation());
      actual_orientation.normalize();
      last_actual_orientation = actual_orientation;
      const double position_error =
        (last_actual_position - desired_position).norm();
      const double quaternion_dot = std::clamp(
        std::abs(actual_orientation.dot(desired_orientation)), 0.0, 1.0);
      const double orientation_error = 2.0 * std::acos(quaternion_dot);
      last_position_error = position_error;
      last_orientation_error = orientation_error;
      const bool sample_in_band =
        position_error <= drawer_arm_pose_tolerance_ &&
        orientation_error <= drawer_arm_orientation_tolerance_;
      if (in_band) {
        const double motion = (last_actual_position - band_anchor).norm();
        worst_position_error = std::max(worst_position_error, position_error);
        worst_orientation_error = std::max(
          worst_orientation_error, orientation_error);
        worst_motion = std::max(worst_motion, motion);
        if (sample_in_band && motion <= drawer_arm_stability_tolerance_ &&
            std::chrono::steady_clock::now() >= band_deadline)
        {
          RCLCPP_INFO(
            get_logger(),
            "Drawer '%s' pose verified stable: position <= %.4f m, "
            "orientation <= %.4f rad, motion <= %.4f m.",
            description.c_str(), worst_position_error,
            worst_orientation_error, worst_motion);
          return;
        }
        if (!sample_in_band || motion > drawer_arm_stability_tolerance_) {
          // 破带（出带或位移超稳）：进带已足 kArrivalBandMinimumSeconds 才判
          // 失败（dwell 过期后无源漂移等真缺陷，stop + 照实抛错留证）。
          const double held_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - band_started).count();
          if (held_seconds >= kArrivalBandMinimumSeconds) {
            stop_active_motion();
            throw OperationError(
                    PressCabinetButton::Result::EXECUTION_FAILED,
                    drawer_arm_pose_inaccuracy_message(
                      description, last_actual_position, desired_position,
                      last_actual_orientation, desired_orientation,
                      last_position_error, last_orientation_error, motion,
                      worst_position_error, worst_orientation_error,
                      worst_motion));
          }
          // 进带不足最短保持即破带：仍在不稳定沉降/驻留修正尾部 → 视为未
          // 收敛，关带重来（收敛预算兜底，不误杀仍在收敛的臂）。
          in_band = false;
        }
      } else if (sample_in_band) {
        // 进入误差带：锚定带起点，启动稳定带窗口。
        in_band = true;
        band_anchor = last_actual_position;
        band_started = std::chrono::steady_clock::now();
        band_deadline = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(stable_state_duration_);
      } else if (std::chrono::steady_clock::now() >= completion_deadline) {
        // 收敛预算耗尽仍未进带（臂未到达/目标不可达/无源持续漂移）。
        stop_active_motion();
        throw OperationError(
                PressCabinetButton::Result::EXECUTION_FAILED,
                drawer_arm_pose_inaccuracy_message(
                  description, last_actual_position, desired_position,
                  last_actual_orientation, desired_orientation,
                  last_position_error, last_orientation_error, 0.0,
                  worst_position_error, worst_orientation_error,
                  worst_motion));
      }
      interruptible_hold(goal_handle, 0.05);
    }
  }

  template<typename GoalHandleT>
  void plan_and_execute_bimanual_poses(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::string & left_group_name,
    const std::string & right_group_name,
    const geometry_msgs::msg::Pose & left_target,
    const geometry_msgs::msg::Pose & right_target,
    const std::vector<double> & left_branch_seed,
    const std::vector<double> & right_branch_seed,
    bool * operation_executed,
    const std::string & description,
    // 2026-09-05 fix#10 (v20 取证): true = 正常返回时把到达驻留（dwell）留在
    // 控制器上，供调用方在 active 驻留指令下做物理测量 —— 无源臂以 ~mm/0.3 s
    // 漂移，会污染 2.5 mm 收敛带（self-center 的 measure_side 注释 6475-6481
    // 要求测量必须在 dwell 保持中进行；v20 在驻留撤销后测量才得 6.6 mm 假散
    // 布）。抛错/取消路径一律照常撤销（catch 兜底）；旧驻留由后续执行轨迹
    // 自然接替或排队（FJT 对运行中轨迹不打断），否则驻留终到自行结束，不留
    // 泄漏。
    bool leave_arrival_dwell = false)
  {
    // P3-8: ready 位姿若带选定的分支种子（select_drawer_branch_seed），就用
    // setJointValueTarget 把目标钉到该 IK 分支的精确关节构型，使后续整条
    // Cartesian 链条（解锁→支撑→预抓→抓取→拽拉）都可规划；否则回退普通位姿
    // 目标，让 OMPL 任意采样分支（run7 的 r5=+2.96 即由此随机产生）。
    const auto plan_arm_to_ready = [&](
        MoveGroupInterface & group, const geometry_msgs::msg::Pose & target,
        const std::string & tool_link, const std::string & group_name,
        const std::vector<double> & branch_seed, const std::string & side) {
      check_cancel(goal_handle);
      MoveGroupInterface::Plan plan;
      bool planned = false;
      for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
        check_cancel(goal_handle);
        const auto current_state = synchronized_current_robot_state(group);
        group.setStartState(*current_state);
        bool branch_ok = false;
        if (!branch_seed.empty()) {
          const auto robot_model = group.getRobotModel();
          const auto * joint_model_group =
            robot_model == nullptr ? nullptr :
            robot_model->getJointModelGroup(group_name);
          if (joint_model_group != nullptr) {
            const auto & variable_names = joint_model_group->getVariableNames();
            if (variable_names.size() == branch_seed.size()) {
              moveit::core::RobotState goal_state(*current_state);
              goal_state.setVariablePositions(variable_names, branch_seed);
              goal_state.update();
              if (goal_state.satisfiesBounds(joint_model_group)) {
                branch_ok = group.setJointValueTarget(goal_state);
              }
            }
          }
        }
        moveit::core::MoveItErrorCode planning_result =
          moveit::core::MoveItErrorCode::FAILURE;
        if (branch_ok) {
          planning_result = group.plan(plan);
          group.clearPoseTargets();
          if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
            planned = true;
            break;
          }
        }
        group.setStartState(*current_state);
        group.setPoseTarget(target, tool_link);
        const auto pose_planning_result = group.plan(plan);
        group.clearPoseTargets();
        if (pose_planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
          planned = true;
          break;
        }
        RCLCPP_WARN(
          get_logger(),
          "Drawer ready planning to '%s' failed (attempt %d/%d, branch=%d "
          "pose=%d).",
          side.c_str(), attempt, motion_planning_attempts_,
          branch_ok ? planning_result.val : -1,
          pose_planning_result.val);
      }
      if (!planned) {
        throw OperationError(
                PressCabinetButton::Result::PLANNING_FAILED,
                "MoveIt could not plan drawer '" + description + "' " + side +
                " arm to its ready pose after " +
                std::to_string(motion_planning_attempts_) + " attempts.");
      }
      check_cancel(goal_handle);
      return plan;
    };

    const auto left_plan = plan_arm_to_ready(
      *left_group, left_target, drawer_left_tool_.contact_tool_link,
      left_group_name, left_branch_seed, "left");
    const auto right_plan = plan_arm_to_ready(
      *right_group, right_target, drawer_right_tool_.contact_tool_link,
      right_group_name, right_branch_seed, "right");
    const auto execution = execute_bimanual_trajectories_bounded(
      left_group, left_plan.trajectory_,
      right_group, right_plan.trajectory_,
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), description);
    if (execution.left_code != moveit::core::MoveItErrorCode::SUCCESS ||
      execution.right_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt failed to execute bimanual '" + description +
              "' (left=" + std::to_string(execution.left_code.val) +
              " right=" + std::to_string(execution.right_code.val) + ").");
    }
    if (operation_executed) {
      *operation_executed = true;
    }
    // 到达驻留（dwell）：ros2_control 轨迹控制器对已成功的目标不再主动维持
    // 目标位姿（指令跟随实测、无恢复力）——cap2 实测到达判定成功后右臂仍以
    // ~mm/0.3 s 持续漂移，稳定带 verify（settle 2 s + 0.3 s 带、0.003 m 门）
    // 在无源保持下不可靠。解法：验证窗口内为每条臂保持一条 active 驻留轨迹
    // （两点同钉执行终态、全程持续指令目标构型），控制器在驻留终点才判定
    // 容差（0.002 rad/关节），verify 全程处于 active 指令下；测完即撤销，
    // 几何门禁与 verify 时序语义均不变。驻留发送失败不判失败——verify 仍以
    // 静止门禁为准，漂移会被照实抓出。
    // 驻留 t=0 点把关节从执行终点残差"阶跃式"钉回精确终态：若 approach
    // 执行结束时尚有容差残差，控制器接受驻留目标即发出阶跃指令，关节以
    // ~1.5-2 s 的伺服衰减收敛（cap2 实测工具姿态修正 ~0.6-1.2°）。稳定带
    // verify 必须在修正完全衰减后才开窗，否则 0.3 s 窗落在衰减尾部 → 阈值
    // 边缘越限（v6 motion 3.17/3.0 mm、v7 orientation 20.8/20.0 mrad 均
    // 为此类）。该等待可取消，只在驻留 active 期间生效。
    constexpr double kArrivalCorrectionSettleSeconds = 3.0;
    // 驻留时长须覆盖"沉降等待(3.0 s) + 左臂 verify(≤~2.3 s) + 右臂 verify
    // (≤~2.3 s)"全程，保证 verify 尾部仍在 active 驻留指令下；测完即撤销。
    constexpr double kDwellHoldSeconds = 12.0;
    auto send_dwell = [this, &goal_handle](
        const moveit_msgs::msg::RobotTrajectory & executed,
        const std::string & side) -> bool {
      const bool left_side = side == "left";
      rclcpp_action::Client<control_msgs::action::FollowJointTrajectory> *
        client = left_side ? left_fjt_client_.get() : right_fjt_client_.get();
      auto & slot = left_side ? bimanual_controller_handles_->left :
        bimanual_controller_handles_->right;
      const auto & joint_trajectory = executed.joint_trajectory;
      if (joint_trajectory.points.empty()) {
        return false;
      }
      if (!client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(
          get_logger(),
          "Bimanual %s-arm arrival-dwell controller unavailable.",
          side.c_str());
        return false;
      }
      auto goal = control_msgs::action::FollowJointTrajectory::Goal();
      auto & trajectory = goal.trajectory;
      trajectory.joint_names = joint_trajectory.joint_names;
      const std::size_t joint_count = trajectory.joint_names.size();
      const auto & final_point = joint_trajectory.points.back();
      trajectory.points.resize(2);
      for (std::size_t point_index = 0; point_index < 2; ++point_index) {
        auto & point = trajectory.points[point_index];
        point.positions = final_point.positions;
        point.velocities.assign(joint_count, 0.0);
        point.accelerations.assign(joint_count, 0.0);
        point.time_from_start.sec = static_cast<int32_t>(
          point_index == 0 ? 0 : kDwellHoldSeconds);
        point.time_from_start.nanosec = 0;
      }
      goal.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);
      goal.goal_tolerance.resize(joint_count);
      for (std::size_t index = 0U; index < joint_count; ++index) {
        auto & tolerance = goal.goal_tolerance[index];
        tolerance.name = trajectory.joint_names[index];
        tolerance.position = 0.002;
        tolerance.velocity = 0.0;
        tolerance.acceleration = 0.0;
      }
      try {
        check_cancel(goal_handle);
        const auto send_future = client->async_send_goal(goal);
        const auto send_deadline = std::chrono::steady_clock::now() +
          std::chrono::seconds(5);
        while (send_future.wait_for(50ms) != std::future_status::ready) {
          check_cancel(goal_handle);
          if (std::chrono::steady_clock::now() >= send_deadline) {
            RCLCPP_ERROR(
              get_logger(),
              "Bimanual %s-arm arrival-dwell goal was not accepted in time.",
              side.c_str());
            return false;
          }
        }
        const auto accepted = send_future.get();
        if (!accepted) {
          RCLCPP_ERROR(
            get_logger(),
            "Bimanual %s-arm controller rejected the arrival-dwell goal.",
            side.c_str());
          return false;
        }
        {
          std::lock_guard<std::mutex> lock(bimanual_controller_handles_->mutex);
          slot = accepted;
        }
        return true;
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Bimanual %s-arm arrival-dwell was not armed: %s",
          side.c_str(), error.what());
        return false;
      }
    };
    auto cancel_dwells = [this]() noexcept {
      std::lock_guard<std::mutex> lock(bimanual_controller_handles_->mutex);
      if (bimanual_controller_handles_->left) {
        try {
          left_fjt_client_->async_cancel_goal(
            bimanual_controller_handles_->left);
        } catch (const std::exception &) {
        }
        bimanual_controller_handles_->left.reset();
      }
      if (bimanual_controller_handles_->right) {
        try {
          right_fjt_client_->async_cancel_goal(
            bimanual_controller_handles_->right);
        } catch (const std::exception &) {
        }
        bimanual_controller_handles_->right.reset();
      }
    };
    const bool left_dwell = send_dwell(left_plan.trajectory_, "left");
    const bool right_dwell = send_dwell(right_plan.trajectory_, "right");
    if (left_dwell || right_dwell) {
      RCLCPP_INFO(
        get_logger(),
        "Bimanual '%s' arrival dwell active (left=%d right=%d) for pose "
        "verification.",
        description.c_str(), left_dwell ? 1 : 0, right_dwell ? 1 : 0);
    }
    try {
      // 沉降等待：驻留目标 t=0 的阶跃修正先衰减完（可取消），再开稳定带窗，
      // 避免 verify 的 0.3 s 窗落在伺服衰减尾部导致边缘越限。窗内门限语义
      // 不变：若等待后仍不稳定（修正过大/漂移），verify 照实抓出。
      interruptible_hold(goal_handle, kArrivalCorrectionSettleSeconds);
      verify_drawer_arm_pose_stable(
        *left_group, goal_handle, left_target,
        drawer_left_tool_.contact_tool_link, description + " left");
      verify_drawer_arm_pose_stable(
        *right_group, goal_handle, right_target,
        drawer_right_tool_.contact_tool_link, description + " right");
    } catch (...) {
      // verify 抛错路径自身会 stop_active_motion()（其撤销驻留目标）；这里再
      // 兜底一次，保证任何退出路径都不把驻留目标留在控制器上。
      cancel_dwells();
      throw;
    }
    if (!leave_arrival_dwell) {
      cancel_dwells();
    } else {
      // 调用方（self-center 测量）需要驻留保持：handle 留在 bimanual_
      // controller_handles_ 槽位，驻留轨迹终到后控制器自行结束；下一个整臂
      // 执行轨迹会按 FJT 语义接替/排队（见签名注释）。不在此撤销。
      RCLCPP_INFO(
        get_logger(),
        "Bimanual '%s' arrival dwells left armed for the caller (measure "
        "runs under active hold).",
        description.c_str());
    }
    check_cancel(goal_handle);
  }

  // Best-effort concurrent pose motion for both arms, used by the drawer
  // safety recovery.  No goal handle (deadline-only), never throws.
  bool best_effort_bimanual_pose_move(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const geometry_msgs::msg::Pose & left_target,
    const geometry_msgs::msg::Pose & right_target,
    const std::string & description) noexcept
  {
    if (!left_group || !right_group) {
      return false;
    }
    bool ok = false;
    try {
      MoveGroupInterface::Plan left_plan;
      MoveGroupInterface::Plan right_plan;
      bool left_planned = false;
      bool right_planned = false;
      for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
        if (!left_planned) {
          const auto state = synchronized_current_robot_state(*left_group);
          left_group->setStartState(*state);
          left_group->setPoseTarget(
            left_target, drawer_left_tool_.contact_tool_link);
          if (left_group->plan(left_plan) ==
            moveit::core::MoveItErrorCode::SUCCESS)
          {
            left_group->clearPoseTargets();
            left_planned = true;
          } else {
            left_group->clearPoseTargets();
          }
        }
        if (!right_planned) {
          const auto state = synchronized_current_robot_state(*right_group);
          right_group->setStartState(*state);
          right_group->setPoseTarget(
            right_target, drawer_right_tool_.contact_tool_link);
          if (right_group->plan(right_plan) ==
            moveit::core::MoveItErrorCode::SUCCESS)
          {
            right_group->clearPoseTargets();
            right_planned = true;
          } else {
            right_group->clearPoseTargets();
          }
        }
        if (left_planned && right_planned) {
          break;
        }
      }
      if (left_planned && right_planned) {
        const auto execution = execute_bimanual_trajectories_bounded(
          left_group, left_plan.trajectory_,
          right_group, right_plan.trajectory_,
          std::function<bool()>(), std::chrono::seconds(60), description);
        ok = execution.left_code == moveit::core::MoveItErrorCode::SUCCESS &&
          execution.right_code == moveit::core::MoveItErrorCode::SUCCESS;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Best-effort bimanual %s failed: %s",
        description.c_str(), error.what());
    }
    return ok;
  }

  void best_effort_bimanual_retreat_and_stow(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const ButtonSpec & control,
    bool * operation_executed = nullptr) noexcept
  {
    if (!left_group || !right_group) {
      return;
    }
    try {
      const auto state = button_snapshot(control);
      const double position = state.received ?
        state.position : control.drawer_closed_position;
      best_effort_bimanual_pose_move(
        left_group, right_group,
        calculate_drawer_side_tool_pose(
          control, DrawerSide::LEFT, position, prepress_distance_),
        calculate_drawer_side_tool_pose(
          control, DrawerSide::RIGHT, position, prepress_distance_),
        "drawer retreat");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Drawer best-effort retreat failed: %s", error.what());
    }
    best_effort_stow_named_target(
      *left_group, drawer_left_tool_.transport_named_target,
      operation_executed);
    best_effort_stow_named_target(
      *right_group, drawer_right_tool_.transport_named_target,
      operation_executed);
  }

  template<typename GoalHandleT>
  void navigate_to_staging_pose(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & staging_pose)
  {
    if (!navigation_client_->wait_for_action_server(
        std::chrono::duration<double>(system_wait_timeout_)))
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Nav2 NavigateToPose action server is unavailable.");
    }
    set_navigation_mode(goal_handle, true);

    NavigateToPose::Goal navigation_goal;
    navigation_goal.pose = staging_pose;
    navigation_goal.pose.header.stamp = get_clock()->now();
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    const auto request_abandoned = std::make_shared<std::atomic<bool>>(false);
    const auto navigation_started =
      std::make_shared<std::atomic<bool>>(false);
    const auto near_distance_since_ns =
      std::make_shared<std::atomic<std::int64_t>>(0);
    const auto takeover_requested =
      std::make_shared<std::atomic<bool>>(false);
    const auto stalled_takeover_requested =
      std::make_shared<std::atomic<bool>>(false);
    tf2::Quaternion target_navigation_rotation;
    tf2::fromMsg(
      staging_pose.pose.orientation, target_navigation_rotation);
    const double target_navigation_yaw =
      tf2::getYaw(target_navigation_rotation);
    const double target_navigation_x = staging_pose.pose.position.x;
    const double target_navigation_y = staging_pose.pose.position.y;
    const std::string target_navigation_frame =
      staging_pose.header.frame_id;
    options.goal_response_callback =
      [this, request_abandoned,
        weak_goal = std::weak_ptr<GoalHandleT>(goal_handle)](
      NavigationGoalHandle::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          return;
        }
        const auto operation_goal = weak_goal.lock();
        const bool operation_should_stop =
          !operation_goal || goal_should_stop(operation_goal);
        bool must_cancel;
        {
          std::lock_guard<std::mutex> lock(navigation_mutex_);
          must_cancel = request_abandoned->load() ||
            operation_should_stop;
          if (!must_cancel) {
            active_navigation_goal_ = goal_handle;
          }
        }
        if (must_cancel) {
          navigation_client_->async_cancel_goal(goal_handle);
        }
      };
    options.feedback_callback =
      [this, weak_goal = std::weak_ptr<GoalHandleT>(goal_handle),
        navigation_started, near_distance_since_ns, takeover_requested,
        stalled_takeover_requested, target_navigation_yaw,
        target_navigation_x, target_navigation_y, target_navigation_frame](
      NavigationGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        const double distance_remaining = feedback->distance_remaining;
        const double current_navigation_x =
          feedback->current_pose.pose.position.x;
        const double current_navigation_y =
          feedback->current_pose.pose.position.y;
        tf2::Quaternion current_navigation_rotation;
        tf2::fromMsg(
          feedback->current_pose.pose.orientation,
          current_navigation_rotation);
        const double quaternion_norm =
          current_navigation_rotation.length2();
        const bool feedback_pose_valid =
          !target_navigation_frame.empty() &&
          feedback->current_pose.header.frame_id == target_navigation_frame &&
          std::isfinite(target_navigation_x) &&
          std::isfinite(target_navigation_y) &&
          std::isfinite(current_navigation_x) &&
          std::isfinite(current_navigation_y) &&
          std::isfinite(quaternion_norm) && quaternion_norm > 1.0e-12;
        double direct_goal_distance =
          std::numeric_limits<double>::quiet_NaN();
        double yaw_error = std::numeric_limits<double>::quiet_NaN();
        if (feedback_pose_valid) {
          direct_goal_distance = std::hypot(
            target_navigation_x - current_navigation_x,
            target_navigation_y - current_navigation_y);
          current_navigation_rotation.normalize();
          const double current_navigation_yaw =
            tf2::getYaw(current_navigation_rotation);
          yaw_error = std::atan2(
            std::sin(target_navigation_yaw - current_navigation_yaw),
            std::cos(target_navigation_yaw - current_navigation_yaw));
        }
        const auto feedback_now = std::chrono::steady_clock::now();
        if (std::isfinite(direct_goal_distance) && std::isfinite(yaw_error)) {
          if (direct_goal_distance > navigation_takeover_distance_) {
            navigation_started->store(true);
            near_distance_since_ns->store(0);
            takeover_requested->store(false);
            stalled_takeover_requested->store(false);
          } else {
            const auto now_ns = std::chrono::duration_cast<
              std::chrono::nanoseconds>(
              feedback_now.time_since_epoch()).count();
            auto first_near_ns = near_distance_since_ns->load();
            if (first_near_ns == 0) {
              if (near_distance_since_ns->compare_exchange_strong(
                  first_near_ns, now_ns))
              {
                first_near_ns = now_ns;
              }
            }
            const double near_duration = static_cast<double>(
              now_ns - first_near_ns) * 1.0e-9;
            const bool yaw_ready = navigation_started->load() &&
              std::abs(yaw_error) <= navigation_takeover_yaw_tolerance_;
            const bool yaw_stalled =
              near_duration >= navigation_takeover_stall_timeout_;
            if (yaw_ready || yaw_stalled) {
              stalled_takeover_requested->store(yaw_stalled && !yaw_ready);
              takeover_requested->store(true);
            }
          }
        } else {
          near_distance_since_ns->store(0);
          takeover_requested->store(false);
          stalled_takeover_requested->store(false);
        }

        {
          std::lock_guard<std::mutex> lock(navigation_feedback_mutex_);
          if (feedback_now - last_navigation_feedback_ < 500ms) {
            return;
          }
          last_navigation_feedback_ = feedback_now;
        }
        if (const auto operation_goal = weak_goal.lock()) {
          publish_feedback(
            operation_goal,
            PressCabinetButton::Feedback::NAVIGATING,
            0.12F,
            "Nav2 distance remaining: " +
            (std::isfinite(distance_remaining) ?
            std::to_string(distance_remaining) : "unavailable") +
            " m, direct goal distance: " +
            (std::isfinite(direct_goal_distance) ?
            std::to_string(direct_goal_distance) : "unavailable") +
            " m, yaw: " +
            (std::isfinite(yaw_error) ?
            std::to_string(std::abs(yaw_error)) : "unavailable") +
            " rad." +
            (stalled_takeover_requested->load() ?
            " Safe-distance yaw stall detected; taking over." : ""));
        }
      };
    NavigationGoalHandle::SharedPtr navigation_goal_handle;
    const auto acceptance_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    int rejected_attempts = 0;
    while (!navigation_goal_handle) {
      check_cancel(goal_handle);
      navigation_goal.pose.header.stamp = get_clock()->now();
      auto goal_future = navigation_client_->async_send_goal(
        navigation_goal, options);
      try {
        wait_for_future(
          goal_future,
          goal_handle,
          system_wait_timeout_,
          PressCabinetButton::Result::NAVIGATION_FAILED,
          "Nav2 did not accept the staging goal in time.");
      } catch (...) {
        request_abandoned->store(true);
        cancel_active_navigation();
        throw;
      }
      navigation_goal_handle = goal_future.get();
      if (navigation_goal_handle) {
        break;
      }
      ++rejected_attempts;
      if (std::chrono::steady_clock::now() >= acceptance_deadline) {
        throw OperationError(
                PressCabinetButton::Result::NAVIGATION_FAILED,
                "Nav2 rejected the cabinet staging goal for " +
                std::to_string(rejected_attempts) +
                " consecutive startup attempts.");
      }
      RCLCPP_WARN(
        get_logger(),
        "Nav2 rejected staging goal attempt %d while its lifecycle may "
        "still be activating; retrying.",
        rejected_attempts);
      std::this_thread::sleep_for(250ms);
    }
    bool must_cancel_navigation = false;
    const bool operation_should_stop = goal_should_stop(goal_handle);
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      must_cancel_navigation = operation_should_stop;
      if (!must_cancel_navigation) {
        active_navigation_goal_ = navigation_goal_handle;
      }
    }
    if (must_cancel_navigation) {
      navigation_client_->async_cancel_goal(navigation_goal_handle);
      check_cancel(goal_handle);
    }

    auto result_future = navigation_client_->async_get_result(
      navigation_goal_handle);
    const auto verified_direct_goal_distance = [this, &staging_pose]() {
        try {
          const auto current_pose = transform_buffer_->lookupTransform(
            staging_pose.header.frame_id, navigation_base_frame_,
            tf2::TimePointZero,
            200ms);
          const double current_x =
            current_pose.transform.translation.x;
          const double current_y =
            current_pose.transform.translation.y;
          if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
            !std::isfinite(staging_pose.pose.position.x) ||
            !std::isfinite(staging_pose.pose.position.y))
          {
            return std::numeric_limits<double>::quiet_NaN();
          }
          return std::hypot(
            staging_pose.pose.position.x - current_x,
            staging_pose.pose.position.y - current_y);
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Could not verify the Nav2 takeover distance: %s", error.what());
          return std::numeric_limits<double>::quiet_NaN();
        }
      };
    bool precision_takeover = false;
    const auto navigation_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(navigation_timeout_);
    while (result_future.wait_for(100ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (takeover_requested->load()) {
        const double verified_distance = verified_direct_goal_distance();
        if (!std::isfinite(verified_distance) ||
          verified_distance > navigation_takeover_distance_)
        {
          takeover_requested->store(false);
          stalled_takeover_requested->store(false);
          near_distance_since_ns->store(0);
          continue;
        }
        precision_takeover = true;
        auto cancel_future = navigation_client_->async_cancel_goal(
          navigation_goal_handle);
        wait_for_future(
          cancel_future,
          goal_handle,
          system_wait_timeout_,
          PressCabinetButton::Result::NAVIGATION_FAILED,
          "Nav2 did not acknowledge the precision-docking takeover.");
        break;
      }
      if (std::chrono::steady_clock::now() >= navigation_deadline) {
        auto cancel_future = navigation_client_->async_cancel_goal(
          navigation_goal_handle);
        wait_for_future(
          cancel_future,
          goal_handle,
          system_wait_timeout_,
          PressCabinetButton::Result::NAVIGATION_FAILED,
          "Nav2 did not acknowledge cancellation after its staging timeout.");
        request_navigation_mode_without_wait(false);
        publish_manual_base_stop();
        throw OperationError(
                PressCabinetButton::Result::NAVIGATION_FAILED,
                "Nav2 timed out while driving to the cabinet.");
      }
    }
    if (precision_takeover) {
      wait_for_future(
        result_future,
        goal_handle,
        system_wait_timeout_,
        PressCabinetButton::Result::NAVIGATION_FAILED,
        "Nav2 did not stop for the precision-docking takeover.");
    }
    const auto wrapped_result = result_future.get();
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      if (active_navigation_goal_ == navigation_goal_handle) {
        active_navigation_goal_.reset();
      }
    }
    check_cancel(goal_handle);
    if (precision_takeover &&
      (wrapped_result.code == rclcpp_action::ResultCode::CANCELED ||
      wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED))
    {
      const double stopped_distance = verified_direct_goal_distance();
      if (!std::isfinite(stopped_distance) ||
        stopped_distance >
        navigation_takeover_distance_ + docking_position_tolerance_)
      {
        throw OperationError(
                PressCabinetButton::Result::NAVIGATION_FAILED,
                "Nav2 stopped outside the verified precision-docking "
                "takeover distance.");
      }
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::NAVIGATING,
        0.16F,
        stalled_takeover_requested->load() ?
        "Nav2 reached the safe staging distance but its terminal yaw "
        "stalled; switching to odometry docking." :
        "Nav2 coarse staging complete; switching to odometry docking.");
      return;
    }
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      throw OperationError(
              PressCabinetButton::Result::NAVIGATION_FAILED,
              "Nav2 did not reach the cabinet staging pose.");
    }
  }

  template<typename GoalHandleT>
  static std::uint8_t docking_feedback_phase(
    const std::shared_ptr<GoalHandleT> &)
  {
    return PressCabinetButton::Feedback::NAVIGATING;
  }

  static std::uint8_t docking_feedback_phase(
    const std::shared_ptr<OperateGoalHandle> &)
  {
    return OperateCabinetControl::Feedback::DOCKING;
  }

  template<typename GoalHandleT>
  void verify_staging_pose_before_arm_motion(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target)
  {
    check_cancel(goal_handle);
    publish_manual_base_stop();

    geometry_msgs::msg::TransformStamped current_transform;
    try {
      current_transform = transform_buffer_->lookupTransform(
        planning_frame_, docking_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(system_wait_timeout_));
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cannot verify the stopped base pose before arm motion: " +
              std::string(error.what()));
    }

    tf2::Quaternion target_rotation;
    tf2::Quaternion current_rotation;
    tf2::fromMsg(target.pose.orientation, target_rotation);
    tf2::fromMsg(
      current_transform.transform.rotation, current_rotation);
    const double current_x = current_transform.transform.translation.x;
    const double current_y = current_transform.transform.translation.y;
    if (target.header.frame_id != planning_frame_ ||
      !std::isfinite(target.pose.position.x) ||
      !std::isfinite(target.pose.position.y) ||
      !std::isfinite(current_x) || !std::isfinite(current_y) ||
      !std::isfinite(target_rotation.length2()) ||
      target_rotation.length2() <= 1.0e-12 ||
      !std::isfinite(current_rotation.length2()) ||
      current_rotation.length2() <= 1.0e-12)
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The stopped base pose is invalid before arm motion.");
    }
    target_rotation.normalize();
    current_rotation.normalize();
    const double target_body_yaw = navigation_yaw_in_model_frame(
      tf2::getYaw(target_rotation), navigation_velocity_yaw_offset_);
    const double current_yaw = tf2::getYaw(current_rotation);
    const double position_error = std::hypot(
      target.pose.position.x - current_x,
      target.pose.position.y - current_y);
    const double yaw_error = std::atan2(
      std::sin(target_body_yaw - current_yaw),
      std::cos(target_body_yaw - current_yaw));
    if (!staging_pose_error_is_safe(
        position_error, yaw_error, docking_position_tolerance_,
        docking_yaw_tolerance_))
    {
      throw OperationError(
              PressCabinetButton::Result::NAVIGATION_FAILED,
              "The base moved outside the verified cabinet station while "
              "the planning scene settled (position error " +
              std::to_string(position_error) + " m, yaw error " +
              std::to_string(std::abs(yaw_error)) +
              " rad); arm motion was blocked.");
    }
  }

  template<typename GoalHandleT>
  void dock_to_staging_pose(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target)
  {
    if (target.header.frame_id != planning_frame_ ||
      !std::isfinite(target.pose.position.x) ||
      !std::isfinite(target.pose.position.y))
    {
      publish_manual_base_stop();
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Precision docking requires a finite target in planning frame '" +
              planning_frame_ + "'.");
    }
    tf2::Quaternion target_rotation;
    tf2::fromMsg(target.pose.orientation, target_rotation);
    if (!std::isfinite(target_rotation.length2()) ||
      target_rotation.length2() <= 1.0e-12)
    {
      publish_manual_base_stop();
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Precision docking received an invalid target orientation.");
    }
    target_rotation.normalize();
    // The base router transforms navigation velocity components with
    // R(+offset). The physical docking frame yaw is therefore the desired
    // navigation-base yaw minus that same shared frame offset.
    const double target_body_yaw = navigation_yaw_in_model_frame(
      tf2::getYaw(target_rotation), navigation_velocity_yaw_offset_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(docking_timeout_);
    auto last_feedback = std::chrono::steady_clock::time_point{};
    int settled_cycles = 0;
    bool takeover_distance_verified = false;

    try {
      while (std::chrono::steady_clock::now() < deadline) {
        check_cancel(goal_handle);
        geometry_msgs::msg::TransformStamped current_transform;
        try {
          current_transform = transform_buffer_->lookupTransform(
            planning_frame_, docking_base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Could not read the robot odometry for precision "
                  "docking: " + std::string(error.what()));
        }

        tf2::Quaternion current_rotation;
        tf2::fromMsg(
          current_transform.transform.rotation, current_rotation);
        const double current_x = current_transform.transform.translation.x;
        const double current_y = current_transform.transform.translation.y;
        if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
          !std::isfinite(current_rotation.length2()) ||
          current_rotation.length2() <= 1.0e-12)
        {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Robot odometry is invalid during precision docking.");
        }
        current_rotation.normalize();
        const double current_yaw = tf2::getYaw(current_rotation);
        const double error_x = target.pose.position.x - current_x;
        const double error_y = target.pose.position.y - current_y;
        const double position_error = std::hypot(error_x, error_y);
        const double yaw_error = std::atan2(
          std::sin(target_body_yaw - current_yaw),
          std::cos(target_body_yaw - current_yaw));
        if (!std::isfinite(position_error) || !std::isfinite(yaw_error)) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Precision docking could not compute a finite pose error.");
        }
        if (!takeover_distance_verified) {
          if (position_error >
            navigation_takeover_distance_ + docking_position_tolerance_)
          {
            throw OperationError(
                    PressCabinetButton::Result::NAVIGATION_FAILED,
                    "The coarse navigation result is " +
                    std::to_string(position_error) +
                    " m from the configured control station, outside the "
                    "precision-docking takeover distance of " +
                    std::to_string(navigation_takeover_distance_) + " m.");
          }
          takeover_distance_verified = true;
        }

        if (staging_pose_error_is_safe(
            position_error, yaw_error, docking_position_tolerance_,
            docking_yaw_tolerance_))
        {
          ++settled_cycles;
          publish_manual_base_stop();
          if (settled_cycles >= 5) {
            return;
          }
        } else {
          settled_cycles = 0;
          const double world_speed = std::min(
            docking_max_linear_speed_,
            docking_linear_gain_ * position_error);
          const double world_velocity_x = position_error > 1.0e-9 ?
            world_speed * error_x / position_error : 0.0;
          const double world_velocity_y = position_error > 1.0e-9 ?
            world_speed * error_y / position_error : 0.0;
          geometry_msgs::msg::Twist command;
          command.linear.x =
            std::cos(current_yaw) * world_velocity_x +
            std::sin(current_yaw) * world_velocity_y;
          command.linear.y =
            -std::sin(current_yaw) * world_velocity_x +
            std::cos(current_yaw) * world_velocity_y;
          command.angular.z = std::clamp(
            docking_angular_gain_ * yaw_error,
            -docking_max_angular_speed_,
            docking_max_angular_speed_);
          manual_base_publisher_->publish(command);
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_feedback >= 500ms) {
          last_feedback = now;
          publish_feedback(
            goal_handle,
            docking_feedback_phase(goal_handle),
            0.20F,
            "Precision docking error: " +
            std::to_string(position_error) + " m, yaw: " +
            std::to_string(std::abs(yaw_error)) + " rad.");
        }
        std::this_thread::sleep_for(50ms);
      }
    } catch (...) {
      publish_manual_base_stop();
      throw;
    }

    publish_manual_base_stop();
    throw OperationError(
            PressCabinetButton::Result::NAVIGATION_FAILED,
            "The robot could not reach the odometry staging pose within "
            "the precision-docking timeout.");
  }

  void publish_manual_base_stop() noexcept
  {
    try {
      manual_base_publisher_->publish(geometry_msgs::msg::Twist{});
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to stop precision docking: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  void set_navigation_mode(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    bool enabled)
  {
    if (!navigation_mode_client_->wait_for_service(
        std::chrono::duration<double>(system_wait_timeout_)))
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Base navigation-mode service is unavailable.");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = navigation_mode_client_->async_send_request(request);
    wait_for_future(
      future,
      goal_handle,
      system_wait_timeout_,
      PressCabinetButton::Result::NOT_READY,
      "Timed out while switching the base command mode.");
    const auto response = future.get();
    if (!response->success) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Failed to switch the base command mode: " +
              response->message);
    }
  }

  template<typename FutureT, typename GoalHandleT>
  void wait_for_future(
    FutureT & future,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    double timeout_seconds,
    std::uint8_t error_code,
    const std::string & timeout_message)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (future.wait_for(100ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw OperationError(error_code, timeout_message);
      }
    }
  }

  template<typename GoalHandleT>
  void acquire_operation_lease(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    ActiveGoalType goal_type,
    bool resources_required)
  {
    operation_lease_lost_.store(false);
    check_cancel(goal_handle);
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(operation_lease_request_timeout_);
    while (!operation_lease_client_->wait_for_service(20ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw OperationError(
                PressCabinetButton::Result::LEASE_LOST,
                "The global robot operation lease service is unavailable; "
                "motion is disabled.");
      }
    }
    check_cancel(goal_handle);

    const std::string owner_id = std::string(get_fully_qualified_name()) +
      ":" + std::to_string(++operation_lease_owner_sequence_);
    auto request = std::make_shared<ManageOperationLease::Request>();
    request->command = ManageOperationLease::Request::ACQUIRE;
    request->owner_id = owner_id;
    request->requested_duration = operation_lease_duration_;

    ManageOperationLease::Response::SharedPtr response;
    try {
      auto future = operation_lease_client_->async_send_request(request);
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(operation_lease_request_timeout_);
      while (future.wait_for(20ms) != std::future_status::ready) {
        check_cancel(goal_handle);
        if (std::chrono::steady_clock::now() >= deadline) {
          throw OperationError(
                  PressCabinetButton::Result::LEASE_LOST,
                  "The global robot operation lease request timed out; "
                  "motion is disabled.");
        }
      }
      response = future.get();
    } catch (const OperationError &) {
      throw;
    } catch (const std::exception & error) {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              std::string("The global robot operation lease request failed: ") +
              error.what());
    }

    if (!response || !response->success) {
      const bool busy = response && response->status_code ==
        ManageOperationLease::Response::RESOURCE_BUSY;
      throw OperationError(
              busy ? PressCabinetButton::Result::RESOURCE_BUSY :
              PressCabinetButton::Result::LEASE_LOST,
              response && !response->message.empty() ? response->message :
              "The global robot operation lease was not granted.");
    }
    if (response->lease_id.empty() || response->owner_id != owner_id ||
      !std::isfinite(response->remaining_duration) ||
      response->remaining_duration <= 0.0)
    {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "The global robot operation lease response was invalid; "
              "motion is disabled.");
    }

    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      operation_lease_owner_id_ = owner_id;
      operation_lease_id_ = response->lease_id;
      operation_lease_held_.store(true);
    }
    operation_lease_stop_.store(false);
    try {
      operation_lease_renewal_thread_ = std::thread(
        [this]() {renew_operation_lease_loop();});
    } catch (...) {
      release_operation_lease_noexcept();
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "Could not start the operation lease renewal worker; "
              "motion is disabled.");
    }
    check_cancel(goal_handle);
    claim_active_goal_physical_motion_resources(
      goal_type, goal_handle->get_goal_id(), resources_required);
    check_cancel(goal_handle);
  }

  void renew_operation_lease_loop() noexcept
  {
    std::unique_lock<std::mutex> wait_lock(operation_lease_wait_mutex_);
    while (!operation_lease_stop_.load()) {
      if (operation_lease_condition_.wait_for(
          wait_lock,
          std::chrono::duration<double>(operation_lease_renew_period_),
          [this]() {return operation_lease_stop_.load();}))
      {
        return;
      }
      wait_lock.unlock();
      renew_operation_lease_once();
      wait_lock.lock();
      if (operation_lease_lost_.load()) {
        return;
      }
    }
  }

  void renew_operation_lease_once() noexcept
  {
    std::string owner_id;
    std::string lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (!operation_lease_held_.load()) {
        return;
      }
      owner_id = operation_lease_owner_id_;
      lease_id = operation_lease_id_;
    }

    if (!operation_lease_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "The global operation lease service became unavailable "
        "during renewal.");
      mark_operation_lease_lost(
        "The global operation lease service became unavailable.");
      return;
    }

    auto request = std::make_shared<ManageOperationLease::Request>();
    request->command = ManageOperationLease::Request::RENEW;
    request->owner_id = owner_id;
    request->lease_id = lease_id;
    request->requested_duration = operation_lease_duration_;
    try {
      auto future = operation_lease_client_->async_send_request(request);
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(operation_lease_request_timeout_);
      while (future.wait_for(20ms) != std::future_status::ready) {
        if (operation_lease_stop_.load()) {
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          RCLCPP_WARN(
            get_logger(),
            "The global operation lease renewal timed out "
            "after %.1fs.",
            operation_lease_request_timeout_);
          mark_operation_lease_lost(
            "The global operation lease renewal timed out.");
          return;
        }
      }
      const auto response = future.get();
      if (!response || !response->success ||
        response->lease_id != lease_id || response->owner_id != owner_id ||
        !std::isfinite(response->remaining_duration) ||
        response->remaining_duration <= 0.0)
      {
        RCLCPP_WARN(
          get_logger(),
          "The global operation lease renewal was rejected: %s",
          response && !response->message.empty() ? response->message.c_str() :
          "(no message)");
        mark_operation_lease_lost(
          response && !response->message.empty() ? response->message :
          "The global operation lease renewal was rejected.");
      } else {
        // Reuse the successful global-lease renewal as the cabinet physics
        // liveness heartbeat.  Only a currently published non-empty control
        // is repeated; idle operators never arm the Gazebo watchdog.
        publish_operation_heartbeat();
      }
    } catch (const std::exception & error) {
      mark_operation_lease_lost(
        std::string("The global operation lease renewal failed: ") +
        error.what());
    } catch (...) {
      mark_operation_lease_lost(
        "The global operation lease renewal failed unexpectedly.");
    }
  }

  void mark_operation_lease_lost(const std::string & reason) noexcept
  {
    mark_operation_lease_lost_impl(reason, nullptr);
  }

  void mark_operation_lease_lost_for_exact_lease(
    const std::string & reason,
    const std::string & expected_lease_id) noexcept
  {
    if (expected_lease_id.empty()) {
      return;
    }
    mark_operation_lease_lost_impl(reason, &expected_lease_id);
  }

  void mark_operation_lease_lost_impl(
    const std::string & reason,
    const std::string * expected_lease_id) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      const bool lease_matches = expected_lease_id ?
        operation_fault_matches_active_lease(
        operation_lease_held_.load(), operation_lease_id_,
        *expected_lease_id) :
        operation_fault_matches_active_lease(
        operation_lease_held_.load(), operation_lease_id_,
        operation_lease_id_);
      if (!lease_matches || operation_lease_lost_.load()) {
        return;
      }
      // Latch the exact expected lease while holding the same mutex used by
      // release/acquire.  A delayed watchdog fault cannot pass validation for
      // an old lease and then race into canceling its successor.
      operation_lease_lost_.store(true);
    }
    bool owns_physical_motion_resources = false;
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      owns_physical_motion_resources =
        active_goal_owns_physical_motion_resources_;
    }
    if (owns_physical_motion_resources) {
      RCLCPP_ERROR(
        get_logger(), "%s All MoveIt and Nav2 motion is being stopped.",
        reason.c_str());
      stop_active_motion();
      cancel_active_navigation();
      publish_manual_base_stop();
      request_navigation_mode_without_wait(false);
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s Planning-only validation is being canceled without "
        "publishing a physical stop command.", reason.c_str());
    }
    publish_active_control("");
  }

  void release_operation_lease_noexcept() noexcept
  {
    operation_lease_stop_.store(true);
    operation_lease_condition_.notify_all();
    if (operation_lease_renewal_thread_.joinable()) {
      operation_lease_renewal_thread_.join();
    }

    std::string owner_id;
    std::string lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (!operation_lease_held_.exchange(false)) {
        operation_lease_lost_.store(false);
        return;
      }
      owner_id = operation_lease_owner_id_;
      lease_id = operation_lease_id_;
      operation_lease_owner_id_.clear();
      operation_lease_id_.clear();
    }

    try {
      if (operation_lease_client_->service_is_ready()) {
        auto request = std::make_shared<ManageOperationLease::Request>();
        request->command = ManageOperationLease::Request::RELEASE;
        request->owner_id = owner_id;
        request->lease_id = lease_id;
        auto future = operation_lease_client_->async_send_request(request);
        if (future.wait_for(
            std::chrono::duration<double>(operation_lease_request_timeout_)) !=
          std::future_status::ready)
        {
          RCLCPP_WARN(
            get_logger(),
            "Operation lease release timed out; its short TTL will expire.");
        } else {
          const auto response = future.get();
          if (!response || !response->success) {
            RCLCPP_WARN(
              get_logger(), "Operation lease release was not acknowledged: %s",
              response ? response->message.c_str() : "empty response");
          }
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Operation lease service is unavailable during release; "
          "its short TTL will expire.");
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Operation lease release failed: %s", error.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Operation lease release failed unexpectedly.");
    }
    operation_lease_lost_.store(false);
  }

  template<typename GoalHandleT>
  void interruptible_hold(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    double seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      std::this_thread::sleep_for(20ms);
    }
  }

  template<typename GoalHandleT>
  void check_cancel(
    const std::shared_ptr<GoalHandleT> & goal_handle) const
  {
    if (operation_lease_lost_.load()) {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "The global robot operation lease was lost.");
    }
    if (goal_should_stop(goal_handle)) {
      throw OperationError(
              PressCabinetButton::Result::CANCELED,
              "Cabinet button operation was canceled.");
    }
    verify_latched_cabinet_transform();
  }

  template<typename GoalHandleT>
  void check_safety_transport_stop(
    const std::shared_ptr<GoalHandleT> &) const
  {
    if (operation_lease_lost_.load()) {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "The global robot operation lease was lost.");
    }
    if (shutdown_requested_.load() || !rclcpp::ok()) {
      throw OperationError(
              PressCabinetButton::Result::CANCELED,
              "Cabinet operation stopped because ROS is shutting down.");
    }
    verify_latched_cabinet_transform();
  }

  bool is_goal_canceling_noexcept(
    const std::shared_ptr<PressGoalHandle> & goal_handle) const noexcept
  {
    return goal_canceling_after_request(goal_handle);
  }

  bool finish_goal_noexcept(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    const std::shared_ptr<PressCabinetButton::Result> & result,
    bool request_success) noexcept
  {
    const auto finalize = [&]() {
        if (result) {
          if (request_success) {
            result->failure_reason.clear();
          } else {
            result->physical_outcome_confirmed = false;
            result->final_state_verified = false;
            result->transport_succeeded = false;
            result->recovery_succeeded = false;
            result->grasp_released = false;
            if (result->failure_reason.empty()) {
              result->failure_reason = result->message.empty() ?
                "Cabinet button operation failed without a diagnostic message." :
                result->message;
            }
          }
        }
        return apply_goal_terminal_disposition(
          goal_terminal_disposition(
            request_success, is_goal_canceling_noexcept(goal_handle)),
          [&]() {goal_handle->succeed(result);},
          [&]() {
            result->success = false;
            result->error_code = PressCabinetButton::Result::CANCELED;
            result->message = "Cabinet button operation was canceled.";
            result->failure_reason = result->message;
            goal_handle->canceled(result);
          },
          [&]() {goal_handle->abort(result);});
      };
    try {
      if (!goal_handle || !goal_handle->is_active()) {
        RCLCPP_WARN(
          get_logger(), "Action goal was already in a terminal state.");
        return false;
      }
      return finalize();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to set the action terminal state: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to set the action terminal state.");
    }

    // A cancel request may race the first terminal-state transition.
    try {
      if (goal_handle && goal_handle->is_active()) {
        return finalize();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the action terminal state: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the action terminal state.");
    }
    return false;
  }

  void publish_feedback(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    std::uint8_t phase,
    float progress,
    const std::string & message)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    const auto button = find_button(goal_handle->get_goal()->button_id);
    const auto state = button ? button_snapshot(*button) : ButtonSnapshot{};
    auto feedback = std::make_shared<PressCabinetButton::Feedback>();
    feedback->phase = phase;
    feedback->progress = progress;
    feedback->button_travel = state.position;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void publish_feedback(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    std::uint8_t phase,
    float progress,
    const std::string & message)
  {
    double target_position = 0.0;
    const auto control = find_button(goal_handle->get_goal()->control_id);
    if (control) {
      try {
        target_position = resolve_operation_target(
          *control, *goal_handle->get_goal(),
          button_snapshot(*control)).first;
      } catch (...) {
        target_position = button_snapshot(*control).position;
      }
    }
    publish_operate_feedback(
      goal_handle, phase, progress, target_position, message);
  }

  void publish_operate_feedback(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    std::uint8_t phase,
    float progress,
    double target_position,
    const std::string & message)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    const auto control = find_button(goal_handle->get_goal()->control_id);
    const auto state = control ?
      button_snapshot(*control) : ButtonSnapshot{};
    auto feedback = std::make_shared<OperateCabinetControl::Feedback>();
    feedback->phase = phase;
    feedback->progress = progress;
    feedback->current_position = state.position;
    feedback->target_position = target_position;
    feedback->current_state = state.state_id;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void stop_active_motion() noexcept
  {
    std::shared_ptr<MoveGroupInterface> move_group;
    std::array<std::shared_ptr<MoveGroupInterface>, 2> bimanual_groups{};
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      move_group = active_move_group_;
      bimanual_groups = bimanual_active_move_groups_;
    }
    const auto stop_one = [this](const std::shared_ptr<MoveGroupInterface> & group) {
        if (!group) {
          return;
        }
        try {
          group->getMoveGroupClient().async_cancel_all_goals();
          group->stop();
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(), "Failed to stop MoveIt: %s", error.what());
        }
      };
    stop_one(move_group);
    for (const auto & group : bimanual_groups) {
      stop_one(group);
    }
    // 双臂 drawer 直发控制器的目标也要撤销，否则楔死的 worker 永远等不到
    // 结果、execution 有界循环的 grace 窗口内无法回归。
    if (left_fjt_client_ && right_fjt_client_) {
      std::lock_guard<std::mutex> lock(bimanual_controller_handles_->mutex);
      if (bimanual_controller_handles_->left) {
        try {
          left_fjt_client_->async_cancel_goal(
            bimanual_controller_handles_->left);
        } catch (const std::exception &) {
        }
      }
      if (bimanual_controller_handles_->right) {
        try {
          right_fjt_client_->async_cancel_goal(
            bimanual_controller_handles_->right);
        } catch (const std::exception &) {
        }
      }
    }
  }

  void cancel_active_navigation() noexcept
  {
    NavigationGoalHandle::SharedPtr navigation_goal;
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      navigation_goal = active_navigation_goal_;
      active_navigation_goal_.reset();
    }
    if (navigation_goal) {
      try {
        navigation_client_->async_cancel_goal(navigation_goal);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Failed to cancel Nav2: %s", error.what());
      }
    }
  }

  void request_navigation_mode_without_wait(bool enabled) noexcept
  {
    try {
      if (!navigation_mode_client_->service_is_ready()) {
        return;
      }
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = enabled;
      navigation_mode_client_->async_send_request(request);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to request a safe base mode: %s", error.what());
    }
  }

  rclcpp_action::Server<PressCabinetButton>::SharedPtr action_server_;
  rclcpp_action::Server<OperateCabinetControl>::SharedPtr
    operate_action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr navigation_mode_client_;
  rclcpp::Client<ManageOperationLease>::SharedPtr operation_lease_client_;
  rclcpp::Client<
    xczs_inspection_robot_interfaces::srv::SetCabinetGrasp>::SharedPtr
    grasp_client_;
  rclcpp::Client<
    xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp>::SharedPtr
    bimanual_grasp_client_;
  rclcpp::Client<
    xczs_inspection_robot_interfaces::srv::SetCabinetUnlock>::SharedPtr
    unlock_client_;
  // 2026-09-06 AGENT doc §4.2: 最近一次抽屉解锁放行的插件证据（模式标签）。
  // set_drawer_unlock 成功后据此把 cap/stage 摘要与 Web 文案标成
  // simulated_linkage（绝不说成"finger3 按到了按钮"）；strict 模式为空/未置。
  std::string last_unlock_acceptance_mode_;
  std::string last_unlock_acceptance_message_;
  bool last_unlock_simulated_{false};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_physics_client_;
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr
    drawer_entity_client_;
  rclcpp::CallbackGroup::SharedPtr drawer_entity_callback_group_;
  // 物理真值测量状态（fix#7）: 首测成功后置真; 服务缺失/超时回退 FK 并只警告
  // 一次——真机无 gazebo 实体服务时保持历史 FK 行为不变。
  bool drawer_entity_measure_available_{false};
  bool drawer_entity_measure_warned_{false};
  rclcpp::CallbackGroup::SharedPtr reset_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr operation_lease_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_service_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_controls_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    manual_base_publisher_;
  rclcpp::Publisher<
    xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>::SharedPtr
    control_catalog_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    active_control_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    operation_heartbeat_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    operation_fault_subscription_;
  std::mutex operation_heartbeat_mutex_;
  std::string operation_heartbeat_control_id_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    cabinet_pose_valid_subscription_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
  button_joint_state_subscriptions_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
  button_pressed_subscriptions_;
  std::vector<rclcpp::Subscription<
      xczs_inspection_robot_interfaces::msg::CabinetControlState>::SharedPtr>
  control_state_subscriptions_;
  std::shared_ptr<tf2_ros::Buffer> transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;

  // 2026-09-04 AGENT §8.4 fix#3 root-cause: 真实关节状态直读（见
  // subscribe_to_real_joint_states 头注释）——rod measured 的唯一信源。
  std::string real_joint_state_topic_;
  rclcpp::CallbackGroup::SharedPtr real_joint_states_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    real_joint_states_subscription_;
  std::mutex real_joint_states_mutex_;
  std::unordered_map<std::string, double> real_joint_positions_;
  // Same-callback effort mirror (2026-09-04 fix#3 v2): the hook-seal press
  // gate judges a blocked rod by its measured effort, so the direct joint-state
  // subscription must also cache effort (a blocked rod reads ~60-68 N pressed,
  // ~0 N free-air settled — the discriminator between "pressed on the plate"
  // and "hovering short / slipped beside it").  Guarded by the same mutex and
  // freshness rule as real_joint_positions_.
  std::unordered_map<std::string, double> real_joint_efforts_;
  rclcpp::Time real_joint_states_stamp_{};
  // 最新 joint-state 样本的接收时刻（见 cache 函数注释的混合时钟说明）。
  std::chrono::steady_clock::time_point real_joint_states_receipt_{};

  // 2026-09-05 fix#13: 双臂 arm 控制器指令参考缓存（见
  // subscribe_to_arm_controller_references 头注释）——self-center 微修正与
  // 驻留的锚点（X_arr）来源。每侧独立 receipt：两侧控制器发布节奏不同步时
  // 任一侧的关节不会因另一侧刷新而误判新鲜。
  rclcpp::CallbackGroup::SharedPtr arm_controller_reference_callback_group_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::
    SharedPtr left_arm_controller_state_subscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::
    SharedPtr right_arm_controller_state_subscription_;
  std::mutex arm_controller_reference_mutex_;
  std::unordered_map<std::string, double> left_arm_reference_positions_;
  std::unordered_map<std::string, double> right_arm_reference_positions_;
  std::chrono::steady_clock::time_point left_arm_reference_receipt_{};
  std::chrono::steady_clock::time_point right_arm_reference_receipt_{};
  // 2026-09-05 fix#6 v32: 双臂驻留保活的关节序缓存 (controller_state.joint_names
  // 原序, 首帧填充)。seal phase 3 长流程里到达验证发的 12 s 双臂驻留约 14 s
  // 后会被 goal_time_tolerance abort (v31 取证), 保活刷新以本序 + reference
  // 重建同一条驻留; 无本缓存时无法构造 FJT 目标, 刷新跳过。
  std::vector<std::string> left_arm_controller_joint_order_;
  std::vector<std::string> right_arm_controller_joint_order_;

  std::unordered_map<std::string, std::shared_ptr<ButtonSpec>> buttons_by_id_;
  std::vector<std::shared_ptr<ButtonSpec>> buttons_in_order_;
  std::mutex motion_mutex_;
  std::shared_ptr<MoveGroupInterface> active_move_group_;
  // Left/right move groups registered while a bimanual drawer motion is in
  // flight, so stop_active_motion() (and the wedge watchdog) cancel BOTH arms
  // instead of only the single active_move_group_.
  std::array<std::shared_ptr<MoveGroupInterface>, 2> bimanual_active_move_groups_{};
  // 双臂 drawer 轨迹直发控制器（绕过 move_group 串行 execute）。左/右各一个
  // action client，执行 worker 各自发送、各自等待结果，从而真正并行；活动
  // goal handle 供 stop_active_motion 在超时/取消时撤销两条臂的控制器目标。
  // handles 堆共享：detach 的楔死 worker 即使 operator 已销毁仍可安全登记/
  // 清理自己的 goal handle，绝不触碰已析构的 this。
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    left_fjt_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    right_fjt_client_;
  // 2026-09-03 AGENT §3: the right unlock motor (unlock_motor_joint,
  // r_three_cyl_finger3_joint) lives on the three_cylinder controller, so its
  // extension is driven through a direct full-joint FJT goal on THAT controller
  // (see drive_unlock_motor) — not through MoveIt group planning, which would
  // collision-check the very latch contact the extension is meant to make.
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    unlock_motor_fjt_client_;
  // 2026-09-03 AGENT §4.2: the LEFT two-cylinder controller's direct FJT client
  // for the left support/gripper rods (drive_drawer_support_stage /
  // drive_drawer_hook_stage), mirroring unlock_motor_fjt_client_ for the
  // right three-cylinder controller.
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    two_cylinder_fjt_client_;
  // 2026-09-05 AGENT §8.4 fix#4 (hook seat release): controller_manager
  // switch/configure clients + callback group for the rod-drive integrator
  // drain.  configure reloads the gains and re-initialises the PID, which is
  // the only discharge path for the blocked-goal I-term charge; see
  // seal_drawer_hooks_by_probe phase 3.
  rclcpp::CallbackGroup::SharedPtr rod_controller_callback_group_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    controller_switch_client_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    controller_configure_client_;
  struct BimanualControllerHandles
  {
    std::mutex mutex;
    rclcpp_action::ClientGoalHandle<
      control_msgs::action::FollowJointTrajectory>::SharedPtr left;
    rclcpp_action::ClientGoalHandle<
      control_msgs::action::FollowJointTrajectory>::SharedPtr right;
  };
  std::shared_ptr<BimanualControllerHandles> bimanual_controller_handles_ =
    std::make_shared<BimanualControllerHandles>();
  // Shared serialization for MoveGroupInterface::execute() across bounded
  // motions (see execute_motion_bounded).  The mutexes are heap-shared so a
  // detached wedge worker can safely clear the transferred lock even if the
  // operator is destroyed; the execute serialization is what keeps a wedged
  // in-flight execute() from racing a later one on the same instance.
  struct MotionExecuteSerialization
  {
    std::mutex execute_mutex;
    std::mutex guard_mutex;
    std::optional<std::unique_lock<std::mutex>> wedged_lock;
  };
  std::shared_ptr<MotionExecuteSerialization> motion_execute_serial_ =
    std::make_shared<MotionExecuteSerialization>();
  std::mutex navigation_mutex_;
  NavigationGoalHandle::SharedPtr active_navigation_goal_;
  std::mutex navigation_feedback_mutex_;
  std::chrono::steady_clock::time_point last_navigation_feedback_{};
  std::mutex worker_mutex_;
  std::thread worker_thread_;
  std::mutex pending_goal_mutex_;
  std::unordered_map<std::string, PendingGoalDisposition>
    pending_goal_dispositions_;
  std::atomic<bool> operation_active_{false};
  std::mutex operation_lease_mutex_;
  std::string operation_lease_owner_id_;
  std::string operation_lease_id_;
  std::atomic<bool> operation_lease_held_{false};
  std::atomic<bool> operation_lease_lost_{false};
  std::atomic<bool> operation_lease_stop_{true};
  std::atomic<std::uint64_t> operation_lease_owner_sequence_{0};
  std::mutex operation_lease_wait_mutex_;
  std::condition_variable operation_lease_condition_;
  std::thread operation_lease_renewal_thread_;
  mutable std::mutex active_goal_mutex_;
  ActiveGoalType active_goal_type_{ActiveGoalType::NONE};
  rclcpp_action::GoalUUID active_goal_id_{};
  bool active_goal_owns_physical_motion_resources_{false};
  bool active_goal_physical_outcome_committed_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> cabinet_pose_valid_{false};
  std::atomic<bool> cabinet_pose_valid_received_{false};
  mutable std::mutex cabinet_transform_mutex_;
  tf2::Transform latched_cabinet_transform_{tf2::Transform::getIdentity()};
  // Watchdog 参照系锁存（cabinet_verify_frame_）：与 latched_cabinet_transform_
  // 在 latch 时刻一起采样，供 verify_latched_cabinet_transform 按物理锚定系
  // 校验真实柜体位移。
  tf2::Transform latched_cabinet_verify_transform_{tf2::Transform::getIdentity()};
  bool cabinet_transform_latched_{false};
  // 物理锚定配置（fix#cap2）：夹具几何真值实体名 + 其 cabinet 系声明局部原点。
  // 见构造器注释与 apply_physics_anchor_to_latch。
  std::string physics_anchor_entity_;
  tf2::Vector3 physics_anchor_local_origin_{0.0, 0.0, 0.0};
  bool physics_anchor_configured_{false};
  bool physics_anchor_measure_warned_{false};
  std::string cabinet_verify_frame_;

  std::string planning_frame_;
  std::string navigation_frame_;
  std::string cabinet_frame_;
  std::string robot_model_name_;
  std::string move_group_name_;
  std::string move_group_namespace_;
  std::string controller_namespace_;
  std::string toolset_;
  std::string contact_tool_link_;
  std::string grasp_link_;
  tf2::Vector3 grasp_point_position_{0.0, 0.0, 0.0};
  std::string transport_named_target_;
  ToolProfile::ToolAxisOrientation tool_axis_orientation_{
    ToolProfile::ToolAxisOrientation::ALONG_OUTWARD};
  tf2::Vector3 tool_tip_position_{0.0, 0.0, 0.0};
  std::vector<std::string> tool_tip_calibration_joint_names_;
  std::vector<double> tool_tip_calibration_joint_positions_;
  double tool_tip_calibration_joint_tolerance_{0.001};
  double tool_calibration_settle_timeout_{6.0};
  std::unordered_map<std::uint8_t, ToolProfile> tool_profiles_;
  // Bimanual drawer tools: right three-cylinder (right_arm) + left
  // two-cylinder (left_arm), read from the adapter's drawer section.  Only
  // configured when at least one drawer control is operable.
  BimanualToolProfile drawer_left_tool_;
  BimanualToolProfile drawer_right_tool_;
  bool drawer_tools_configured_{false};
  std::string drawer_transport_named_target_;
  double planning_time_{10.0};
  int planning_attempts_{10};
  // AGENT §7.2 现场分级封顶调试（0 = 全流程；1/3/4/5/6/7/8 = 阶段边界/拉距）。
  int debug_stage_cap_{0};
  double planning_velocity_scale_{0.20};
  double planning_acceleration_scale_{0.20};
  double goal_position_tolerance_{0.005};
  double goal_orientation_tolerance_{0.01};
  double goal_joint_tolerance_{0.001};
  bool allow_replanning_{true};
  bool allow_embedded_navigation_{true};
  std::string navigation_base_frame_;
  std::string docking_base_frame_;
  std::string grasp_brake_link_;
  bool require_cabinet_pose_valid_{true};
  double cabinet_pose_translation_tolerance_{0.020};
  double cabinet_pose_rotation_tolerance_{0.035};
  double operation_lease_duration_{3.0};
  double operation_lease_renew_period_{0.75};
  double operation_lease_request_timeout_{0.50};
  double prepress_distance_{0.060};
  double grasp_outward_offset_{0.020};
  double contact_clearance_{0.001};
  double press_depth_{0.007};
  double button_state_timeout_{1.0};
  double system_wait_timeout_{15.0};
  double navigation_timeout_{120.0};
  double navigation_takeover_distance_{0.15};
  double navigation_takeover_yaw_tolerance_{0.35};
  double navigation_takeover_stall_timeout_{4.0};
  double docking_timeout_{45.0};
  double docking_position_tolerance_{0.015};
  double docking_yaw_tolerance_{0.10};
  std::vector<PlanarFootprintPoint> docking_base_footprint_;
  double docking_base_footprint_padding_{0.03};
  double docking_max_linear_speed_{0.15};
  double docking_max_angular_speed_{0.45};
  double docking_linear_gain_{0.8};
  double docking_angular_gain_{1.2};
  // P3-8 grab-and-drive drawer pull (base translation along the drawer axis).
  double drawer_base_drive_max_speed_{0.05};
  double drawer_base_drive_gain_{1.0};
  double drawer_base_drive_timeout_{60.0};
  double navigation_velocity_yaw_offset_{-1.57079632679};
  double press_detection_timeout_{3.0};
  double release_detection_timeout_{3.0};
  double press_hold_seconds_{0.5};
  double force_tracking_tolerance_{0.00005};
  double force_tracking_max_compensation_{0.0010};
  double force_tracking_settle_seconds_{0.15};
  int force_tracking_attempts_{3};
  double button_press_minimum_cartesian_fraction_{0.95};
  double target_tolerance_{0.035};
  double slider_position_tolerance_{0.03};
  // AGENT §7.2 (cap7 run2 fix): 封顶拉距取证门。通用 slider_position_tolerance_
  // (0.03) ≥ cap7 拉距 0.03，会把"抽屉根本没动"当成"已到位"（run2 全程
  // 未越过 ~0.0075 却宣称 0.03 verified）。封顶流程专用：
  //   - debug_capped_pull_tolerance_：验证带 ±X，拉距完成后实测位置须落在
  //     [target−X, target+X]（0.03 拉距下静止漂移 ~0.003–0.008 不可能通过）；
  //   - capped_pull_already_at_tolerance_：跳过判据收紧到 0.002，保证 0.03
  //     这类短拉距本身不会被"already at the requested position"短路。
  double debug_capped_pull_tolerance_{0.010};
  double capped_pull_already_at_tolerance_{0.002};
  double stable_velocity_tolerance_{0.03};
  double stable_state_duration_{0.30};
  double drawer_support_contact_tolerance_{0.008};
  double drawer_gripper_contact_tolerance_{0.008};
  double drawer_arm_pose_tolerance_{0.008};
  double drawer_arm_orientation_tolerance_{0.02};
  double drawer_arm_stability_tolerance_{0.003};
  double grasp_attach_settle_duration_{0.15};
  double grasp_release_settle_duration_{0.30};
  double door_release_fraction_{0.60};
  double door_settle_timeout_{90.0};
  double door_release_position_timeout_{10.0};
  double door_detent_hysteresis_{0.02};
  double door_release_position_margin_{0.01};
  double rotary_pregrasp_clearance_{0.010};
  double knob_pregrasp_clearance_{0.050};
  double door_release_clearance_{0.30};
  double planning_scene_settle_seconds_{0.50};
  double rotation_waypoint_step_{0.03490658504};
  // Linear spacing (m) of the drawer grasp-drag waypoints along the slide axis.
  double drawer_waypoint_step_{0.03};
  // 2026-09-02 (reworded 2026-09-03 §3): unlock-retreat shaping.  After the
  // unlock motor retracts out of the zone the right tool disengages on a short
  // +outward/+z diagonal (release), then moves east at the raised height
  // (clear) so no part of it drags the now-free drawer along its rail.
  double drawer_unlock_retreat_outward_{0.020};
  double drawer_unlock_retreat_lift_{0.050};
  // A westward grasp press can only bite if the free drawer can still yield
  // west.  At / near the closed rail hard stop (position ~0) a full press pose
  // is unreachable and the arm controller aborts on state tolerance
  // (2026-09-02 run).  Keep at least this much closed-side travel as slack when
  // saturating the press depth (see drawer_grasp_contact_offset).
  double drawer_grasp_closed_floor_{0.004};
  double cartesian_velocity_scale_{0.08};
  double cartesian_acceleration_scale_{0.08};
  double cartesian_jump_threshold_{2.0};
  int cartesian_planning_attempts_{5};
  int door_cartesian_segment_waypoints_{2};
  int motion_planning_attempts_{3};
};

}  // namespace xczs_inspection_robot_control

#ifndef XCZS_CABINET_BUTTON_OPERATOR_NO_MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<
    xczs_inspection_robot_control::CabinetButtonOperator>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
#endif
