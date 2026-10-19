#include <gtest/gtest.h>
#include <joint_trajectory_controller/trajectory.hpp>
#include "xczs_inspection_robot_control/knob_jaw_hold.hpp"

using xczs_inspection_robot_control::bounded_knob_jaw_hold;
using xczs_inspection_robot_control::bounded_knob_jaw_close;

TEST(KnobJawClose, ProgressesThroughRecordedFreeSpaceServoLag)
{
  double target = 0.008;
  double measured = target - 0.000265;
  for (int tick = 0; tick < 100; ++tick) {
    const double next = bounded_knob_jaw_close(measured, target, 0.0145);
    ASSERT_GT(next, target);
    ASSERT_LE(next - measured, 0.00035 + 1e-12);
    target = next;
    measured = target - 0.000265;
    if (measured >= 0.0113) {break;}
  }
  EXPECT_GE(measured, 0.0113);
}

TEST(KnobJawClose, BlockedJawCannotAccumulateLeadAndLargeRecoilYields)
{
  constexpr double measured = 0.008;
  double target = measured;
  for (int tick = 0; tick < 10000; ++tick) {
    target = bounded_knob_jaw_close(measured, target, 0.0145);
    ASSERT_LE(target - measured, 0.00035 + 1e-12);
  }
  EXPECT_NEAR(target, measured + 0.00035, 1e-12);
  EXPECT_LT(bounded_knob_jaw_close(measured - 0.001, target, 0.0145), target);
}

TEST(KnobJawHold, RepeatedUpdatesCannotRatchetThroughPlate)
{
  constexpr double contact = 0.0118;
  double measured = contact;
  for (int tick = 0; tick < 10000; ++tick) {
    measured = bounded_knob_jaw_hold(measured, contact, 0.0, 0.0145);
    ASSERT_LE(measured - contact, 0.00035 + 1e-12);
  }
  EXPECT_NEAR(measured - contact, 0.00035, 1e-12);
}

TEST(KnobJawHold, FollowsRecoilAndSmallSeparationWithinFrozenBudget)
{
  constexpr double contact = 0.0118;
  // A solver displacement must allow the jaw to yield instead of holding an
  // old, deeper target. A 0.102 mm separation can still be closed by preload.
  EXPECT_LT(bounded_knob_jaw_hold(contact - 0.0003, contact, 0.0, 0.0145), contact);
  EXPECT_GT(bounded_knob_jaw_hold(contact, contact, 0.0, 0.0145) - contact, 0.000102);
  EXPECT_DOUBLE_EQ(bounded_knob_jaw_hold(0.02, contact, 0.0, 0.012), 0.012);
  EXPECT_DOUBLE_EQ(bounded_knob_jaw_hold(-0.001, contact, 0.0, 0.0145), 0.0);
}

TEST(KnobJawHold, PositionOnlyReleaseCannotInheritContactVelocityOvershoot)
{
  // Exercise the installed Humble trajectory sampler, not a duplicate spline.
  // The jaw is stationary against a plate, but the solver reports 20 mm/s.
  trajectory_msgs::msg::JointTrajectoryPoint measured;
  measured.positions = {0.012};
  measured.velocities = {0.02};
  const auto peak = [&](bool endpoint_derivatives) {
      auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
      msg->joint_names = {"jaw"};
      trajectory_msgs::msg::JointTrajectoryPoint goal;
      goal.positions = {0.0};
      goal.time_from_start = rclcpp::Duration::from_seconds(4.0);
      if (endpoint_derivatives) {goal.velocities = {0.0}; goal.accelerations = {0.0};}
      msg->points.push_back(goal);
      joint_trajectory_controller::Trajectory path(rclcpp::Time(0), measured, msg);
      double maximum = 0.0;
      for (int step = 0; step <= 400; ++step) {
        trajectory_msgs::msg::JointTrajectoryPoint output;
        joint_trajectory_controller::TrajectoryPointConstIter begin, end;
        EXPECT_TRUE(path.sample(rclcpp::Time(0) + rclcpp::Duration::from_seconds(step * 0.01),
          joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,
          output, begin, end));
        maximum = std::max(maximum, output.positions[0]);
        if (!endpoint_derivatives) {
          EXPECT_GE(output.positions[0], -1e-12);
          EXPECT_LE(output.positions[0], measured.positions[0] + 1e-12);
        }
      }
      return maximum;
    };
  EXPECT_GT(peak(true), 0.018);  // Previous opening command first closed >6 mm.
  EXPECT_NEAR(peak(false), 0.012, 1e-12);
}
