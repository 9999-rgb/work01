#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include "xczs_inspection_robot_control/precision_cartesian_retiming.hpp"

TEST(PrecisionCartesianRetiming, PreservesSubMilliradianCorrectionsAndLimits)
{
  const auto urdf = urdf::parseURDF(R"(<robot name="tiny"><link name="base"/>
    <link name="tip"/><joint name="joint" type="revolute"><parent link="base"/>
    <child link="tip"/><axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)");
  ASSERT_TRUE(urdf);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf, R"(<robot name="tiny"><group name="arm">
    <joint name="joint"/></group></robot>)"));
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  for (double travel : {0.0001, -0.0003, 0.0006, 0.0015}) {
    robot_trajectory::RobotTrajectory trajectory(model, "arm");
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (int index = 0; index <= 4; ++index) {
      state.setVariablePosition("joint", travel * index / 4.0);
      trajectory.addSuffixWayPoint(state, 0.1);
    }
    ASSERT_TRUE(xczs_inspection_robot_control::retime_precision_cartesian_path(
      trajectory, 0.1, 0.1));
    ASSERT_GT(trajectory.getWayPointCount(), 1U);
    EXPECT_NEAR(trajectory.getFirstWayPoint().getVariablePosition("joint"), 0.0, 1e-9);
    EXPECT_NEAR(trajectory.getLastWayPoint().getVariablePosition("joint"), travel, 1e-9);
    EXPECT_GT(trajectory.getDuration(), 0.0);
    for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index) {
      EXPECT_LE(std::abs(trajectory.getWayPoint(index).getVariableVelocity("joint")), 0.100001);
    }
  }
}
