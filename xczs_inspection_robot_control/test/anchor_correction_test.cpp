#include <gtest/gtest.h>
#include "xczs_inspection_robot_control/anchor_correction.hpp"

TEST(AnchorCorrection, DoesNotAmplifyYawByDistanceToSceneOrigin)
{
  const tf2::Vector3 robot(-30.0, 5.8, 0.59);
  tf2::Quaternion yaw;
  yaw.setRPY(0, 0, 0.22);
  const tf2::Transform corrected(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));
  const tf2::Transform previous(yaw, robot - tf2::quatRotate(yaw, robot));
  EXPECT_GT(previous.getOrigin().length(), 6.0);
  EXPECT_NEAR(xczs_inspection_robot_control::anchor_correction_at_reference(
    previous, corrected, robot), 0.0, 1e-10);
}

TEST(AnchorCorrection, DetectsTranslationAndIsIndependentOfCoordinateOrigin)
{
  const tf2::Vector3 robot(-30.0, 5.8, 0.59);
  const tf2::Transform previous(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));
  const tf2::Transform corrected(tf2::Quaternion::getIdentity(), tf2::Vector3(2.1, 0, 0));
  EXPECT_NEAR(xczs_inspection_robot_control::anchor_correction_at_reference(
    previous, corrected, robot), 2.1, 1e-10);
  tf2::Quaternion yaw;
  yaw.setRPY(0, 0, 0.7);
  const tf2::Transform change_coordinates(yaw, tf2::Vector3(100, -80, 0));
  EXPECT_NEAR(xczs_inspection_robot_control::anchor_correction_at_reference(
    change_coordinates * previous, change_coordinates * corrected,
    change_coordinates * robot), 2.1, 1e-10);
}
