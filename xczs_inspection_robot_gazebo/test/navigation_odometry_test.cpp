#include <gtest/gtest.h>
#include <tf2/utils.h>
#include "xczs_inspection_robot_gazebo/navigation_odometry.hpp"

TEST(NavigationOdometry, ConvertsBodyMotionToNav2AxesWithoutChangingParentPosition)
{
  nav_msgs::msg::Odometry source;
  source.header.frame_id = "odom";
  source.child_frame_id = "body";
  source.pose.pose.position.x = -30;
  source.pose.pose.position.y = 5.8;
  tf2::Quaternion heading;
  heading.setRPY(0, 0, M_PI_2);
  source.pose.pose.orientation = tf2::toMsg(heading);
  source.twist.twist.linear.x = -.05;
  source.twist.twist.linear.y = -.45;
  source.twist.twist.angular.z = .2;
  source.twist.covariance[0] = .01;
  source.twist.covariance[7] = .04;
  source.twist.covariance[1] = source.twist.covariance[6] = .002;
  const auto result = xczs_inspection_robot_gazebo::navigation_odometry(
    source, "base_link", -M_PI_2);
  EXPECT_EQ(result.child_frame_id, "base_link");
  EXPECT_EQ(result.pose.pose.position, source.pose.pose.position);
  EXPECT_NEAR(tf2::getYaw(result.pose.pose.orientation), 0, 1e-10);
  EXPECT_NEAR(result.twist.twist.linear.x, .45, 1e-10);
  EXPECT_NEAR(result.twist.twist.linear.y, -.05, 1e-10);
  EXPECT_DOUBLE_EQ(result.twist.twist.angular.z, .2);
  EXPECT_NEAR(result.twist.covariance[0], .04, 1e-10);
  EXPECT_NEAR(result.twist.covariance[7], .01, 1e-10);
  EXPECT_NEAR(result.twist.covariance[1], -.002, 1e-10);
  const auto round_trip = xczs_inspection_robot_gazebo::navigation_odometry(
    result, "body", M_PI_2);
  EXPECT_NEAR(round_trip.twist.twist.linear.x, source.twist.twist.linear.x, 1e-10);
  EXPECT_NEAR(round_trip.twist.twist.linear.y, source.twist.twist.linear.y, 1e-10);
}
