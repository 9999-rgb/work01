// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace xczs_inspection_robot_gazebo
{
// Re-express a child frame whose origin is unchanged and whose axes differ
// by a fixed yaw. Pose covariance remains in the parent frame; twist and its
// full covariance rotate into the new child frame.
inline nav_msgs::msg::Odometry navigation_odometry(
  const nav_msgs::msg::Odometry & source, const std::string & child, double yaw)
{
  auto result = source;
  result.child_frame_id = child;
  tf2::Quaternion original, rotation;
  tf2::fromMsg(source.pose.pose.orientation, original);
  rotation.setRPY(0, 0, yaw);
  result.pose.pose.orientation = tf2::toMsg(original * rotation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  double axes[6][6] = {};
  for (int block : {0, 3}) {
    axes[block][block] = c; axes[block][block + 1] = s;
    axes[block + 1][block] = -s; axes[block + 1][block + 1] = c;
    axes[block + 2][block + 2] = 1;
  }
  for (auto pair : {std::make_pair(&source.twist.twist.linear, &result.twist.twist.linear),
      std::make_pair(&source.twist.twist.angular, &result.twist.twist.angular)})
  {
    pair.second->x = c * pair.first->x + s * pair.first->y;
    pair.second->y = -s * pair.first->x + c * pair.first->y;
  }
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      double value = 0;
      for (int a = 0; a < 6; ++a) {
        for (int b = 0; b < 6; ++b) {
          value += axes[row][a] * source.twist.covariance[a * 6 + b] * axes[col][b];
        }
      }
      result.twist.covariance[row * 6 + col] = value;
    }
  }
  return result;
}
}
