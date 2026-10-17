// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace xczs_inspection_robot_control
{
inline bool retime_precision_cartesian_path(
  robot_trajectory::RobotTrajectory & trajectory,
  double velocity_scale, double acceleration_scale)
{
  // TOTG's default 1e-3 rad minimum change can collapse an entire sub-mm
  // alignment into one stationary point. Preserve those solved endpoints;
  // retain the caller's velocity/acceleration limits and collision checks.
  trajectory_processing::TimeOptimalTrajectoryGeneration parameterizer(
    1.0e-5, 0.02, 1.0e-6);
  return parameterizer.computeTimeStamps(trajectory, velocity_scale, acceleration_scale);
}
}
