// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace xczs_inspection_robot_control
{

struct PlanarFootprintPoint
{
  double x{0.0};
  double y{0.0};
};

/**
 * Return the footprint extent from the base origin toward the cabinet.
 *
 * A navigation station faces base +X toward the cabinet before applying its
 * configured yaw offset.  Expressing the cabinet direction in the base frame
 * therefore gives [cos(offset), -sin(offset)].  Taking the footprint support
 * in that direction keeps the clearance calculation valid for asymmetric
 * footprints and non-zero station yaw offsets.
 */
inline double cabinet_facing_footprint_extent(
  const std::vector<PlanarFootprintPoint> & footprint,
  double base_yaw_offset) noexcept
{
  if (footprint.size() < 3U || !std::isfinite(base_yaw_offset)) {
    return std::numeric_limits<double>::infinity();
  }
  const double direction_x = std::cos(base_yaw_offset);
  const double direction_y = -std::sin(base_yaw_offset);
  double extent = -std::numeric_limits<double>::infinity();
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return std::numeric_limits<double>::infinity();
    }
    extent = std::max(
      extent, point.x * direction_x + point.y * direction_y);
  }
  return extent;
}

/**
 * Return the worst cabinet-facing extent over the full accepted yaw error.
 *
 * For one footprint point, x*cos(angle)-y*sin(angle) reaches its maximum at
 * angle=atan2(-y, x), or at one of the accepted interval endpoints.  Testing
 * those analytic candidates is exact for a polygon and avoids sampling a
 * safety envelope.
 */
inline double worst_cabinet_facing_footprint_extent(
  const std::vector<PlanarFootprintPoint> & footprint,
  double base_yaw_offset,
  double docking_yaw_tolerance) noexcept
{
  if (footprint.size() < 3U || !std::isfinite(base_yaw_offset) ||
    !std::isfinite(docking_yaw_tolerance) || docking_yaw_tolerance < 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  constexpr double pi = 3.14159265358979323846;
  double extent = -std::numeric_limits<double>::infinity();
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return std::numeric_limits<double>::infinity();
    }
    const auto projection = [&point](double angle) {
        return point.x * std::cos(angle) - point.y * std::sin(angle);
      };
    extent = std::max(
      {extent,
        projection(base_yaw_offset - docking_yaw_tolerance),
        projection(base_yaw_offset + docking_yaw_tolerance)});

    const double point_radius = std::hypot(point.x, point.y);
    if (docking_yaw_tolerance >= pi) {
      extent = std::max(extent, point_radius);
      continue;
    }
    const double maximizing_angle = std::atan2(-point.y, point.x);
    const double angle_from_interval_center = std::atan2(
      std::sin(maximizing_angle - base_yaw_offset),
      std::cos(maximizing_angle - base_yaw_offset));
    if (std::abs(angle_from_interval_center) <=
      docking_yaw_tolerance + 1.0e-12)
    {
      extent = std::max(extent, point_radius);
    }
  }
  return extent;
}

/**
 * Minimum station standoff that preserves the padded base footprint even at
 * the closest position and worst heading accepted by the docking controller.
 */
inline double minimum_safe_station_standoff(
  const std::vector<PlanarFootprintPoint> & footprint,
  double footprint_padding,
  double docking_position_tolerance,
  double docking_yaw_tolerance,
  double base_yaw_offset) noexcept
{
  const double extent = worst_cabinet_facing_footprint_extent(
    footprint, base_yaw_offset, docking_yaw_tolerance);
  if (!std::isfinite(extent) || extent <= 0.0 ||
    !std::isfinite(footprint_padding) || footprint_padding < 0.0 ||
    !std::isfinite(docking_position_tolerance) ||
    docking_position_tolerance < 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  return extent + footprint_padding + docking_position_tolerance;
}

inline bool station_standoff_is_safe(
  double standoff,
  const std::vector<PlanarFootprintPoint> & footprint,
  double footprint_padding,
  double docking_position_tolerance,
  double docking_yaw_tolerance,
  double base_yaw_offset) noexcept
{
  const double minimum = minimum_safe_station_standoff(
    footprint, footprint_padding, docking_position_tolerance,
    docking_yaw_tolerance, base_yaw_offset);
  return std::isfinite(standoff) && std::isfinite(minimum) &&
         standoff + 1.0e-12 >= minimum;
}

/** Require both translation and heading to remain inside the docking gate. */
inline bool staging_pose_error_is_safe(
  double position_error,
  double yaw_error,
  double position_tolerance,
  double yaw_tolerance) noexcept
{
  return std::isfinite(position_error) && position_error >= 0.0 &&
         std::isfinite(yaw_error) &&
         std::isfinite(position_tolerance) && position_tolerance >= 0.0 &&
         std::isfinite(yaw_tolerance) && yaw_tolerance >= 0.0 &&
         position_error <= position_tolerance &&
         std::abs(yaw_error) <= yaw_tolerance;
}

}  // namespace xczs_inspection_robot_control
