// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace xczs_inspection_robot_control
{
using ContactVector = std::array<double, 3>;
struct ContactBox
{
  ContactVector center;
  std::array<ContactVector, 3> axes;
  ContactVector half_size;
};

// Separating-axis test for two oriented boxes. Positive is separation,
// negative is overlap; all 15 face/edge axes are checked.
inline double box_contact_separation(const ContactBox & a, const ContactBox & b)
{
  const auto dot = [](const ContactVector & x, const ContactVector & y) {
      return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
    };
  ContactVector delta{};
  for (int i = 0; i < 3; ++i) {
    delta[i] = b.center[i] - a.center[i];
    if (!std::isfinite(delta[i]) || !std::isfinite(a.half_size[i]) ||
      !std::isfinite(b.half_size[i]) || a.half_size[i] <= 0 || b.half_size[i] <= 0)
    {
      return std::numeric_limits<double>::infinity();
    }
    for (int j = 0; j < 3; ++j) {
      if (!std::isfinite(a.axes[i][j]) || !std::isfinite(b.axes[i][j])) {
        return std::numeric_limits<double>::infinity();
      }
    }
  }
  double separation = -std::numeric_limits<double>::infinity();
  const auto test = [&](ContactVector axis) {
      const double norm = std::sqrt(dot(axis, axis));
      if (norm < 1e-9) {return;}
      for (double & value : axis) {value /= norm;}
      double radius = 0;
      for (int i = 0; i < 3; ++i) {
        radius += a.half_size[i] * std::abs(dot(a.axes[i], axis)) +
          b.half_size[i] * std::abs(dot(b.axes[i], axis));
      }
      separation = std::max(separation, std::abs(dot(delta, axis)) - radius);
    };
  for (int i = 0; i < 3; ++i) {
    test(a.axes[i]);
    test(b.axes[i]);
    for (int j = 0; j < 3; ++j) {
      const auto & x = a.axes[i];
      const auto & y = b.axes[j];
      test({x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2],
        x[0] * y[1] - x[1] * y[0]});
    }
  }
  return separation;
}
}  // namespace xczs_inspection_robot_control
