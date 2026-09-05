// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

// 2026-09-05 AGENT doc §4.3 (P1): 座封参考 = home_joint + 沿真实伸缩轴的有符号
// 投影。doc §4.3 明令禁止用工具端点到 home 端点的欧氏距离（欧氏会混入横向
// 抖动）；extension = dot(p_contact - p_home, extension_axis)，joint_ref =
// clamp(home_joint + extension, [joint_min, joint_max])。axis 必须是已归一化的
// 单位向量（调用方从工具标定/连杆姿态取得）。
//
// 本模型 1 关节米 = 1 端点米（棱柱杆沿其轴移动，杆端位移与关节位移 1:1），
// 故 extension（端点投影）可直接加到 home_joint（关节米）上。
//
// 设计为无 Eigen 依赖的纯函数（以 x/y/z 坐标传 3D 点），便于 §4.5 纯逻辑单元
// 测试（正/负/横向扰动投影、上下限裁剪）独立编译运行；operator 端用 Eigen
// 点调用它，保证「端点位移转关节位移」只有一份实现（§4.3 禁止复制近似实现）。

#include <algorithm>
#include <cmath>

namespace xczs_inspection_robot_control
{

struct DrawerHookSeatProjection
{
  // 行程在 [travel_min, travel_max] 且横向残差 <= transverse_max 时才置 true。
  bool ok{false};
  // dot(p_contact - p_home, axis)，单位 = 关节米。
  double extension{0.0};
  // |位移 - extension*axis|（滤除后的横向残余），单位 = 米。
  double transverse_residual{0.0};
  // clamp(home_joint + extension, [joint_min, joint_max])。
  double joint_ref{0.0};
};

inline DrawerHookSeatProjection project_drawer_hook_seat(
  double seat_x, double seat_y, double seat_z,
  double home_x, double home_y, double home_z,
  double axis_x, double axis_y, double axis_z,
  double home_joint, double joint_min, double joint_max,
  double travel_min, double travel_max, double transverse_max)
{
  DrawerHookSeatProjection out;
  const double dx = seat_x - home_x;
  const double dy = seat_y - home_y;
  const double dz = seat_z - home_z;
  const double extension = dx * axis_x + dy * axis_y + dz * axis_z;
  const double residual_x = dx - extension * axis_x;
  const double residual_y = dy - extension * axis_y;
  const double residual_z = dz - extension * axis_z;
  out.extension = extension;
  out.transverse_residual = std::sqrt(
    residual_x * residual_x + residual_y * residual_y + residual_z * residual_z);
  out.ok = extension >= travel_min && extension <= travel_max &&
    out.transverse_residual <= transverse_max;
  out.joint_ref = std::clamp(home_joint + extension, joint_min, joint_max);
  return out;
}

}  // namespace xczs_inspection_robot_control
