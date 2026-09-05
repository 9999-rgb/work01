// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

// 2026-09-05 AGENT doc §4.5 (P1) 单元测试：座封参考 = home_joint + 沿真实伸缩轴
// 的有符号投影（禁止欧氏距离，避免混入横向抖动），并裁剪到关节上下限。
//
// 用例轴线：z 轴为伸缩轴（axis=(0,0,1) 归一），home 端点取原点，home_joint=0。
// 正/负/横向扰动对应座封点在轴正负向与横向的位置。行程门取 doc §4.3/当前实现
// 的同款 [0.0015, 0.025]，横向残差上限 0.0010，关节上限 0.120（URDF finger-rod）。
// clamp 用关节 [-0.05, 0.120] 显式验证上下界两侧裁剪（上限取真实值之上做通用性
// 断言，并另用收窄上限验证 clamp 生效）。

#include <cmath>

#include "gtest/gtest.h"

#include "xczs_inspection_robot_control/drawer_hook_seat_projection.hpp"

namespace xczs_inspection_robot_control
{
namespace
{

constexpr double kTravelMin = 0.0015;
constexpr double kTravelMax = 0.025;
constexpr double kTransverseMax = 0.0010;
constexpr double kJointMin = -0.05;
constexpr double kJointMax = 0.120;

// 真实场景：seat 沿 +z 前进 0.005 m（无横向分量）。
TEST(DrawerHookSeatProjection, PositiveAlongAxis)
{
  const auto out = project_drawer_hook_seat(
    0.0, 0.0, 0.005,                      // seat
    0.0, 0.0, 0.0,                        // home
    0.0, 0.0, 1.0,                        // axis (unit +z)
    0.0, kJointMin, kJointMax,            // home_joint, joint bounds
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_TRUE(out.ok);
  EXPECT_NEAR(out.extension, 0.005, 1e-9);
  EXPECT_NEAR(out.transverse_residual, 0.0, 1e-9);
  EXPECT_NEAR(out.joint_ref, 0.005, 1e-9);
}

// home_joint 本身非零（如历史遗留 0.0003）：joint_ref = home_joint + extension。
TEST(DrawerHookSeatProjection, HonorsHomeJointBias)
{
  const auto out = project_drawer_hook_seat(
    0.0, 0.0, 0.005,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0003, kJointMin, kJointMax,
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_TRUE(out.ok);
  EXPECT_NEAR(out.extension, 0.005, 1e-9);
  EXPECT_NEAR(out.joint_ref, 0.0053, 1e-9);
}

// 座封点落在 home 的轴负向（反向/误座封）：extension<0 → 超出 [travel_min,...]
// → ok=false，调用方据以整体失败进入安全收尾。
TEST(DrawerHookSeatProjection, NegativeExtensionIsRejected)
{
  const auto out = project_drawer_hook_seat(
    0.0, 0.0, -0.002,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, kJointMin, kJointMax,
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_FALSE(out.ok);
  EXPECT_NEAR(out.extension, -0.002, 1e-9);
}

// 纯横向扰动：位移几乎与轴垂直，欧氏距离会误判成行程；投影应给出 ~0 extension
// 且横向残差超出上限 → ok=false（轴错/臂漂检测）。
TEST(DrawerHookSeatProjection, TransverseOnlyIsRejected)
{
  const auto out = project_drawer_hook_seat(
    0.003, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, kJointMin, kJointMax,
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_FALSE(out.ok);
  EXPECT_NEAR(out.extension, 0.0, 1e-9);
  EXPECT_NEAR(out.transverse_residual, 0.003, 1e-9);
}

// 横向分量不超过上限时允许：行程仍按轴向分量计，横向残余不否决（容许标定/读
// 数噪声）。0.0005 m 横向 < 0.0010 上限。
TEST(DrawerHookSeatProjection, SmallTransverseWithinLimitKeepsOk)
{
  const auto out = project_drawer_hook_seat(
    0.0005, 0.0, 0.005,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, kJointMin, kJointMax,
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_TRUE(out.ok);
  EXPECT_NEAR(out.extension, 0.005, 1e-9);
  EXPECT_NEAR(out.joint_ref, 0.005, 1e-9);
}

// 行程超过合理性上界（如旧 v32 混叠 +10 mm 假座封）→ ok=false。
TEST(DrawerHookSeatProjection, OvershootTravelIsRejected)
{
  const auto out = project_drawer_hook_seat(
    0.0, 0.0, 0.050,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, kJointMin, kJointMax,
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_FALSE(out.ok);
  EXPECT_NEAR(out.extension, 0.050, 1e-9);
}

// 关节上限裁剪：home_joint + extension 超过上限 → joint_ref == 上限（不改 ok：
// 合理大行程被 clamp 到 URDF 上限，调用方后续有 travel 上界门保护）。
TEST(DrawerHookSeatProjection, ClampsJointRefAtUpperLimit)
{
  const auto out = project_drawer_hook_seat(
    0.0, 0.0, 0.150,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 0.0, 0.120,   // 收窄上限，验证 clamp 生效
    kTravelMin, kTravelMax, kTransverseMax);
  // extension 0.150 超出 travel_max → ok=false（行程门），但 joint_ref 仍应裁到
  // 上限而非越过。
  EXPECT_FALSE(out.ok);
  EXPECT_DOUBLE_EQ(out.joint_ref, 0.120);
}

// 关节下限裁剪：负 home_joint + 负 extension 突破下限 → joint_ref == 下限。
TEST(DrawerHookSeatProjection, ClampsJointRefAtLowerLimit)
{
  const auto out = project_drawer_hook_seat(
    0.0, 0.0, -0.010,
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    -0.05, kJointMin, kJointMax,
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_FALSE(out.ok);
  EXPECT_DOUBLE_EQ(out.joint_ref, kJointMin);
}

// 轴是任意单位向量时（非坐标轴对齐），投影不依赖基：沿轴 0.005 的位移应得
// extension=0.005，无论轴方向；横向残余 ~0。
TEST(DrawerHookSeatProjection, AxisNeedNotBeCoordinateAligned)
{
  constexpr double kInvSqrt3 = 1.0 / std::sqrt(3.0);
  // axis = (1,1,1)/sqrt(3)，位移 = 0.005 * axis。
  const auto out = project_drawer_hook_seat(
    0.005 * kInvSqrt3, 0.005 * kInvSqrt3, 0.005 * kInvSqrt3,
    0.0, 0.0, 0.0,
    kInvSqrt3, kInvSqrt3, kInvSqrt3,
    0.0, kJointMin, kJointMax,
    kTravelMin, kTravelMax, kTransverseMax);
  EXPECT_TRUE(out.ok);
  EXPECT_NEAR(out.extension, 0.005, 1e-9);
  EXPECT_NEAR(out.transverse_residual, 0.0, 1e-9);
  EXPECT_NEAR(out.joint_ref, 0.005, 1e-9);
}

}  // namespace
}  // namespace xczs_inspection_robot_control
