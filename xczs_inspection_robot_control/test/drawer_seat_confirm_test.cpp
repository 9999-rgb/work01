// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

// 2026-09-06 AGENT doc §5.1 单元测试：fix#21 座封接管确认的时序判断（纯逻辑）。
// doc 要求至少覆盖四类合成事件：
//   1) 一帧成功后缺数不得通过；
//   2) 单侧持续缺数不得通过；
//   3) 持续回缩拒绝；
//   4) 正常双侧连续样本通过。
// 另补：超界即时失败、越预算断数打断在位计时、短暂掉带（自愈颤振）后仍可确认、
// 掉带超 slip 时限才失败（< 1.0 s 的瞬态不掉带不判死）、确认须过 base。

#include <cmath>

#include "gtest/gtest.h"

#include "xczs_inspection_robot_control/drawer_seat_confirm.hpp"

namespace xczs_inspection_robot_control
{
namespace
{

constexpr double kDt = 0.05;         // 采样节拍（模拟 operator 复核循环 ~0.05 s）
constexpr double kSeat = 0.030;      // 座封参考（cap2 实测语义，在带 [0.025, 0.038]）

SeatConfirmParams params()
{
  return SeatConfirmParams{};  // base 1.5 / max 2.6 / confirm 0.4 / slip 1.0 / 带默认 / budget 0.5
}

SeatConfirmSideFrame in_band()
{
  SeatConfirmSideFrame f;
  f.present = true;
  f.travel = kSeat;   // 正好在座封参考位
  f.seat = kSeat;
  return f;
}

SeatConfirmSideFrame below()
{
  SeatConfirmSideFrame f;
  f.present = true;
  f.travel = kSeat - 0.010;  // 回缩 10 mm（> 下带 5 mm）→ 掉带
  f.seat = kSeat;
  return f;
}

SeatConfirmSideFrame over_up()
{
  SeatConfirmSideFrame f;
  f.present = true;
  f.travel = kSeat + 0.010;  // 顶出 10 mm（> 上带 8 mm）→ 超界
  f.seat = kSeat;
  return f;
}

SeatConfirmSideFrame dropout()
{
  return SeatConfirmSideFrame{};  // present = false
}

// 正常双侧在带：必须在 base 之后靠「配对连续新鲜样本」确认。
TEST(DrawerSeatConfirm, NormalPairedContiguousSamplesConfirm)
{
  SeatConfirmWindow w(params());
  w.Reset(0.0);
  SeatConfirmStatus last = SeatConfirmStatus::kPending;
  for (double t = kDt; t <= 2.0; t += kDt) {
    last = w.Update(t, in_band(), in_band());
  }
  EXPECT_EQ(last, SeatConfirmStatus::kConfirmed);
  EXPECT_GE(w.wall_elapsed(), 1.5);       // 确认必然在最短观察之后
  EXPECT_GE(w.paired_dwell(), 0.4 - 1e-9);
}

// 一帧成功后缺数（长时间无新样本）不得通过 —— 旧 fix#21 用「inband_since 墙钟」
// 会被单帧好样本 + 之后的缺数骗过；新逻辑在位计时只由新鲜配对帧累计。
TEST(DrawerSeatConfirm, SingleGoodFrameThenDropoutDoesNotConfirm)
{
  SeatConfirmWindow w(params());
  w.Reset(0.0);
  // 前 1.5 s 只喂一帧在带，随后全部 dropout。
  SeatConfirmStatus last = w.Update(1.0, in_band(), in_band());
  EXPECT_EQ(last, SeatConfirmStatus::kPending);
  for (double t = 1.05; t <= 3.0; t += kDt) {
    last = w.Update(t, dropout(), dropout());
    EXPECT_NE(last, SeatConfirmStatus::kConfirmed);  // 绝不通过
  }
  EXPECT_EQ(last, SeatConfirmStatus::kFailWindowBudget);  // 硬上限兜底，而非假通过
}

// 单侧持续缺数不得通过：左持续在带新鲜，右始终无读数 → 无配对帧 → 不通过。
TEST(DrawerSeatConfirm, OneSideMissingDoesNotConfirm)
{
  SeatConfirmWindow w(params());
  w.Reset(0.0);
  SeatConfirmStatus last = SeatConfirmStatus::kPending;
  for (double t = kDt; t <= 3.0; t += kDt) {
    last = w.Update(t, in_band(), dropout());
    EXPECT_NE(last, SeatConfirmStatus::kConfirmed);
  }
  EXPECT_EQ(last, SeatConfirmStatus::kFailWindowBudget);
  // 诊断应能区分「持续缺数 / 采样过旧」（fresh age 超过预算）。
  EXPECT_TRUE(std::string(w.Describe(3.0)).find("missing/stale") !=
    std::string::npos);
}

// 持续回缩（双侧低于下带 > 1.0 s）拒绝；瞬态掉带 < slip 时限不判死。
TEST(DrawerSeatConfirm, SustainedRetractIsRejectedButBriefDipIsTolerated)
{
  SeatConfirmWindow w(params());
  w.Reset(0.0);
  // 先正常在带一小段，然后进入持续回缩。
  SeatConfirmStatus last = SeatConfirmStatus::kPending;
  for (double t = kDt; t < 0.3; t += kDt) {
    last = w.Update(t, in_band(), in_band());
  }
  double t = 0.3;
  const double dip_start = t;
  for (; t - dip_start <= 0.8 + 1e-9; t += kDt) {  // 0.8 s 掉带：仍在 slip 时限内
    last = w.Update(t, below(), below());
    EXPECT_NE(last, SeatConfirmStatus::kConfirmed);
    EXPECT_TRUE(last == SeatConfirmStatus::kPending ||
      last == SeatConfirmStatus::kFailSlipDownLeft ||
      last == SeatConfirmStatus::kFailSlipDownRight);
  }
  EXPECT_EQ(last, SeatConfirmStatus::kPending);  // 0.8 s < 1.0 s 时限：容忍
  // 再继续掉带超过 1.0 s 总时长 → 判失败。
  bool failed = false;
  for (; t <= 2.0; t += kDt) {
    last = w.Update(t, below(), below());
    if (last == SeatConfirmStatus::kFailSlipDownLeft ||
      last == SeatConfirmStatus::kFailSlipDownRight)
    {
      failed = true;
      break;
    }
  }
  EXPECT_TRUE(failed);
  EXPECT_TRUE(last == SeatConfirmStatus::kFailSlipDownLeft ||
    last == SeatConfirmStatus::kFailSlipDownRight);
}

// 新鲜样本超上带（顶出/冲出）即时失败，不等到硬上限。
TEST(DrawerSeatConfirm, OverUpBandFailsImmediately)
{
  SeatConfirmWindow w(params());
  w.Reset(0.0);
  SeatConfirmStatus last = SeatConfirmStatus::kPending;
  for (double t = kDt; t < 1.0; t += kDt) {
    last = w.Update(t, in_band(), in_band());
  }
  last = w.Update(1.05, over_up(), in_band());   // 左超界
  EXPECT_EQ(last, SeatConfirmStatus::kFailTravelUpLeft);
}

// 越预算断数打断在位计时：掉带/断数 0.6 s（> budget 0.5 s）后再恢复在带，须重新
// 凑满 0.4 s 才确认，不能沿用断数前攒下的 dwell。
TEST(DrawerSeatConfirm, OverBudgetDropoutResetsDwell)
{
  SeatConfirmWindow w(params());
  w.Reset(0.0);
  SeatConfirmStatus last = SeatConfirmStatus::kPending;
  // 攒 ~0.3 s 配对在带 dwell。
  for (double t = kDt; t <= 0.3 + 1e-9; t += kDt) {
    last = w.Update(t, in_band(), in_band());
  }
  // 0.6 s 双侧缺数（> budget 0.5 s）→ 打断。
  double t = 0.35;
  for (; t <= 0.9 + 1e-9; t += kDt) {
    last = w.Update(t, dropout(), dropout());
  }
  // 恢复在带后立刻不能确认（dwell 已被打断）。
  last = w.Update(t, in_band(), in_band());
  EXPECT_EQ(last, SeatConfirmStatus::kPending);
  // 重新凑满 0.4 s 之前仍 pending。
  for (; t < 1.5 + 0.4; t += kDt) {
    last = w.Update(t, in_band(), in_band());
  }
  // 从断数恢复后须再累积 ≥ 0.4 s（越过 base 之后）才确认。
  EXPECT_EQ(last, SeatConfirmStatus::kConfirmed);
}

// 确认不得早于最短被动观察 base。
TEST(DrawerSeatConfirm, DoesNotConfirmBeforeBaseObservation)
{
  SeatConfirmWindow w(params());
  w.Reset(0.0);
  SeatConfirmStatus last = SeatConfirmStatus::kPending;
  for (double t = kDt; t <= 1.0; t += kDt) {   // 仅到 1.0 s < base 1.5 s
    last = w.Update(t, in_band(), in_band());
  }
  EXPECT_EQ(last, SeatConfirmStatus::kPending);
}

}  // namespace
}  // namespace xczs_inspection_robot_control
