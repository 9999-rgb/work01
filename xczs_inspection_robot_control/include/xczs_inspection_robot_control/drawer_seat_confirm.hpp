// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

// 2026-09-06 AGENT doc §5.1: 把 fix#21 座封接管确认的时序判断抽成小型纯逻辑，
// 用合成事件单测覆盖「一帧成功后缺数不得通过 / 单侧持续缺数不得通过 / 持续回缩
// 拒绝 / 正常双侧连续样本通过」。doc 对判定器的四项硬要求：
//   1) 每侧保存最近成功样本时间和序号；
//   2) 连续确认必须由多帧新样本覆盖指定时长；缺数超过采样预算即打断连续计时；
//   3) 获取两侧样本后更新判断时钟；同步查询耗时不能被当作有效在位时间；
//   4) 明确区分「超界 / 持续缺数 / 采样过旧 / 机械振荡」，不得混成一个 NOT_READY。
//
// 语义模型（无 ROS / 无时间库依赖，now 为调用方单调秒数）：
//   * 只有「新鲜在带采样帧」驱动在位计时。所谓新鲜 = 调用方在该帧确实取到了新的
//     物理读数（dropout 帧 present=false）。
//   * 判定采用「配对帧」模型：只有同一帧两侧都取到新鲜在带样本（paired in-band），
//     该帧之间被真实观测覆盖的时长才会计入 paired_dwell_；缺一帧则暂停、缺数超过
//     fresh_budget_seconds 则打断（paired_dwell_ 清零）。同步查询 / 保持耗时从不
//     直接变成在位时长 —— 它们只会让相邻两帧的时间间隔变大，一旦越过预算即清零。
//   * 连续低于下带（持续回缩 / 卡死）按侧累计 slip_dwell_，同样只由新鲜样本段累计，
//     越预算即清零重计；一旦 >= slip_max_seconds 立即失败（区分「持续回缩」）。
//   * 行程高于上带（顶出 / 冲出）是新鲜样本即败的硬错误（区分「超界」）。
//   * base/max 用墙钟（从 Reset 起），只作「最短被动观察」与安全硬上限，不计在位。

#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

namespace xczs_inspection_robot_control
{

enum class SeatConfirmStatus
{
  kPending,            // 未确认、仍在预算内（可能缺数 / 采样过旧 / 振荡中）
  kConfirmed,          // 双侧配对连续在位时长达标 → 通过
  kFailTravelUpLeft,   // 左行程高于上带（顶出 / 冲出），新鲜样本即败
  kFailTravelUpRight,  // 右行程高于上带
  kFailSlipDownLeft,   // 左连续低于下带超过 slip 时限（持续回缩 / 卡死）
  kFailSlipDownRight,  // 右连续低于下带超过 slip 时限
  kFailWindowBudget,   // 墙钟观察超过 max 仍未确认（持续缺数 / 振荡 / 长期未达标）
};

inline const char * seat_confirm_status_name(SeatConfirmStatus status)
{
  switch (status) {
    case SeatConfirmStatus::kPending: return "pending";
    case SeatConfirmStatus::kConfirmed: return "confirmed";
    case SeatConfirmStatus::kFailTravelUpLeft: return "fail_travel_up_left";
    case SeatConfirmStatus::kFailTravelUpRight: return "fail_travel_up_right";
    case SeatConfirmStatus::kFailSlipDownLeft: return "fail_slip_down_left";
    case SeatConfirmStatus::kFailSlipDownRight: return "fail_slip_down_right";
    case SeatConfirmStatus::kFailWindowBudget: return "fail_window_budget";
    default: return "unknown";
  }
}

struct SeatConfirmParams
{
  double base_seconds{1.5};    // 最短被动墙钟观察（fix#15 语义：接管瞬间不滑）
  double max_seconds{2.6};     // 墙钟安全硬上限
  double confirm_seconds{0.4}; // 双侧配对连续在位所需时长（fix#21）
  double slip_max_seconds{1.0};  // 连续低于下带超过此时长 = 真滑脱 / 卡死（fix#21）
  double down_band_m{0.005};   // 座封-5 mm 单向底线（fix#19）
  double up_band_m{0.008};     // 座封+8 mm 单向顶线（fix#19）
  // 相邻两次新鲜样本的最大许可间隔。超过即视为「持续缺数」：打断该侧在位/掉带
  // 连续段（fresh_budget 越大越容忍读回空隙，越小对缺数越敏感）。须大于正常采样
  // 节拍若干倍、小于最短需要被识别的掉带持续时长。
  double fresh_budget_seconds{0.5};
};

// 一帧内某一侧的新鲜样本。present=false 表示该帧没取到新读数（dropout），此时
// travel/seat 字段无效。
struct SeatConfirmSideFrame
{
  bool present{false};
  double travel{0.0};  // 该侧行程（杆端沿伸缩轴有符号位移，米）
  double seat{0.0};    // 该侧座封参考（extension，米），在带 = [seat-down, seat+up]
};

class SeatConfirmWindow
{
public:
  explicit SeatConfirmWindow(const SeatConfirmParams & params)
  : params_(params) {}

  // (重新)开始观察。start_time 为本次观察的单调秒数基准（只作墙钟预算，不进位）。
  void Reset(double start_time)
  {
    start_ = start_time;
    last_frame_ = start_time;
    wall_elapsed_ = 0.0;
    paired_dwell_ = 0.0;
    paired_last_ = -1.0;
    status_ = SeatConfirmStatus::kPending;
    left_.reset();
    right_.reset();
  }

  // 喂入一帧（now 单调不减，单位秒）。返回当前判定：一旦返回终态（非 kPending），
  // 后续调用保持终态不变（调用方应停止喂帧）。处理顺序：新鲜超界 → 掉带累计 →
  // 配对在位累计 → 确认 / 硬上限。
  SeatConfirmStatus Update(
    double now, const SeatConfirmSideFrame & left,
    const SeatConfirmSideFrame & right)
  {
    if (status_ != SeatConfirmStatus::kPending) {
      return status_;
    }
    if (now < last_frame_) {
      now = last_frame_;  // 时间回拨保护：不允许负的“观测时长”
    }
    last_frame_ = now;
    wall_elapsed_ = now - start_;

    // 本帧起点的新鲜标记清空：fresh_inband_this_frame 只代表「这一帧」真的取到
    // 新鲜在带读数，不能把上一帧的标记带进 dropout 帧（否则 dropout 会被误当作
    // 仍在带而错误累计）。
    left_.fresh_inband_this_frame = false;
    right_.fresh_inband_this_frame = false;

    // 每侧新鲜样本处理（超界即时失败；掉带累计在本函数内由配对/后续判断体现）。
    SeatConfirmStatus side_status = process_side(
      now, left, true, left_);
    if (side_status != SeatConfirmStatus::kPending) {
      return set_status(side_status);
    }
    side_status = process_side(now, right, false, right_);
    if (side_status != SeatConfirmStatus::kPending) {
      return set_status(side_status);
    }

    // 配对在位累计：只有同帧双侧都新鲜且在带才推进 paired_dwell_。
    // 任一侧缺一帧 → 本帧不累计（暂停）；缺数越预算 → active_inband 已在上方
    // process_side 清掉，走到 else 分支把 paired_dwell_ 清零（打断连续计时）。
    const bool paired_inband_now = left_.active_inband && right_.active_inband;
    const bool both_fresh_inband = left_.fresh_inband_this_frame &&
      right_.fresh_inband_this_frame;
    if (paired_inband_now && both_fresh_inband) {
      if (paired_last_ >= 0.0) {
        const double gap = now - paired_last_;
        if (gap >= 0.0 && gap <= params_.fresh_budget_seconds) {
          paired_dwell_ += gap;
        } else {
          paired_dwell_ = 0.0;  // 断数/振荡超预算：打断，从本帧重新计
        }
      } else {
        paired_dwell_ = 0.0;
      }
      paired_last_ = now;
    } else if (!paired_inband_now) {
      paired_dwell_ = 0.0;      // 任一侧掉出在位 → 双侧“连续在位”不复存在
      paired_last_ = -1.0;
    }
    // 若 paired_inband_now 但本帧不是两侧都新鲜（暂停帧），dwell/paired_last_ 均
    // 保持：只要两侧仍在 active_inband 且下一对新鲜帧间隔 <= 预算，就接着累计。

    // 确认：墙钟过 base + 双侧配对连续在位 >= confirm。
    if (paired_dwell_ >= params_.confirm_seconds &&
      wall_elapsed_ >= params_.base_seconds &&
      left_.active_inband && right_.active_inband)
    {
      return set_status(SeatConfirmStatus::kConfirmed);
    }
    if (wall_elapsed_ > params_.max_seconds) {
      // 安全硬上限：即便持续缺数也不能无限等。区分原因交给 Describe()。
      return set_status(SeatConfirmStatus::kFailWindowBudget);
    }
    return SeatConfirmStatus::kPending;
  }

  SeatConfirmStatus status() const { return status_; }

  // ── 诊断（供调用方拼 NOT_READY 消息，明确区分失败/未决原因）──────────
  double wall_elapsed() const { return wall_elapsed_; }
  double paired_dwell() const { return paired_dwell_; }
  double left_fresh_age(double now) const { return left_.last_fresh >= 0.0 ? now - left_.last_fresh : -1.0; }
  double right_fresh_age(double now) const { return right_.last_fresh >= 0.0 ? now - right_.last_fresh : -1.0; }
  std::uint64_t left_samples() const { return left_.samples; }
  std::uint64_t right_samples() const { return right_.samples; }
  std::uint64_t left_seq() const { return left_.fresh_seq; }
  std::uint64_t right_seq() const { return right_.fresh_seq; }
  double left_slip() const { return left_.slip_dwell; }
  double right_slip() const { return right_.slip_dwell; }
  bool left_active_inband() const { return left_.active_inband; }
  bool right_active_inband() const { return right_.active_inband; }

  // 简洁原因字符串：区分 kPending 下的「持续缺数 / 采样过旧 / 机械振荡 / 未达
  // 最短观察」，及终态的超界 / 掉带 / 硬上限。now 为当前墙钟秒数。
  std::string Describe(double now) const
  {
    switch (status_) {
      case SeatConfirmStatus::kConfirmed:
        return "confirmed: paired in-band dwell " +
          fmt(paired_dwell_) + " s >= " + fmt(params_.confirm_seconds) + " s";
      case SeatConfirmStatus::kFailTravelUpLeft:
        return "left travel exceeded the up band";
      case SeatConfirmStatus::kFailTravelUpRight:
        return "right travel exceeded the up band";
      case SeatConfirmStatus::kFailSlipDownLeft:
        return "left stayed below the down band for " + fmt(left_.slip_dwell) +
          " s >= " + fmt(params_.slip_max_seconds) + " s (sustained slip)";
      case SeatConfirmStatus::kFailSlipDownRight:
        return "right stayed below the down band for " + fmt(right_.slip_dwell) +
          " s >= " + fmt(params_.slip_max_seconds) + " s (sustained slip)";
      case SeatConfirmStatus::kFailWindowBudget: {
        // 硬上限未确认：按样本年龄区分「持续缺数/采样过旧」与「机械振荡」。
        const double l_age = left_fresh_age(now);
        const double r_age = right_fresh_age(now);
        const bool stale = (l_age < 0.0 || r_age < 0.0) ||
          l_age > params_.fresh_budget_seconds ||
          r_age > params_.fresh_budget_seconds;
        if (stale) {
          return "no confirmation within the " + fmt(params_.max_seconds) +
            " s wall budget due to continuous missing/stale samples (left "
            "fresh age " + fmt(l_age) + " s, right fresh age " + fmt(r_age) +
            " s > budget " + fmt(params_.fresh_budget_seconds) + " s)";
        }
        return "no confirmation within the " + fmt(params_.max_seconds) +
          " s wall budget due to mechanical chatter (paired in-band dwell " +
          fmt(paired_dwell_) + " s < " + fmt(params_.confirm_seconds) + " s)";
      }
      case SeatConfirmStatus::kPending: {
        const double l_age = left_fresh_age(now);
        const double r_age = right_fresh_age(now);
        const bool stale = (l_age > params_.fresh_budget_seconds) ||
          (r_age > params_.fresh_budget_seconds) || l_age < 0.0 || r_age < 0.0;
        std::string reason;
        if (stale) {
          reason = "missing/stale samples (left age " + fmt(l_age) +
            " s, right age " + fmt(r_age) + " s)";
        } else if (wall_elapsed_ < params_.base_seconds) {
          reason = "still in the " + fmt(params_.base_seconds) +
            " s minimum passive observation";
        } else {
          reason = "mechanical chatter / not both sides continuously in band "
            "(paired dwell " + fmt(paired_dwell_) + " s < " +
            fmt(params_.confirm_seconds) + " s)";
        }
        return "pending: " + reason;
      }
      default:
        return "unknown";
    }
  }

private:
  struct SideState
  {
    // 最近一次新鲜采样是否在带（用于配对 in-band 判定）。
    bool fresh_inband_this_frame{false};
    // 该侧是否处于“active in-band”：最近一次新鲜样本在带，且其后缺数未越预算。
    bool active_inband{false};
    bool last_fresh_below{false};   // 上一个新鲜样本是否低于下带（掉带连续段用）
    double slip_dwell{0.0};         // 当前连续“新鲜掉带”段时长（s）
    double last_fresh{-1.0};        // 最近一次新鲜样本时间（-1 = 尚无）
    double last_inband_fresh{-1.0}; // 最近一次新鲜在带样本时间
    std::uint64_t fresh_seq{0};
    std::uint64_t samples{0};

    void reset()
    {
      fresh_inband_this_frame = false;
      active_inband = false;
      last_fresh_below = false;
      slip_dwell = 0.0;
      last_fresh = -1.0;
      last_inband_fresh = -1.0;
      fresh_seq = 0;
      samples = 0;
    }
  };

  static std::string fmt(double value)
  {
    char buf[48];
    const auto written = std::snprintf(buf, sizeof(buf), "%.4f", value);
    return std::string(buf, written > 0 ? static_cast<std::size_t>(written) : 0U);
  }

  // 处理一侧的新鲜样本并更新该侧状态；返回该侧即时失败（超界）或 kPending。
  SeatConfirmStatus process_side(
    double now, const SeatConfirmSideFrame & frame, bool is_left, SideState & side)
  {
    if (!frame.present) {
      // dropout：不产生新读数。active in-band 若因缺数超过预算则打断（active 清零）。
      // 这里只处理“仍在 active 却已超预算”的过期情形；下一次新鲜样本到达时也会
      // 用 gap 打断/重计，二者都覆盖 doc 的「缺数超过采样预算即打断连续计时」。
      if (side.active_inband && side.last_inband_fresh >= 0.0 &&
        (now - side.last_inband_fresh) > params_.fresh_budget_seconds)
      {
        side.active_inband = false;
      }
      return SeatConfirmStatus::kPending;
    }

    ++side.samples;
    const bool first_fresh = side.last_fresh < 0.0;
    const double gap = first_fresh ? -1.0 : (now - side.last_fresh);
    side.last_fresh = now;
    ++side.fresh_seq;
    if (gap >= 0.0 && gap > params_.fresh_budget_seconds) {
      // 断数超过采样预算：打断该侧在位连续段与掉带连续段。
      side.active_inband = false;
      side.last_fresh_below = false;
      side.slip_dwell = 0.0;
    }

    if (frame.travel > frame.seat + params_.up_band_m) {
      // 超界（顶出/冲出）：新鲜样本即败，不累计。
      side.fresh_inband_this_frame = false;
      return is_left ? SeatConfirmStatus::kFailTravelUpLeft :
        SeatConfirmStatus::kFailTravelUpRight;
    }
    if (frame.travel >= frame.seat - params_.down_band_m) {
      // 在带。
      side.fresh_inband_this_frame = true;
      side.active_inband = true;
      side.last_inband_fresh = now;
      side.last_fresh_below = false;
      side.slip_dwell = 0.0;  // 回到在带 → 掉带连续段结束
      return SeatConfirmStatus::kPending;
    }
    // 低于下带（掉带）。
    side.fresh_inband_this_frame = false;
    side.active_inband = false;
    if (!first_fresh && gap >= 0.0 && gap <= params_.fresh_budget_seconds &&
      side.last_fresh_below)
    {
      side.slip_dwell += gap;  // 与上一新鲜样本同为掉带：真实持续回缩证据
    } else {
      side.slip_dwell = 0.0;   // 掉带段起点，或断数后重计
    }
    side.last_fresh_below = true;
    if (side.slip_dwell >= params_.slip_max_seconds) {
      return is_left ? SeatConfirmStatus::kFailSlipDownLeft :
        SeatConfirmStatus::kFailSlipDownRight;
    }
    return SeatConfirmStatus::kPending;
  }

  SeatConfirmStatus set_status(SeatConfirmStatus status)
  {
    status_ = status;
    return status_;
  }

  SeatConfirmParams params_;
  double start_{0.0};
  double last_frame_{0.0};
  double wall_elapsed_{0.0};
  double paired_dwell_{0.0};  // 双侧配对连续在位时长（s）
  double paired_last_{-1.0};  // 上一配对在位帧时间（-1 = 无活动段）
  SideState left_;
  SideState right_;
  SeatConfirmStatus status_{SeatConfirmStatus::kPending};
};

}  // namespace xczs_inspection_robot_control
