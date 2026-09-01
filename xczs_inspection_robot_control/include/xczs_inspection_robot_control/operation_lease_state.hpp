// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__OPERATION_LEASE_STATE_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__OPERATION_LEASE_STATE_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

namespace xczs_inspection_robot_control
{

enum class OperationLeaseStatus : std::uint8_t
{
  GRANTED = 0,
  RESOURCE_BUSY = 1,
  INVALID_REQUEST = 2,
  NOT_OWNER = 3,
  RELEASED = 4,
};

struct OperationLeaseReply
{
  bool success{false};
  OperationLeaseStatus status{OperationLeaseStatus::INVALID_REQUEST};
  std::string message;
  std::string lease_id;
  std::string owner_id;
  double remaining_duration{0.0};
};

class OperationLeaseState
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Duration = std::chrono::duration<double>;

  OperationLeaseReply acquire(
    const std::string & owner_id,
    Duration duration,
    TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expire_locked(now);
    if (!valid_request(owner_id, duration)) {
      return invalid_request();
    }
    if (!lease_id_.empty()) {
      auto reply = reply_locked(
        false, OperationLeaseStatus::RESOURCE_BUSY,
        "The global robot operation resource is already leased.", now);
      reply.lease_id.clear();
      return reply;
    }

    owner_id_ = owner_id;
    lease_id_ = std::to_string(++sequence_) + "-" +
      std::to_string(now.time_since_epoch().count());
    expires_at_ = now +
      std::chrono::duration_cast<Clock::duration>(duration);
    return reply_locked(
      true, OperationLeaseStatus::GRANTED,
      "The global robot operation lease was granted.", now);
  }

  OperationLeaseReply renew(
    const std::string & owner_id,
    const std::string & lease_id,
    Duration duration,
    TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expire_locked(now);
    if (!valid_request(owner_id, duration) || lease_id.empty()) {
      return invalid_request();
    }
    if (owner_id != owner_id_ || lease_id != lease_id_) {
      auto reply = reply_locked(
        false, OperationLeaseStatus::NOT_OWNER,
        "The operation lease no longer belongs to this client.", now);
      reply.lease_id.clear();
      return reply;
    }

    expires_at_ = now +
      std::chrono::duration_cast<Clock::duration>(duration);
    return reply_locked(
      true, OperationLeaseStatus::GRANTED,
      "The global robot operation lease was renewed.", now);
  }

  OperationLeaseReply release(
    const std::string & owner_id,
    const std::string & lease_id,
    TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expire_locked(now);
    if (owner_id.empty() || lease_id.empty()) {
      return invalid_request();
    }
    if (owner_id != owner_id_ || lease_id != lease_id_) {
      auto reply = reply_locked(
        false, OperationLeaseStatus::NOT_OWNER,
        "The operation lease no longer belongs to this client.", now);
      reply.lease_id.clear();
      return reply;
    }

    const std::string released_lease_id = lease_id_;
    const std::string released_owner_id = owner_id_;
    clear_locked();
    OperationLeaseReply reply;
    reply.success = true;
    reply.status = OperationLeaseStatus::RELEASED;
    reply.message = "The global robot operation lease was released.";
    reply.lease_id = released_lease_id;
    reply.owner_id = released_owner_id;
    return reply;
  }

  bool expire(TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return expire_locked(now);
  }

  // Debug accessors (temporary diagnostics).
  std::string held_owner()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return owner_id_;
  }

  std::string held_lease()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return lease_id_;
  }

private:
  static bool valid_request(
    const std::string & owner_id,
    Duration duration)
  {
    if (owner_id.empty() || !std::isfinite(duration.count()) ||
        duration.count() <= 0.0)
    {
      return false;
    }
    // 正时长若向下取整到 0 个时钟 tick，会授予一个「已过期」的租约
    // （expires_at_ 等于 now）；要求至少一个 tick，避免 sub-tick 时长
    // 让 acquire/renew 立即失效、破坏互斥语义。
    return std::chrono::duration_cast<Clock::duration>(duration).count() >= 1;
  }

  static OperationLeaseReply invalid_request()
  {
    OperationLeaseReply reply;
    reply.status = OperationLeaseStatus::INVALID_REQUEST;
    reply.message =
      "Lease owner and a finite positive duration are required.";
    return reply;
  }

  OperationLeaseReply reply_locked(
    bool success,
    OperationLeaseStatus status,
    const std::string & message,
    TimePoint now) const
  {
    OperationLeaseReply reply;
    reply.success = success;
    reply.status = status;
    reply.message = message;
    reply.lease_id = lease_id_;
    reply.owner_id = owner_id_;
    if (!lease_id_.empty() && expires_at_ > now) {
      reply.remaining_duration =
        std::chrono::duration<double>(expires_at_ - now).count();
    }
    return reply;
  }

  bool expire_locked(TimePoint now)
  {
    if (lease_id_.empty() || now < expires_at_) {
      return false;
    }
    clear_locked();
    return true;
  }

  void clear_locked()
  {
    owner_id_.clear();
    lease_id_.clear();
    expires_at_ = TimePoint{};
  }

  std::mutex mutex_;
  std::string owner_id_;
  std::string lease_id_;
  TimePoint expires_at_{};
  std::uint64_t sequence_{0};
};

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__OPERATION_LEASE_STATE_HPP_
