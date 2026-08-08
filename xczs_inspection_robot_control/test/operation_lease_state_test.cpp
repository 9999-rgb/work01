// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <atomic>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "xczs_inspection_robot_control/operation_lease_state.hpp"

namespace xczs_inspection_robot_control
{

using namespace std::chrono_literals;

TEST(OperationLeaseState, ExcludesOtherOwnersAndAllowsRelease)
{
  OperationLeaseState state;
  const auto now = OperationLeaseState::TimePoint{} + 1s;
  const auto first = state.acquire("cabinet_a", 3s, now);
  ASSERT_TRUE(first.success);
  ASSERT_FALSE(first.lease_id.empty());

  const auto busy = state.acquire("cabinet_b", 3s, now + 100ms);
  EXPECT_FALSE(busy.success);
  EXPECT_EQ(busy.status, OperationLeaseStatus::RESOURCE_BUSY);
  EXPECT_EQ(busy.owner_id, "cabinet_a");
  EXPECT_TRUE(busy.lease_id.empty());

  const auto released = state.release(
    "cabinet_a", first.lease_id, now + 200ms);
  EXPECT_TRUE(released.success);
  EXPECT_EQ(released.status, OperationLeaseStatus::RELEASED);
  EXPECT_TRUE(state.acquire("cabinet_b", 3s, now + 300ms).success);
}

TEST(OperationLeaseState, ConcurrentAcquireHasExactlyOneWinner)
{
  constexpr std::size_t client_count = 16U;
  OperationLeaseState state;
  const auto now = OperationLeaseState::TimePoint{} + 1s;
  std::atomic<bool> start{false};
  std::vector<OperationLeaseReply> replies(client_count);
  std::vector<std::thread> workers;
  workers.reserve(client_count);
  for (std::size_t index = 0U; index < client_count; ++index) {
    workers.emplace_back(
      [&state, &start, &replies, index, now]() {
        while (!start.load()) {
          std::this_thread::yield();
        }
        replies[index] = state.acquire(
          "cabinet_" + std::to_string(index), 3s, now);
      });
  }
  start.store(true);
  for (auto & worker : workers) {
    worker.join();
  }

  std::size_t winners = 0U;
  for (const auto & reply : replies) {
    if (reply.success) {
      ++winners;
      EXPECT_EQ(reply.status, OperationLeaseStatus::GRANTED);
    } else {
      EXPECT_EQ(reply.status, OperationLeaseStatus::RESOURCE_BUSY);
      EXPECT_TRUE(reply.lease_id.empty());
    }
  }
  EXPECT_EQ(winners, 1U);
}

TEST(OperationLeaseState, RenewalExtendsLeaseAndStaleTokenCannotRelease)
{
  OperationLeaseState state;
  const auto now = OperationLeaseState::TimePoint{} + 1s;
  const auto first = state.acquire("cabinet_a", 1s, now);
  ASSERT_TRUE(first.success);
  EXPECT_TRUE(
    state.renew("cabinet_a", first.lease_id, 2s, now + 500ms).success);
  EXPECT_FALSE(state.expire(now + 2s));

  const auto stale_release = state.release(
    "cabinet_b", first.lease_id, now + 2100ms);
  EXPECT_FALSE(stale_release.success);
  EXPECT_EQ(stale_release.status, OperationLeaseStatus::NOT_OWNER);
  EXPECT_TRUE(stale_release.lease_id.empty());
  EXPECT_TRUE(state.expire(now + 2600ms));
}

TEST(OperationLeaseState, ExpiredOwnerCannotRenewAndResourceRecovers)
{
  OperationLeaseState state;
  const auto now = OperationLeaseState::TimePoint{} + 1s;
  const auto first = state.acquire("cabinet_a", 1s, now);
  ASSERT_TRUE(first.success);

  const auto expired_renew = state.renew(
    "cabinet_a", first.lease_id, 1s, now + 1100ms);
  EXPECT_FALSE(expired_renew.success);
  EXPECT_EQ(expired_renew.status, OperationLeaseStatus::NOT_OWNER);
  EXPECT_TRUE(state.acquire("cabinet_b", 1s, now + 1100ms).success);
}

}  // namespace xczs_inspection_robot_control
