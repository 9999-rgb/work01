// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "xczs_inspection_robot_control/operation_lease_state.hpp"
#include "xczs_inspection_robot_interfaces/srv/manage_operation_lease.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using ManageOperationLease =
  xczs_inspection_robot_interfaces::srv::ManageOperationLease;

}  // namespace

class OperationLeaseCoordinator final : public rclcpp::Node
{
public:
  OperationLeaseCoordinator()
  : Node("xczs_operation_lease_coordinator")
  {
    const auto service_name = declare_parameter<std::string>(
      "service_name", "/xczs/operation_lease");
    maximum_lease_duration_ = declare_parameter<double>(
      "maximum_lease_duration", 5.0);
    if (service_name.empty() || !std::isfinite(maximum_lease_duration_) ||
      maximum_lease_duration_ <= 0.0)
    {
      throw std::invalid_argument(
              "A service name and positive maximum lease duration are required.");
    }

    service_ = create_service<ManageOperationLease>(
      service_name,
      [this](
        const std::shared_ptr<ManageOperationLease::Request> request,
        std::shared_ptr<ManageOperationLease::Response> response)
      {
        handle_request(*request, *response);
      });
    expiry_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (state_.expire(OperationLeaseState::Clock::now())) {
          RCLCPP_WARN(
            get_logger(),
            "An abandoned global operation lease expired safely.");
        }
      });
    RCLCPP_INFO(
      get_logger(), "Global robot operation lease service is ready at %s.",
      service_name.c_str());
  }

private:
  void handle_request(
    const ManageOperationLease::Request & request,
    ManageOperationLease::Response & response)
  {
    OperationLeaseReply reply;
    const auto now = OperationLeaseState::Clock::now();
    const OperationLeaseState::Duration duration(request.requested_duration);
    const bool duration_valid = std::isfinite(request.requested_duration) &&
      request.requested_duration > 0.0 &&
      request.requested_duration <= maximum_lease_duration_;

    if ((request.command == ManageOperationLease::Request::ACQUIRE ||
      request.command == ManageOperationLease::Request::RENEW) &&
      !duration_valid)
    {
      reply.status = OperationLeaseStatus::INVALID_REQUEST;
      reply.message = "Requested lease duration is outside the allowed range.";
    } else if (request.command == ManageOperationLease::Request::ACQUIRE) {
      reply = state_.acquire(request.owner_id, duration, now);
    } else if (request.command == ManageOperationLease::Request::RENEW) {
      reply = state_.renew(
        request.owner_id, request.lease_id, duration, now);
    } else if (request.command == ManageOperationLease::Request::RELEASE) {
      reply = state_.release(request.owner_id, request.lease_id, now);
    } else {
      reply.status = OperationLeaseStatus::INVALID_REQUEST;
      reply.message = "Unknown lease command.";
    }

    response.success = reply.success;
    response.status_code = static_cast<std::uint8_t>(reply.status);
    response.message = reply.message;
    response.lease_id = reply.lease_id;
    response.owner_id = reply.owner_id;
    response.remaining_duration = reply.remaining_duration;
    RCLCPP_INFO(
      get_logger(),
      "[dbg] req cmd=%d owner='%s' lease='%s' dur=%.2f -> status=%u rem=%.2f "
      "held_owner='%s' held_lease='%s' msg='%s'",
      request.command, request.owner_id.c_str(), request.lease_id.c_str(),
      request.requested_duration, static_cast<unsigned>(reply.status),
      reply.remaining_duration, state_owner_for_log().c_str(),
      state_lease_for_log().c_str(), reply.message.c_str());
  }

  double maximum_lease_duration_{5.0};
  OperationLeaseState state_;

  std::string state_owner_for_log() { return state_.held_owner(); }
  std::string state_lease_for_log() { return state_.held_lease(); }
  rclcpp::Service<ManageOperationLease>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
};

}  // namespace xczs_inspection_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      xczs_inspection_robot_control::OperationLeaseCoordinator>());
  rclcpp::shutdown();
  return 0;
}
