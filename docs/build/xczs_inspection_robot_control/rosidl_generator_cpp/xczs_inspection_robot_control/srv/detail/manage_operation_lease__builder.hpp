// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_control:srv/ManageOperationLease.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_control/srv/detail/manage_operation_lease__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_control
{

namespace srv
{

namespace builder
{

class Init_ManageOperationLease_Request_requested_duration
{
public:
  explicit Init_ManageOperationLease_Request_requested_duration(::xczs_inspection_robot_control::srv::ManageOperationLease_Request & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Request requested_duration(::xczs_inspection_robot_control::srv::ManageOperationLease_Request::_requested_duration_type arg)
  {
    msg_.requested_duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Request msg_;
};

class Init_ManageOperationLease_Request_lease_id
{
public:
  explicit Init_ManageOperationLease_Request_lease_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Request & msg)
  : msg_(msg)
  {}
  Init_ManageOperationLease_Request_requested_duration lease_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Request::_lease_id_type arg)
  {
    msg_.lease_id = std::move(arg);
    return Init_ManageOperationLease_Request_requested_duration(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Request msg_;
};

class Init_ManageOperationLease_Request_owner_id
{
public:
  explicit Init_ManageOperationLease_Request_owner_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Request & msg)
  : msg_(msg)
  {}
  Init_ManageOperationLease_Request_lease_id owner_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Request::_owner_id_type arg)
  {
    msg_.owner_id = std::move(arg);
    return Init_ManageOperationLease_Request_lease_id(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Request msg_;
};

class Init_ManageOperationLease_Request_command
{
public:
  Init_ManageOperationLease_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManageOperationLease_Request_owner_id command(::xczs_inspection_robot_control::srv::ManageOperationLease_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_ManageOperationLease_Request_owner_id(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::srv::ManageOperationLease_Request>()
{
  return xczs_inspection_robot_control::srv::builder::Init_ManageOperationLease_Request_command();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace srv
{

namespace builder
{

class Init_ManageOperationLease_Response_remaining_duration
{
public:
  explicit Init_ManageOperationLease_Response_remaining_duration(::xczs_inspection_robot_control::srv::ManageOperationLease_Response & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Response remaining_duration(::xczs_inspection_robot_control::srv::ManageOperationLease_Response::_remaining_duration_type arg)
  {
    msg_.remaining_duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Response msg_;
};

class Init_ManageOperationLease_Response_owner_id
{
public:
  explicit Init_ManageOperationLease_Response_owner_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Response & msg)
  : msg_(msg)
  {}
  Init_ManageOperationLease_Response_remaining_duration owner_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Response::_owner_id_type arg)
  {
    msg_.owner_id = std::move(arg);
    return Init_ManageOperationLease_Response_remaining_duration(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Response msg_;
};

class Init_ManageOperationLease_Response_lease_id
{
public:
  explicit Init_ManageOperationLease_Response_lease_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Response & msg)
  : msg_(msg)
  {}
  Init_ManageOperationLease_Response_owner_id lease_id(::xczs_inspection_robot_control::srv::ManageOperationLease_Response::_lease_id_type arg)
  {
    msg_.lease_id = std::move(arg);
    return Init_ManageOperationLease_Response_owner_id(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Response msg_;
};

class Init_ManageOperationLease_Response_message
{
public:
  explicit Init_ManageOperationLease_Response_message(::xczs_inspection_robot_control::srv::ManageOperationLease_Response & msg)
  : msg_(msg)
  {}
  Init_ManageOperationLease_Response_lease_id message(::xczs_inspection_robot_control::srv::ManageOperationLease_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ManageOperationLease_Response_lease_id(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Response msg_;
};

class Init_ManageOperationLease_Response_status_code
{
public:
  explicit Init_ManageOperationLease_Response_status_code(::xczs_inspection_robot_control::srv::ManageOperationLease_Response & msg)
  : msg_(msg)
  {}
  Init_ManageOperationLease_Response_message status_code(::xczs_inspection_robot_control::srv::ManageOperationLease_Response::_status_code_type arg)
  {
    msg_.status_code = std::move(arg);
    return Init_ManageOperationLease_Response_message(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Response msg_;
};

class Init_ManageOperationLease_Response_success
{
public:
  Init_ManageOperationLease_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManageOperationLease_Response_status_code success(::xczs_inspection_robot_control::srv::ManageOperationLease_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ManageOperationLease_Response_status_code(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::ManageOperationLease_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::srv::ManageOperationLease_Response>()
{
  return xczs_inspection_robot_control::srv::builder::Init_ManageOperationLease_Response_success();
}

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__BUILDER_HPP_
