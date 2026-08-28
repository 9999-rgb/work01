// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_control:srv/SetCabinetGrasp.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_control
{

namespace srv
{

namespace builder
{

class Init_SetCabinetGrasp_Request_attach
{
public:
  explicit Init_SetCabinetGrasp_Request_attach(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request attach(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request::_attach_type arg)
  {
    msg_.attach = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request msg_;
};

class Init_SetCabinetGrasp_Request_robot_base_link
{
public:
  explicit Init_SetCabinetGrasp_Request_robot_base_link(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg)
  : msg_(msg)
  {}
  Init_SetCabinetGrasp_Request_attach robot_base_link(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request::_robot_base_link_type arg)
  {
    msg_.robot_base_link = std::move(arg);
    return Init_SetCabinetGrasp_Request_attach(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request msg_;
};

class Init_SetCabinetGrasp_Request_robot_grasp_point
{
public:
  explicit Init_SetCabinetGrasp_Request_robot_grasp_point(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg)
  : msg_(msg)
  {}
  Init_SetCabinetGrasp_Request_robot_base_link robot_grasp_point(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request::_robot_grasp_point_type arg)
  {
    msg_.robot_grasp_point = std::move(arg);
    return Init_SetCabinetGrasp_Request_robot_base_link(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request msg_;
};

class Init_SetCabinetGrasp_Request_robot_link
{
public:
  explicit Init_SetCabinetGrasp_Request_robot_link(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg)
  : msg_(msg)
  {}
  Init_SetCabinetGrasp_Request_robot_grasp_point robot_link(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request::_robot_link_type arg)
  {
    msg_.robot_link = std::move(arg);
    return Init_SetCabinetGrasp_Request_robot_grasp_point(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request msg_;
};

class Init_SetCabinetGrasp_Request_robot_model
{
public:
  explicit Init_SetCabinetGrasp_Request_robot_model(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg)
  : msg_(msg)
  {}
  Init_SetCabinetGrasp_Request_robot_link robot_model(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request::_robot_model_type arg)
  {
    msg_.robot_model = std::move(arg);
    return Init_SetCabinetGrasp_Request_robot_link(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request msg_;
};

class Init_SetCabinetGrasp_Request_operation_lease_id
{
public:
  explicit Init_SetCabinetGrasp_Request_operation_lease_id(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg)
  : msg_(msg)
  {}
  Init_SetCabinetGrasp_Request_robot_model operation_lease_id(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request::_operation_lease_id_type arg)
  {
    msg_.operation_lease_id = std::move(arg);
    return Init_SetCabinetGrasp_Request_robot_model(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request msg_;
};

class Init_SetCabinetGrasp_Request_control_id
{
public:
  Init_SetCabinetGrasp_Request_control_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetCabinetGrasp_Request_operation_lease_id control_id(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request::_control_id_type arg)
  {
    msg_.control_id = std::move(arg);
    return Init_SetCabinetGrasp_Request_operation_lease_id(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>()
{
  return xczs_inspection_robot_control::srv::builder::Init_SetCabinetGrasp_Request_control_id();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace srv
{

namespace builder
{

class Init_SetCabinetGrasp_Response_distance
{
public:
  explicit Init_SetCabinetGrasp_Response_distance(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response distance(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response msg_;
};

class Init_SetCabinetGrasp_Response_message
{
public:
  explicit Init_SetCabinetGrasp_Response_message(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response & msg)
  : msg_(msg)
  {}
  Init_SetCabinetGrasp_Response_distance message(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_SetCabinetGrasp_Response_distance(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response msg_;
};

class Init_SetCabinetGrasp_Response_success
{
public:
  Init_SetCabinetGrasp_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetCabinetGrasp_Response_message success(::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetCabinetGrasp_Response_message(msg_);
  }

private:
  ::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>()
{
  return xczs_inspection_robot_control::srv::builder::Init_SetCabinetGrasp_Response_success();
}

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__BUILDER_HPP_
