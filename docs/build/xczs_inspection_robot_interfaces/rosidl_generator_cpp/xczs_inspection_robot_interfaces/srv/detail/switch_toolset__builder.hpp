// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_interfaces:srv/SwitchToolset.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SwitchToolset_Request_expected_generation
{
public:
  explicit Init_SwitchToolset_Request_expected_generation(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Request & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Request expected_generation(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Request::_expected_generation_type arg)
  {
    msg_.expected_generation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Request msg_;
};

class Init_SwitchToolset_Request_toolset
{
public:
  Init_SwitchToolset_Request_toolset()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwitchToolset_Request_expected_generation toolset(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Request::_toolset_type arg)
  {
    msg_.toolset = std::move(arg);
    return Init_SwitchToolset_Request_expected_generation(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>()
{
  return xczs_inspection_robot_interfaces::srv::builder::Init_SwitchToolset_Request_toolset();
}

}  // namespace xczs_inspection_robot_interfaces


namespace xczs_inspection_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SwitchToolset_Response_message
{
public:
  explicit Init_SwitchToolset_Response_message(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response message(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response msg_;
};

class Init_SwitchToolset_Response_generation
{
public:
  explicit Init_SwitchToolset_Response_generation(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response & msg)
  : msg_(msg)
  {}
  Init_SwitchToolset_Response_message generation(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response::_generation_type arg)
  {
    msg_.generation = std::move(arg);
    return Init_SwitchToolset_Response_message(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response msg_;
};

class Init_SwitchToolset_Response_target_toolset
{
public:
  explicit Init_SwitchToolset_Response_target_toolset(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response & msg)
  : msg_(msg)
  {}
  Init_SwitchToolset_Response_generation target_toolset(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response::_target_toolset_type arg)
  {
    msg_.target_toolset = std::move(arg);
    return Init_SwitchToolset_Response_generation(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response msg_;
};

class Init_SwitchToolset_Response_active_toolset
{
public:
  explicit Init_SwitchToolset_Response_active_toolset(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response & msg)
  : msg_(msg)
  {}
  Init_SwitchToolset_Response_target_toolset active_toolset(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response::_active_toolset_type arg)
  {
    msg_.active_toolset = std::move(arg);
    return Init_SwitchToolset_Response_target_toolset(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response msg_;
};

class Init_SwitchToolset_Response_state
{
public:
  explicit Init_SwitchToolset_Response_state(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response & msg)
  : msg_(msg)
  {}
  Init_SwitchToolset_Response_active_toolset state(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_SwitchToolset_Response_active_toolset(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response msg_;
};

class Init_SwitchToolset_Response_accepted
{
public:
  Init_SwitchToolset_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwitchToolset_Response_state accepted(::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SwitchToolset_Response_state(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>()
{
  return xczs_inspection_robot_interfaces::srv::builder::Init_SwitchToolset_Response_accepted();
}

}  // namespace xczs_inspection_robot_interfaces

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__BUILDER_HPP_
