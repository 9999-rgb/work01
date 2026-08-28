// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlState.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CabinetControlState_transition_sequence
{
public:
  explicit Init_CabinetControlState_transition_sequence(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState transition_sequence(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_transition_sequence_type arg)
  {
    msg_.transition_sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_in_motion
{
public:
  explicit Init_CabinetControlState_in_motion(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_transition_sequence in_motion(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_in_motion_type arg)
  {
    msg_.in_motion = std::move(arg);
    return Init_CabinetControlState_transition_sequence(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_activated
{
public:
  explicit Init_CabinetControlState_activated(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_in_motion activated(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_activated_type arg)
  {
    msg_.activated = std::move(arg);
    return Init_CabinetControlState_in_motion(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_state_id
{
public:
  explicit Init_CabinetControlState_state_id(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_activated state_id(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_state_id_type arg)
  {
    msg_.state_id = std::move(arg);
    return Init_CabinetControlState_activated(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_normalized_position
{
public:
  explicit Init_CabinetControlState_normalized_position(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_state_id normalized_position(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_normalized_position_type arg)
  {
    msg_.normalized_position = std::move(arg);
    return Init_CabinetControlState_state_id(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_effort
{
public:
  explicit Init_CabinetControlState_effort(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_normalized_position effort(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_CabinetControlState_normalized_position(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_velocity
{
public:
  explicit Init_CabinetControlState_velocity(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_effort velocity(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_CabinetControlState_effort(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_position
{
public:
  explicit Init_CabinetControlState_position(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_velocity position(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_CabinetControlState_velocity(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_valid
{
public:
  explicit Init_CabinetControlState_valid(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_position valid(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_CabinetControlState_position(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_control_type
{
public:
  explicit Init_CabinetControlState_control_type(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_valid control_type(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_control_type_type arg)
  {
    msg_.control_type = std::move(arg);
    return Init_CabinetControlState_valid(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_control_id
{
public:
  explicit Init_CabinetControlState_control_id(::xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
  : msg_(msg)
  {}
  Init_CabinetControlState_control_type control_id(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_control_id_type arg)
  {
    msg_.control_id = std::move(arg);
    return Init_CabinetControlState_control_type(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

class Init_CabinetControlState_header
{
public:
  Init_CabinetControlState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CabinetControlState_control_id header(::xczs_inspection_robot_interfaces::msg::CabinetControlState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CabinetControlState_control_id(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_interfaces::msg::CabinetControlState>()
{
  return xczs_inspection_robot_interfaces::msg::builder::Init_CabinetControlState_header();
}

}  // namespace xczs_inspection_robot_interfaces

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__BUILDER_HPP_
