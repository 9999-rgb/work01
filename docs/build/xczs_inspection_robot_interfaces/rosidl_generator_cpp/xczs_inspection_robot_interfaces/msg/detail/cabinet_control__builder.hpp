// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CabinetControl_max_force
{
public:
  explicit Init_CabinetControl_max_force(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_interfaces::msg::CabinetControl max_force(::xczs_inspection_robot_interfaces::msg::CabinetControl::_max_force_type arg)
  {
    msg_.max_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_min_trigger_force
{
public:
  explicit Init_CabinetControl_min_trigger_force(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_max_force min_trigger_force(::xczs_inspection_robot_interfaces::msg::CabinetControl::_min_trigger_force_type arg)
  {
    msg_.min_trigger_force = std::move(arg);
    return Init_CabinetControl_max_force(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_default_force
{
public:
  explicit Init_CabinetControl_default_force(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_min_trigger_force default_force(::xczs_inspection_robot_interfaces::msg::CabinetControl::_default_force_type arg)
  {
    msg_.default_force = std::move(arg);
    return Init_CabinetControl_min_trigger_force(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_adapter_validated
{
public:
  explicit Init_CabinetControl_adapter_validated(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_default_force adapter_validated(::xczs_inspection_robot_interfaces::msg::CabinetControl::_adapter_validated_type arg)
  {
    msg_.adapter_validated = std::move(arg);
    return Init_CabinetControl_default_force(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_toolset_compatible
{
public:
  explicit Init_CabinetControl_toolset_compatible(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_adapter_validated toolset_compatible(::xczs_inspection_robot_interfaces::msg::CabinetControl::_toolset_compatible_type arg)
  {
    msg_.toolset_compatible = std::move(arg);
    return Init_CabinetControl_adapter_validated(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_required_toolset
{
public:
  explicit Init_CabinetControl_required_toolset(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_toolset_compatible required_toolset(::xczs_inspection_robot_interfaces::msg::CabinetControl::_required_toolset_type arg)
  {
    msg_.required_toolset = std::move(arg);
    return Init_CabinetControl_toolset_compatible(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_unavailable_reason
{
public:
  explicit Init_CabinetControl_unavailable_reason(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_required_toolset unavailable_reason(::xczs_inspection_robot_interfaces::msg::CabinetControl::_unavailable_reason_type arg)
  {
    msg_.unavailable_reason = std::move(arg);
    return Init_CabinetControl_required_toolset(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_operable
{
public:
  explicit Init_CabinetControl_operable(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_unavailable_reason operable(::xczs_inspection_robot_interfaces::msg::CabinetControl::_operable_type arg)
  {
    msg_.operable = std::move(arg);
    return Init_CabinetControl_unavailable_reason(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_requires_grasp
{
public:
  explicit Init_CabinetControl_requires_grasp(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_operable requires_grasp(::xczs_inspection_robot_interfaces::msg::CabinetControl::_requires_grasp_type arg)
  {
    msg_.requires_grasp = std::move(arg);
    return Init_CabinetControl_operable(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_state_positions
{
public:
  explicit Init_CabinetControl_state_positions(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_requires_grasp state_positions(::xczs_inspection_robot_interfaces::msg::CabinetControl::_state_positions_type arg)
  {
    msg_.state_positions = std::move(arg);
    return Init_CabinetControl_requires_grasp(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_state_labels
{
public:
  explicit Init_CabinetControl_state_labels(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_state_positions state_labels(::xczs_inspection_robot_interfaces::msg::CabinetControl::_state_labels_type arg)
  {
    msg_.state_labels = std::move(arg);
    return Init_CabinetControl_state_positions(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_state_ids
{
public:
  explicit Init_CabinetControl_state_ids(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_state_labels state_ids(::xczs_inspection_robot_interfaces::msg::CabinetControl::_state_ids_type arg)
  {
    msg_.state_ids = std::move(arg);
    return Init_CabinetControl_state_labels(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_max_position
{
public:
  explicit Init_CabinetControl_max_position(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_state_ids max_position(::xczs_inspection_robot_interfaces::msg::CabinetControl::_max_position_type arg)
  {
    msg_.max_position = std::move(arg);
    return Init_CabinetControl_state_ids(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_min_position
{
public:
  explicit Init_CabinetControl_min_position(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_max_position min_position(::xczs_inspection_robot_interfaces::msg::CabinetControl::_min_position_type arg)
  {
    msg_.min_position = std::move(arg);
    return Init_CabinetControl_max_position(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_unit
{
public:
  explicit Init_CabinetControl_unit(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_min_position unit(::xczs_inspection_robot_interfaces::msg::CabinetControl::_unit_type arg)
  {
    msg_.unit = std::move(arg);
    return Init_CabinetControl_min_position(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_supported_commands
{
public:
  explicit Init_CabinetControl_supported_commands(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_unit supported_commands(::xczs_inspection_robot_interfaces::msg::CabinetControl::_supported_commands_type arg)
  {
    msg_.supported_commands = std::move(arg);
    return Init_CabinetControl_unit(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_state_topic
{
public:
  explicit Init_CabinetControl_state_topic(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_supported_commands state_topic(::xczs_inspection_robot_interfaces::msg::CabinetControl::_state_topic_type arg)
  {
    msg_.state_topic = std::move(arg);
    return Init_CabinetControl_supported_commands(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_pressed_topic
{
public:
  explicit Init_CabinetControl_pressed_topic(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_state_topic pressed_topic(::xczs_inspection_robot_interfaces::msg::CabinetControl::_pressed_topic_type arg)
  {
    msg_.pressed_topic = std::move(arg);
    return Init_CabinetControl_state_topic(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_joint_state_topic
{
public:
  explicit Init_CabinetControl_joint_state_topic(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_pressed_topic joint_state_topic(::xczs_inspection_robot_interfaces::msg::CabinetControl::_joint_state_topic_type arg)
  {
    msg_.joint_state_topic = std::move(arg);
    return Init_CabinetControl_pressed_topic(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_joint_name
{
public:
  explicit Init_CabinetControl_joint_name(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_joint_state_topic joint_name(::xczs_inspection_robot_interfaces::msg::CabinetControl::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_CabinetControl_joint_state_topic(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_control_type
{
public:
  explicit Init_CabinetControl_control_type(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_joint_name control_type(::xczs_inspection_robot_interfaces::msg::CabinetControl::_control_type_type arg)
  {
    msg_.control_type = std::move(arg);
    return Init_CabinetControl_joint_name(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_display_name
{
public:
  explicit Init_CabinetControl_display_name(::xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
  : msg_(msg)
  {}
  Init_CabinetControl_control_type display_name(::xczs_inspection_robot_interfaces::msg::CabinetControl::_display_name_type arg)
  {
    msg_.display_name = std::move(arg);
    return Init_CabinetControl_control_type(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

class Init_CabinetControl_control_id
{
public:
  Init_CabinetControl_control_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CabinetControl_display_name control_id(::xczs_inspection_robot_interfaces::msg::CabinetControl::_control_id_type arg)
  {
    msg_.control_id = std::move(arg);
    return Init_CabinetControl_display_name(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_interfaces::msg::CabinetControl>()
{
  return xczs_inspection_robot_interfaces::msg::builder::Init_CabinetControl_control_id();
}

}  // namespace xczs_inspection_robot_interfaces

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__BUILDER_HPP_
