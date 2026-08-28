// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlCatalog.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_catalog__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CabinetControlCatalog_controls
{
public:
  Init_CabinetControlCatalog_controls()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xczs_inspection_robot_interfaces::msg::CabinetControlCatalog controls(::xczs_inspection_robot_interfaces::msg::CabinetControlCatalog::_controls_type arg)
  {
    msg_.controls = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_interfaces::msg::CabinetControlCatalog msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>()
{
  return xczs_inspection_robot_interfaces::msg::builder::Init_CabinetControlCatalog_controls();
}

}  // namespace xczs_inspection_robot_interfaces

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__BUILDER_HPP_
