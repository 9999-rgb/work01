// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlCatalog.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_catalog__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'controls'
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CabinetControlCatalog & msg,
  std::ostream & out)
{
  out << "{";
  // member: controls
  {
    if (msg.controls.size() == 0) {
      out << "controls: []";
    } else {
      out << "controls: [";
      size_t pending_items = msg.controls.size();
      for (auto item : msg.controls) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CabinetControlCatalog & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: controls
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.controls.size() == 0) {
      out << "controls: []\n";
    } else {
      out << "controls:\n";
      for (auto item : msg.controls) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CabinetControlCatalog & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::msg::CabinetControlCatalog & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::msg::CabinetControlCatalog & msg)
{
  return xczs_inspection_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>()
{
  return "xczs_inspection_robot_interfaces::msg::CabinetControlCatalog";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>()
{
  return "xczs_inspection_robot_interfaces/msg/CabinetControlCatalog";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__TRAITS_HPP_
