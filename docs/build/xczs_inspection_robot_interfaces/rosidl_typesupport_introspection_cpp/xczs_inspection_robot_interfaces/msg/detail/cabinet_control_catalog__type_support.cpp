// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlCatalog.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_catalog__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace xczs_inspection_robot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CabinetControlCatalog_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) xczs_inspection_robot_interfaces::msg::CabinetControlCatalog(_init);
}

void CabinetControlCatalog_fini_function(void * message_memory)
{
  auto typed_message = static_cast<xczs_inspection_robot_interfaces::msg::CabinetControlCatalog *>(message_memory);
  typed_message->~CabinetControlCatalog();
}

size_t size_function__CabinetControlCatalog__controls(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<xczs_inspection_robot_interfaces::msg::CabinetControl> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CabinetControlCatalog__controls(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<xczs_inspection_robot_interfaces::msg::CabinetControl> *>(untyped_member);
  return &member[index];
}

void * get_function__CabinetControlCatalog__controls(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<xczs_inspection_robot_interfaces::msg::CabinetControl> *>(untyped_member);
  return &member[index];
}

void fetch_function__CabinetControlCatalog__controls(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const xczs_inspection_robot_interfaces::msg::CabinetControl *>(
    get_const_function__CabinetControlCatalog__controls(untyped_member, index));
  auto & value = *reinterpret_cast<xczs_inspection_robot_interfaces::msg::CabinetControl *>(untyped_value);
  value = item;
}

void assign_function__CabinetControlCatalog__controls(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<xczs_inspection_robot_interfaces::msg::CabinetControl *>(
    get_function__CabinetControlCatalog__controls(untyped_member, index));
  const auto & value = *reinterpret_cast<const xczs_inspection_robot_interfaces::msg::CabinetControl *>(untyped_value);
  item = value;
}

void resize_function__CabinetControlCatalog__controls(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<xczs_inspection_robot_interfaces::msg::CabinetControl> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CabinetControlCatalog_message_member_array[1] = {
  {
    "controls",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<xczs_inspection_robot_interfaces::msg::CabinetControl>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces::msg::CabinetControlCatalog, controls),  // bytes offset in struct
    nullptr,  // default value
    size_function__CabinetControlCatalog__controls,  // size() function pointer
    get_const_function__CabinetControlCatalog__controls,  // get_const(index) function pointer
    get_function__CabinetControlCatalog__controls,  // get(index) function pointer
    fetch_function__CabinetControlCatalog__controls,  // fetch(index, &value) function pointer
    assign_function__CabinetControlCatalog__controls,  // assign(index, value) function pointer
    resize_function__CabinetControlCatalog__controls  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CabinetControlCatalog_message_members = {
  "xczs_inspection_robot_interfaces::msg",  // message namespace
  "CabinetControlCatalog",  // message name
  1,  // number of fields
  sizeof(xczs_inspection_robot_interfaces::msg::CabinetControlCatalog),
  CabinetControlCatalog_message_member_array,  // message members
  CabinetControlCatalog_init_function,  // function to initialize message memory (memory has to be allocated)
  CabinetControlCatalog_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CabinetControlCatalog_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CabinetControlCatalog_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace xczs_inspection_robot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>()
{
  return &::xczs_inspection_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::CabinetControlCatalog_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, xczs_inspection_robot_interfaces, msg, CabinetControlCatalog)() {
  return &::xczs_inspection_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::CabinetControlCatalog_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
