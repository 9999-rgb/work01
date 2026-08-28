// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from xczs_inspection_robot_interfaces:srv/ManageOperationLease.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_interfaces/srv/detail/manage_operation_lease__rosidl_typesupport_fastrtps_cpp.hpp"
#include "xczs_inspection_robot_interfaces/srv/detail/manage_operation_lease__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
cdr_serialize(
  const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: command
  cdr << ros_message.command;
  // Member: owner_id
  cdr << ros_message.owner_id;
  // Member: lease_id
  cdr << ros_message.lease_id;
  // Member: requested_duration
  cdr << ros_message.requested_duration;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request & ros_message)
{
  // Member: command
  cdr >> ros_message.command;

  // Member: owner_id
  cdr >> ros_message.owner_id;

  // Member: lease_id
  cdr >> ros_message.lease_id;

  // Member: requested_duration
  cdr >> ros_message.requested_duration;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
get_serialized_size(
  const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: command
  {
    size_t item_size = sizeof(ros_message.command);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: owner_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.owner_id.size() + 1);
  // Member: lease_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.lease_id.size() + 1);
  // Member: requested_duration
  {
    size_t item_size = sizeof(ros_message.requested_duration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
max_serialized_size_ManageOperationLease_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: command
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: owner_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: lease_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: requested_duration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request;
    is_plain =
      (
      offsetof(DataType, requested_duration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ManageOperationLease_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ManageOperationLease_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ManageOperationLease_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ManageOperationLease_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ManageOperationLease_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ManageOperationLease_Request__callbacks = {
  "xczs_inspection_robot_interfaces::srv",
  "ManageOperationLease_Request",
  _ManageOperationLease_Request__cdr_serialize,
  _ManageOperationLease_Request__cdr_deserialize,
  _ManageOperationLease_Request__get_serialized_size,
  _ManageOperationLease_Request__max_serialized_size
};

static rosidl_message_type_support_t _ManageOperationLease_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ManageOperationLease_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<xczs_inspection_robot_interfaces::srv::ManageOperationLease_Request>()
{
  return &xczs_inspection_robot_interfaces::srv::typesupport_fastrtps_cpp::_ManageOperationLease_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_interfaces, srv, ManageOperationLease_Request)() {
  return &xczs_inspection_robot_interfaces::srv::typesupport_fastrtps_cpp::_ManageOperationLease_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
cdr_serialize(
  const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: success
  cdr << (ros_message.success ? true : false);
  // Member: status_code
  cdr << ros_message.status_code;
  // Member: message
  cdr << ros_message.message;
  // Member: lease_id
  cdr << ros_message.lease_id;
  // Member: owner_id
  cdr << ros_message.owner_id;
  // Member: remaining_duration
  cdr << ros_message.remaining_duration;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response & ros_message)
{
  // Member: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.success = tmp ? true : false;
  }

  // Member: status_code
  cdr >> ros_message.status_code;

  // Member: message
  cdr >> ros_message.message;

  // Member: lease_id
  cdr >> ros_message.lease_id;

  // Member: owner_id
  cdr >> ros_message.owner_id;

  // Member: remaining_duration
  cdr >> ros_message.remaining_duration;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
get_serialized_size(
  const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: success
  {
    size_t item_size = sizeof(ros_message.success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: status_code
  {
    size_t item_size = sizeof(ros_message.status_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.message.size() + 1);
  // Member: lease_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.lease_id.size() + 1);
  // Member: owner_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.owner_id.size() + 1);
  // Member: remaining_duration
  {
    size_t item_size = sizeof(ros_message.remaining_duration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_interfaces
max_serialized_size_ManageOperationLease_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: status_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: lease_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: owner_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: remaining_duration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response;
    is_plain =
      (
      offsetof(DataType, remaining_duration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ManageOperationLease_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ManageOperationLease_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ManageOperationLease_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ManageOperationLease_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ManageOperationLease_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ManageOperationLease_Response__callbacks = {
  "xczs_inspection_robot_interfaces::srv",
  "ManageOperationLease_Response",
  _ManageOperationLease_Response__cdr_serialize,
  _ManageOperationLease_Response__cdr_deserialize,
  _ManageOperationLease_Response__get_serialized_size,
  _ManageOperationLease_Response__max_serialized_size
};

static rosidl_message_type_support_t _ManageOperationLease_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ManageOperationLease_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<xczs_inspection_robot_interfaces::srv::ManageOperationLease_Response>()
{
  return &xczs_inspection_robot_interfaces::srv::typesupport_fastrtps_cpp::_ManageOperationLease_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_interfaces, srv, ManageOperationLease_Response)() {
  return &xczs_inspection_robot_interfaces::srv::typesupport_fastrtps_cpp::_ManageOperationLease_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _ManageOperationLease__callbacks = {
  "xczs_inspection_robot_interfaces::srv",
  "ManageOperationLease",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_interfaces, srv, ManageOperationLease_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_interfaces, srv, ManageOperationLease_Response)(),
};

static rosidl_service_type_support_t _ManageOperationLease__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ManageOperationLease__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_xczs_inspection_robot_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<xczs_inspection_robot_interfaces::srv::ManageOperationLease>()
{
  return &xczs_inspection_robot_interfaces::srv::typesupport_fastrtps_cpp::_ManageOperationLease__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_interfaces, srv, ManageOperationLease)() {
  return &xczs_inspection_robot_interfaces::srv::typesupport_fastrtps_cpp::_ManageOperationLease__handle;
}

#ifdef __cplusplus
}
#endif
