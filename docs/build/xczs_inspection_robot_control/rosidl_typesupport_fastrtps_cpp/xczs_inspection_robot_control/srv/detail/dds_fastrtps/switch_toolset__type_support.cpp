// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from xczs_inspection_robot_control:srv/SwitchToolset.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_control/srv/detail/switch_toolset__rosidl_typesupport_fastrtps_cpp.hpp"
#include "xczs_inspection_robot_control/srv/detail/switch_toolset__struct.hpp"

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

namespace xczs_inspection_robot_control
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_serialize(
  const xczs_inspection_robot_control::srv::SwitchToolset_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: toolset
  cdr << ros_message.toolset;
  // Member: expected_generation
  cdr << ros_message.expected_generation;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  xczs_inspection_robot_control::srv::SwitchToolset_Request & ros_message)
{
  // Member: toolset
  cdr >> ros_message.toolset;

  // Member: expected_generation
  cdr >> ros_message.expected_generation;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
get_serialized_size(
  const xczs_inspection_robot_control::srv::SwitchToolset_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.toolset.size() + 1);
  // Member: expected_generation
  {
    size_t item_size = sizeof(ros_message.expected_generation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
max_serialized_size_SwitchToolset_Request(
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


  // Member: toolset
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

  // Member: expected_generation
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
    using DataType = xczs_inspection_robot_control::srv::SwitchToolset_Request;
    is_plain =
      (
      offsetof(DataType, expected_generation) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SwitchToolset_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_control::srv::SwitchToolset_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SwitchToolset_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<xczs_inspection_robot_control::srv::SwitchToolset_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SwitchToolset_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_control::srv::SwitchToolset_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SwitchToolset_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SwitchToolset_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SwitchToolset_Request__callbacks = {
  "xczs_inspection_robot_control::srv",
  "SwitchToolset_Request",
  _SwitchToolset_Request__cdr_serialize,
  _SwitchToolset_Request__cdr_deserialize,
  _SwitchToolset_Request__get_serialized_size,
  _SwitchToolset_Request__max_serialized_size
};

static rosidl_message_type_support_t _SwitchToolset_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SwitchToolset_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_control

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_xczs_inspection_robot_control
const rosidl_message_type_support_t *
get_message_type_support_handle<xczs_inspection_robot_control::srv::SwitchToolset_Request>()
{
  return &xczs_inspection_robot_control::srv::typesupport_fastrtps_cpp::_SwitchToolset_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, SwitchToolset_Request)() {
  return &xczs_inspection_robot_control::srv::typesupport_fastrtps_cpp::_SwitchToolset_Request__handle;
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

namespace xczs_inspection_robot_control
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_serialize(
  const xczs_inspection_robot_control::srv::SwitchToolset_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: accepted
  cdr << (ros_message.accepted ? true : false);
  // Member: state
  cdr << ros_message.state;
  // Member: active_toolset
  cdr << ros_message.active_toolset;
  // Member: target_toolset
  cdr << ros_message.target_toolset;
  // Member: generation
  cdr << ros_message.generation;
  // Member: message
  cdr << ros_message.message;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  xczs_inspection_robot_control::srv::SwitchToolset_Response & ros_message)
{
  // Member: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.accepted = tmp ? true : false;
  }

  // Member: state
  cdr >> ros_message.state;

  // Member: active_toolset
  cdr >> ros_message.active_toolset;

  // Member: target_toolset
  cdr >> ros_message.target_toolset;

  // Member: generation
  cdr >> ros_message.generation;

  // Member: message
  cdr >> ros_message.message;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
get_serialized_size(
  const xczs_inspection_robot_control::srv::SwitchToolset_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: accepted
  {
    size_t item_size = sizeof(ros_message.accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: state
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.state.size() + 1);
  // Member: active_toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.active_toolset.size() + 1);
  // Member: target_toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.target_toolset.size() + 1);
  // Member: generation
  {
    size_t item_size = sizeof(ros_message.generation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.message.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
max_serialized_size_SwitchToolset_Response(
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


  // Member: accepted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: state
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

  // Member: active_toolset
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

  // Member: target_toolset
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

  // Member: generation
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = xczs_inspection_robot_control::srv::SwitchToolset_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SwitchToolset_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_control::srv::SwitchToolset_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SwitchToolset_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<xczs_inspection_robot_control::srv::SwitchToolset_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SwitchToolset_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_control::srv::SwitchToolset_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SwitchToolset_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SwitchToolset_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SwitchToolset_Response__callbacks = {
  "xczs_inspection_robot_control::srv",
  "SwitchToolset_Response",
  _SwitchToolset_Response__cdr_serialize,
  _SwitchToolset_Response__cdr_deserialize,
  _SwitchToolset_Response__get_serialized_size,
  _SwitchToolset_Response__max_serialized_size
};

static rosidl_message_type_support_t _SwitchToolset_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SwitchToolset_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_control

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_xczs_inspection_robot_control
const rosidl_message_type_support_t *
get_message_type_support_handle<xczs_inspection_robot_control::srv::SwitchToolset_Response>()
{
  return &xczs_inspection_robot_control::srv::typesupport_fastrtps_cpp::_SwitchToolset_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, SwitchToolset_Response)() {
  return &xczs_inspection_robot_control::srv::typesupport_fastrtps_cpp::_SwitchToolset_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace xczs_inspection_robot_control
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _SwitchToolset__callbacks = {
  "xczs_inspection_robot_control::srv",
  "SwitchToolset",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, SwitchToolset_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, SwitchToolset_Response)(),
};

static rosidl_service_type_support_t _SwitchToolset__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SwitchToolset__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_control

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_xczs_inspection_robot_control
const rosidl_service_type_support_t *
get_service_type_support_handle<xczs_inspection_robot_control::srv::SwitchToolset>()
{
  return &xczs_inspection_robot_control::srv::typesupport_fastrtps_cpp::_SwitchToolset__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, SwitchToolset)() {
  return &xczs_inspection_robot_control::srv::typesupport_fastrtps_cpp::_SwitchToolset__handle;
}

#ifdef __cplusplus
}
#endif
