// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xczs_inspection_robot_interfaces:srv/SwitchToolset.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__struct.h"
#include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // toolset
#include "rosidl_runtime_c/string_functions.h"  // toolset

// forward declare type support functions


using _SwitchToolset_Request__ros_msg_type = xczs_inspection_robot_interfaces__srv__SwitchToolset_Request;

static bool _SwitchToolset_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SwitchToolset_Request__ros_msg_type * ros_message = static_cast<const _SwitchToolset_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: toolset
  {
    const rosidl_runtime_c__String * str = &ros_message->toolset;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: expected_generation
  {
    cdr << ros_message->expected_generation;
  }

  return true;
}

static bool _SwitchToolset_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SwitchToolset_Request__ros_msg_type * ros_message = static_cast<_SwitchToolset_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: toolset
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->toolset.data) {
      rosidl_runtime_c__String__init(&ros_message->toolset);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->toolset,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'toolset'\n");
      return false;
    }
  }

  // Field name: expected_generation
  {
    cdr >> ros_message->expected_generation;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_interfaces
size_t get_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SwitchToolset_Request__ros_msg_type * ros_message = static_cast<const _SwitchToolset_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->toolset.size + 1);
  // field.name expected_generation
  {
    size_t item_size = sizeof(ros_message->expected_generation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SwitchToolset_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_interfaces
size_t max_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Request(
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

  // member: toolset
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
  // member: expected_generation
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
    using DataType = xczs_inspection_robot_interfaces__srv__SwitchToolset_Request;
    is_plain =
      (
      offsetof(DataType, expected_generation) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SwitchToolset_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SwitchToolset_Request = {
  "xczs_inspection_robot_interfaces::srv",
  "SwitchToolset_Request",
  _SwitchToolset_Request__cdr_serialize,
  _SwitchToolset_Request__cdr_deserialize,
  _SwitchToolset_Request__get_serialized_size,
  _SwitchToolset_Request__max_serialized_size
};

static rosidl_message_type_support_t _SwitchToolset_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SwitchToolset_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xczs_inspection_robot_interfaces, srv, SwitchToolset_Request)() {
  return &_SwitchToolset_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // active_toolset, message, state, target_toolset
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // active_toolset, message, state, target_toolset

// forward declare type support functions


using _SwitchToolset_Response__ros_msg_type = xczs_inspection_robot_interfaces__srv__SwitchToolset_Response;

static bool _SwitchToolset_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SwitchToolset_Response__ros_msg_type * ros_message = static_cast<const _SwitchToolset_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  // Field name: state
  {
    const rosidl_runtime_c__String * str = &ros_message->state;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: active_toolset
  {
    const rosidl_runtime_c__String * str = &ros_message->active_toolset;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: target_toolset
  {
    const rosidl_runtime_c__String * str = &ros_message->target_toolset;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: generation
  {
    cdr << ros_message->generation;
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _SwitchToolset_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SwitchToolset_Response__ros_msg_type * ros_message = static_cast<_SwitchToolset_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->accepted = tmp ? true : false;
  }

  // Field name: state
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->state.data) {
      rosidl_runtime_c__String__init(&ros_message->state);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->state,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'state'\n");
      return false;
    }
  }

  // Field name: active_toolset
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->active_toolset.data) {
      rosidl_runtime_c__String__init(&ros_message->active_toolset);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->active_toolset,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'active_toolset'\n");
      return false;
    }
  }

  // Field name: target_toolset
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->target_toolset.data) {
      rosidl_runtime_c__String__init(&ros_message->target_toolset);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->target_toolset,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'target_toolset'\n");
      return false;
    }
  }

  // Field name: generation
  {
    cdr >> ros_message->generation;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_interfaces
size_t get_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SwitchToolset_Response__ros_msg_type * ros_message = static_cast<const _SwitchToolset_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name state
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->state.size + 1);
  // field.name active_toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->active_toolset.size + 1);
  // field.name target_toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->target_toolset.size + 1);
  // field.name generation
  {
    size_t item_size = sizeof(ros_message->generation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _SwitchToolset_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_interfaces
size_t max_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Response(
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

  // member: accepted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: state
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
  // member: active_toolset
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
  // member: target_toolset
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
  // member: generation
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: message
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
    using DataType = xczs_inspection_robot_interfaces__srv__SwitchToolset_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SwitchToolset_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_xczs_inspection_robot_interfaces__srv__SwitchToolset_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SwitchToolset_Response = {
  "xczs_inspection_robot_interfaces::srv",
  "SwitchToolset_Response",
  _SwitchToolset_Response__cdr_serialize,
  _SwitchToolset_Response__cdr_deserialize,
  _SwitchToolset_Response__get_serialized_size,
  _SwitchToolset_Response__max_serialized_size
};

static rosidl_message_type_support_t _SwitchToolset_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SwitchToolset_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xczs_inspection_robot_interfaces, srv, SwitchToolset_Response)() {
  return &_SwitchToolset_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xczs_inspection_robot_interfaces/srv/switch_toolset.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SwitchToolset__callbacks = {
  "xczs_inspection_robot_interfaces::srv",
  "SwitchToolset",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xczs_inspection_robot_interfaces, srv, SwitchToolset_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xczs_inspection_robot_interfaces, srv, SwitchToolset_Response)(),
};

static rosidl_service_type_support_t SwitchToolset__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SwitchToolset__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xczs_inspection_robot_interfaces, srv, SwitchToolset)() {
  return &SwitchToolset__handle;
}

#if defined(__cplusplus)
}
#endif
