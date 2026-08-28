// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControl.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__struct.h"
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // state_positions
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // state_positions
#include "rosidl_runtime_c/string.h"  // control_id, display_name, joint_name, joint_state_topic, pressed_topic, required_toolset, state_ids, state_labels, state_topic, unavailable_reason, unit
#include "rosidl_runtime_c/string_functions.h"  // control_id, display_name, joint_name, joint_state_topic, pressed_topic, required_toolset, state_ids, state_labels, state_topic, unavailable_reason, unit

// forward declare type support functions


using _CabinetControl__ros_msg_type = xczs_inspection_robot_interfaces__msg__CabinetControl;

static bool _CabinetControl__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CabinetControl__ros_msg_type * ros_message = static_cast<const _CabinetControl__ros_msg_type *>(untyped_ros_message);
  // Field name: control_id
  {
    const rosidl_runtime_c__String * str = &ros_message->control_id;
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

  // Field name: display_name
  {
    const rosidl_runtime_c__String * str = &ros_message->display_name;
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

  // Field name: control_type
  {
    cdr << ros_message->control_type;
  }

  // Field name: joint_name
  {
    const rosidl_runtime_c__String * str = &ros_message->joint_name;
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

  // Field name: joint_state_topic
  {
    const rosidl_runtime_c__String * str = &ros_message->joint_state_topic;
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

  // Field name: pressed_topic
  {
    const rosidl_runtime_c__String * str = &ros_message->pressed_topic;
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

  // Field name: state_topic
  {
    const rosidl_runtime_c__String * str = &ros_message->state_topic;
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

  // Field name: supported_commands
  {
    cdr << ros_message->supported_commands;
  }

  // Field name: unit
  {
    const rosidl_runtime_c__String * str = &ros_message->unit;
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

  // Field name: min_position
  {
    cdr << ros_message->min_position;
  }

  // Field name: max_position
  {
    cdr << ros_message->max_position;
  }

  // Field name: state_ids
  {
    size_t size = ros_message->state_ids.size;
    auto array_ptr = ros_message->state_ids.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
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
  }

  // Field name: state_labels
  {
    size_t size = ros_message->state_labels.size;
    auto array_ptr = ros_message->state_labels.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
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
  }

  // Field name: state_positions
  {
    size_t size = ros_message->state_positions.size;
    auto array_ptr = ros_message->state_positions.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: requires_grasp
  {
    cdr << (ros_message->requires_grasp ? true : false);
  }

  // Field name: operable
  {
    cdr << (ros_message->operable ? true : false);
  }

  // Field name: unavailable_reason
  {
    const rosidl_runtime_c__String * str = &ros_message->unavailable_reason;
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

  // Field name: required_toolset
  {
    const rosidl_runtime_c__String * str = &ros_message->required_toolset;
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

  // Field name: toolset_compatible
  {
    cdr << (ros_message->toolset_compatible ? true : false);
  }

  // Field name: adapter_validated
  {
    cdr << (ros_message->adapter_validated ? true : false);
  }

  // Field name: default_force
  {
    cdr << ros_message->default_force;
  }

  // Field name: min_trigger_force
  {
    cdr << ros_message->min_trigger_force;
  }

  // Field name: max_force
  {
    cdr << ros_message->max_force;
  }

  return true;
}

static bool _CabinetControl__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CabinetControl__ros_msg_type * ros_message = static_cast<_CabinetControl__ros_msg_type *>(untyped_ros_message);
  // Field name: control_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->control_id.data) {
      rosidl_runtime_c__String__init(&ros_message->control_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->control_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'control_id'\n");
      return false;
    }
  }

  // Field name: display_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->display_name.data) {
      rosidl_runtime_c__String__init(&ros_message->display_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->display_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'display_name'\n");
      return false;
    }
  }

  // Field name: control_type
  {
    cdr >> ros_message->control_type;
  }

  // Field name: joint_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->joint_name.data) {
      rosidl_runtime_c__String__init(&ros_message->joint_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->joint_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'joint_name'\n");
      return false;
    }
  }

  // Field name: joint_state_topic
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->joint_state_topic.data) {
      rosidl_runtime_c__String__init(&ros_message->joint_state_topic);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->joint_state_topic,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'joint_state_topic'\n");
      return false;
    }
  }

  // Field name: pressed_topic
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->pressed_topic.data) {
      rosidl_runtime_c__String__init(&ros_message->pressed_topic);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->pressed_topic,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'pressed_topic'\n");
      return false;
    }
  }

  // Field name: state_topic
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->state_topic.data) {
      rosidl_runtime_c__String__init(&ros_message->state_topic);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->state_topic,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'state_topic'\n");
      return false;
    }
  }

  // Field name: supported_commands
  {
    cdr >> ros_message->supported_commands;
  }

  // Field name: unit
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->unit.data) {
      rosidl_runtime_c__String__init(&ros_message->unit);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->unit,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'unit'\n");
      return false;
    }
  }

  // Field name: min_position
  {
    cdr >> ros_message->min_position;
  }

  // Field name: max_position
  {
    cdr >> ros_message->max_position;
  }

  // Field name: state_ids
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->state_ids.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->state_ids);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->state_ids, size)) {
      fprintf(stderr, "failed to create array for field 'state_ids'");
      return false;
    }
    auto array_ptr = ros_message->state_ids.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'state_ids'\n");
        return false;
      }
    }
  }

  // Field name: state_labels
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->state_labels.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->state_labels);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->state_labels, size)) {
      fprintf(stderr, "failed to create array for field 'state_labels'");
      return false;
    }
    auto array_ptr = ros_message->state_labels.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'state_labels'\n");
        return false;
      }
    }
  }

  // Field name: state_positions
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->state_positions.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->state_positions);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->state_positions, size)) {
      fprintf(stderr, "failed to create array for field 'state_positions'");
      return false;
    }
    auto array_ptr = ros_message->state_positions.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: requires_grasp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->requires_grasp = tmp ? true : false;
  }

  // Field name: operable
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->operable = tmp ? true : false;
  }

  // Field name: unavailable_reason
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->unavailable_reason.data) {
      rosidl_runtime_c__String__init(&ros_message->unavailable_reason);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->unavailable_reason,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'unavailable_reason'\n");
      return false;
    }
  }

  // Field name: required_toolset
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->required_toolset.data) {
      rosidl_runtime_c__String__init(&ros_message->required_toolset);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->required_toolset,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'required_toolset'\n");
      return false;
    }
  }

  // Field name: toolset_compatible
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->toolset_compatible = tmp ? true : false;
  }

  // Field name: adapter_validated
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->adapter_validated = tmp ? true : false;
  }

  // Field name: default_force
  {
    cdr >> ros_message->default_force;
  }

  // Field name: min_trigger_force
  {
    cdr >> ros_message->min_trigger_force;
  }

  // Field name: max_force
  {
    cdr >> ros_message->max_force;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_interfaces
size_t get_serialized_size_xczs_inspection_robot_interfaces__msg__CabinetControl(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CabinetControl__ros_msg_type * ros_message = static_cast<const _CabinetControl__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name control_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->control_id.size + 1);
  // field.name display_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->display_name.size + 1);
  // field.name control_type
  {
    size_t item_size = sizeof(ros_message->control_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->joint_name.size + 1);
  // field.name joint_state_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->joint_state_topic.size + 1);
  // field.name pressed_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->pressed_topic.size + 1);
  // field.name state_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->state_topic.size + 1);
  // field.name supported_commands
  {
    size_t item_size = sizeof(ros_message->supported_commands);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name unit
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->unit.size + 1);
  // field.name min_position
  {
    size_t item_size = sizeof(ros_message->min_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_position
  {
    size_t item_size = sizeof(ros_message->max_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name state_ids
  {
    size_t array_size = ros_message->state_ids.size;
    auto array_ptr = ros_message->state_ids.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name state_labels
  {
    size_t array_size = ros_message->state_labels.size;
    auto array_ptr = ros_message->state_labels.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name state_positions
  {
    size_t array_size = ros_message->state_positions.size;
    auto array_ptr = ros_message->state_positions.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name requires_grasp
  {
    size_t item_size = sizeof(ros_message->requires_grasp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name operable
  {
    size_t item_size = sizeof(ros_message->operable);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name unavailable_reason
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->unavailable_reason.size + 1);
  // field.name required_toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->required_toolset.size + 1);
  // field.name toolset_compatible
  {
    size_t item_size = sizeof(ros_message->toolset_compatible);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name adapter_validated
  {
    size_t item_size = sizeof(ros_message->adapter_validated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name default_force
  {
    size_t item_size = sizeof(ros_message->default_force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name min_trigger_force
  {
    size_t item_size = sizeof(ros_message->min_trigger_force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_force
  {
    size_t item_size = sizeof(ros_message->max_force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CabinetControl__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xczs_inspection_robot_interfaces__msg__CabinetControl(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xczs_inspection_robot_interfaces
size_t max_serialized_size_xczs_inspection_robot_interfaces__msg__CabinetControl(
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

  // member: control_id
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
  // member: display_name
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
  // member: control_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: joint_name
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
  // member: joint_state_topic
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
  // member: pressed_topic
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
  // member: state_topic
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
  // member: supported_commands
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: unit
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
  // member: min_position
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: max_position
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: state_ids
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: state_labels
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: state_positions
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: requires_grasp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: operable
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: unavailable_reason
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
  // member: required_toolset
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
  // member: toolset_compatible
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: adapter_validated
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: default_force
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: min_trigger_force
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: max_force
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
    using DataType = xczs_inspection_robot_interfaces__msg__CabinetControl;
    is_plain =
      (
      offsetof(DataType, max_force) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CabinetControl__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_xczs_inspection_robot_interfaces__msg__CabinetControl(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CabinetControl = {
  "xczs_inspection_robot_interfaces::msg",
  "CabinetControl",
  _CabinetControl__cdr_serialize,
  _CabinetControl__cdr_deserialize,
  _CabinetControl__get_serialized_size,
  _CabinetControl__max_serialized_size
};

static rosidl_message_type_support_t _CabinetControl__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CabinetControl,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xczs_inspection_robot_interfaces, msg, CabinetControl)() {
  return &_CabinetControl__type_support;
}

#if defined(__cplusplus)
}
#endif
