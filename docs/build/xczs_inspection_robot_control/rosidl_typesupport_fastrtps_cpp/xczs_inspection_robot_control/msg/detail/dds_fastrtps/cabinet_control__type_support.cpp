// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from xczs_inspection_robot_control:msg/CabinetControl.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__rosidl_typesupport_fastrtps_cpp.hpp"
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__struct.hpp"

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

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_serialize(
  const xczs_inspection_robot_control::msg::CabinetControl & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: control_id
  cdr << ros_message.control_id;
  // Member: display_name
  cdr << ros_message.display_name;
  // Member: control_type
  cdr << ros_message.control_type;
  // Member: joint_name
  cdr << ros_message.joint_name;
  // Member: joint_state_topic
  cdr << ros_message.joint_state_topic;
  // Member: pressed_topic
  cdr << ros_message.pressed_topic;
  // Member: state_topic
  cdr << ros_message.state_topic;
  // Member: supported_commands
  cdr << ros_message.supported_commands;
  // Member: unit
  cdr << ros_message.unit;
  // Member: min_position
  cdr << ros_message.min_position;
  // Member: max_position
  cdr << ros_message.max_position;
  // Member: state_ids
  {
    cdr << ros_message.state_ids;
  }
  // Member: state_labels
  {
    cdr << ros_message.state_labels;
  }
  // Member: state_positions
  {
    cdr << ros_message.state_positions;
  }
  // Member: requires_grasp
  cdr << (ros_message.requires_grasp ? true : false);
  // Member: operable
  cdr << (ros_message.operable ? true : false);
  // Member: unavailable_reason
  cdr << ros_message.unavailable_reason;
  // Member: required_toolset
  cdr << ros_message.required_toolset;
  // Member: toolset_compatible
  cdr << (ros_message.toolset_compatible ? true : false);
  // Member: adapter_validated
  cdr << (ros_message.adapter_validated ? true : false);
  // Member: default_force
  cdr << ros_message.default_force;
  // Member: min_trigger_force
  cdr << ros_message.min_trigger_force;
  // Member: max_force
  cdr << ros_message.max_force;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  xczs_inspection_robot_control::msg::CabinetControl & ros_message)
{
  // Member: control_id
  cdr >> ros_message.control_id;

  // Member: display_name
  cdr >> ros_message.display_name;

  // Member: control_type
  cdr >> ros_message.control_type;

  // Member: joint_name
  cdr >> ros_message.joint_name;

  // Member: joint_state_topic
  cdr >> ros_message.joint_state_topic;

  // Member: pressed_topic
  cdr >> ros_message.pressed_topic;

  // Member: state_topic
  cdr >> ros_message.state_topic;

  // Member: supported_commands
  cdr >> ros_message.supported_commands;

  // Member: unit
  cdr >> ros_message.unit;

  // Member: min_position
  cdr >> ros_message.min_position;

  // Member: max_position
  cdr >> ros_message.max_position;

  // Member: state_ids
  {
    cdr >> ros_message.state_ids;
  }

  // Member: state_labels
  {
    cdr >> ros_message.state_labels;
  }

  // Member: state_positions
  {
    cdr >> ros_message.state_positions;
  }

  // Member: requires_grasp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.requires_grasp = tmp ? true : false;
  }

  // Member: operable
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.operable = tmp ? true : false;
  }

  // Member: unavailable_reason
  cdr >> ros_message.unavailable_reason;

  // Member: required_toolset
  cdr >> ros_message.required_toolset;

  // Member: toolset_compatible
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.toolset_compatible = tmp ? true : false;
  }

  // Member: adapter_validated
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.adapter_validated = tmp ? true : false;
  }

  // Member: default_force
  cdr >> ros_message.default_force;

  // Member: min_trigger_force
  cdr >> ros_message.min_trigger_force;

  // Member: max_force
  cdr >> ros_message.max_force;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
get_serialized_size(
  const xczs_inspection_robot_control::msg::CabinetControl & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: control_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.control_id.size() + 1);
  // Member: display_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.display_name.size() + 1);
  // Member: control_type
  {
    size_t item_size = sizeof(ros_message.control_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.joint_name.size() + 1);
  // Member: joint_state_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.joint_state_topic.size() + 1);
  // Member: pressed_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.pressed_topic.size() + 1);
  // Member: state_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.state_topic.size() + 1);
  // Member: supported_commands
  {
    size_t item_size = sizeof(ros_message.supported_commands);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: unit
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.unit.size() + 1);
  // Member: min_position
  {
    size_t item_size = sizeof(ros_message.min_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_position
  {
    size_t item_size = sizeof(ros_message.max_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: state_ids
  {
    size_t array_size = ros_message.state_ids.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.state_ids[index].size() + 1);
    }
  }
  // Member: state_labels
  {
    size_t array_size = ros_message.state_labels.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.state_labels[index].size() + 1);
    }
  }
  // Member: state_positions
  {
    size_t array_size = ros_message.state_positions.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.state_positions[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: requires_grasp
  {
    size_t item_size = sizeof(ros_message.requires_grasp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: operable
  {
    size_t item_size = sizeof(ros_message.operable);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: unavailable_reason
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.unavailable_reason.size() + 1);
  // Member: required_toolset
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.required_toolset.size() + 1);
  // Member: toolset_compatible
  {
    size_t item_size = sizeof(ros_message.toolset_compatible);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: adapter_validated
  {
    size_t item_size = sizeof(ros_message.adapter_validated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: default_force
  {
    size_t item_size = sizeof(ros_message.default_force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: min_trigger_force
  {
    size_t item_size = sizeof(ros_message.min_trigger_force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_force
  {
    size_t item_size = sizeof(ros_message.max_force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
max_serialized_size_CabinetControl(
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


  // Member: control_id
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

  // Member: display_name
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

  // Member: control_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: joint_name
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

  // Member: joint_state_topic
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

  // Member: pressed_topic
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

  // Member: state_topic
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

  // Member: supported_commands
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: unit
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

  // Member: min_position
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: max_position
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: state_ids
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

  // Member: state_labels
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

  // Member: state_positions
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

  // Member: requires_grasp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: operable
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: unavailable_reason
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

  // Member: required_toolset
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

  // Member: toolset_compatible
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: adapter_validated
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: default_force
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: min_trigger_force
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: max_force
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
    using DataType = xczs_inspection_robot_control::msg::CabinetControl;
    is_plain =
      (
      offsetof(DataType, max_force) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _CabinetControl__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_control::msg::CabinetControl *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CabinetControl__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<xczs_inspection_robot_control::msg::CabinetControl *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CabinetControl__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const xczs_inspection_robot_control::msg::CabinetControl *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CabinetControl__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_CabinetControl(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _CabinetControl__callbacks = {
  "xczs_inspection_robot_control::msg",
  "CabinetControl",
  _CabinetControl__cdr_serialize,
  _CabinetControl__cdr_deserialize,
  _CabinetControl__get_serialized_size,
  _CabinetControl__max_serialized_size
};

static rosidl_message_type_support_t _CabinetControl__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CabinetControl__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace xczs_inspection_robot_control

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_xczs_inspection_robot_control
const rosidl_message_type_support_t *
get_message_type_support_handle<xczs_inspection_robot_control::msg::CabinetControl>()
{
  return &xczs_inspection_robot_control::msg::typesupport_fastrtps_cpp::_CabinetControl__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, msg, CabinetControl)() {
  return &xczs_inspection_robot_control::msg::typesupport_fastrtps_cpp::_CabinetControl__handle;
}

#ifdef __cplusplus
}
#endif
