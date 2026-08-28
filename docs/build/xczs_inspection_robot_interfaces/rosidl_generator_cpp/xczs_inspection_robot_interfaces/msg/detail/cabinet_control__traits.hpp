// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CabinetControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: control_id
  {
    out << "control_id: ";
    rosidl_generator_traits::value_to_yaml(msg.control_id, out);
    out << ", ";
  }

  // member: display_name
  {
    out << "display_name: ";
    rosidl_generator_traits::value_to_yaml(msg.display_name, out);
    out << ", ";
  }

  // member: control_type
  {
    out << "control_type: ";
    rosidl_generator_traits::value_to_yaml(msg.control_type, out);
    out << ", ";
  }

  // member: joint_name
  {
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << ", ";
  }

  // member: joint_state_topic
  {
    out << "joint_state_topic: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_state_topic, out);
    out << ", ";
  }

  // member: pressed_topic
  {
    out << "pressed_topic: ";
    rosidl_generator_traits::value_to_yaml(msg.pressed_topic, out);
    out << ", ";
  }

  // member: state_topic
  {
    out << "state_topic: ";
    rosidl_generator_traits::value_to_yaml(msg.state_topic, out);
    out << ", ";
  }

  // member: supported_commands
  {
    out << "supported_commands: ";
    rosidl_generator_traits::value_to_yaml(msg.supported_commands, out);
    out << ", ";
  }

  // member: unit
  {
    out << "unit: ";
    rosidl_generator_traits::value_to_yaml(msg.unit, out);
    out << ", ";
  }

  // member: min_position
  {
    out << "min_position: ";
    rosidl_generator_traits::value_to_yaml(msg.min_position, out);
    out << ", ";
  }

  // member: max_position
  {
    out << "max_position: ";
    rosidl_generator_traits::value_to_yaml(msg.max_position, out);
    out << ", ";
  }

  // member: state_ids
  {
    if (msg.state_ids.size() == 0) {
      out << "state_ids: []";
    } else {
      out << "state_ids: [";
      size_t pending_items = msg.state_ids.size();
      for (auto item : msg.state_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: state_labels
  {
    if (msg.state_labels.size() == 0) {
      out << "state_labels: []";
    } else {
      out << "state_labels: [";
      size_t pending_items = msg.state_labels.size();
      for (auto item : msg.state_labels) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: state_positions
  {
    if (msg.state_positions.size() == 0) {
      out << "state_positions: []";
    } else {
      out << "state_positions: [";
      size_t pending_items = msg.state_positions.size();
      for (auto item : msg.state_positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: requires_grasp
  {
    out << "requires_grasp: ";
    rosidl_generator_traits::value_to_yaml(msg.requires_grasp, out);
    out << ", ";
  }

  // member: operable
  {
    out << "operable: ";
    rosidl_generator_traits::value_to_yaml(msg.operable, out);
    out << ", ";
  }

  // member: unavailable_reason
  {
    out << "unavailable_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.unavailable_reason, out);
    out << ", ";
  }

  // member: required_toolset
  {
    out << "required_toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.required_toolset, out);
    out << ", ";
  }

  // member: toolset_compatible
  {
    out << "toolset_compatible: ";
    rosidl_generator_traits::value_to_yaml(msg.toolset_compatible, out);
    out << ", ";
  }

  // member: adapter_validated
  {
    out << "adapter_validated: ";
    rosidl_generator_traits::value_to_yaml(msg.adapter_validated, out);
    out << ", ";
  }

  // member: default_force
  {
    out << "default_force: ";
    rosidl_generator_traits::value_to_yaml(msg.default_force, out);
    out << ", ";
  }

  // member: min_trigger_force
  {
    out << "min_trigger_force: ";
    rosidl_generator_traits::value_to_yaml(msg.min_trigger_force, out);
    out << ", ";
  }

  // member: max_force
  {
    out << "max_force: ";
    rosidl_generator_traits::value_to_yaml(msg.max_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CabinetControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: control_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_id: ";
    rosidl_generator_traits::value_to_yaml(msg.control_id, out);
    out << "\n";
  }

  // member: display_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "display_name: ";
    rosidl_generator_traits::value_to_yaml(msg.display_name, out);
    out << "\n";
  }

  // member: control_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_type: ";
    rosidl_generator_traits::value_to_yaml(msg.control_type, out);
    out << "\n";
  }

  // member: joint_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << "\n";
  }

  // member: joint_state_topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_state_topic: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_state_topic, out);
    out << "\n";
  }

  // member: pressed_topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pressed_topic: ";
    rosidl_generator_traits::value_to_yaml(msg.pressed_topic, out);
    out << "\n";
  }

  // member: state_topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_topic: ";
    rosidl_generator_traits::value_to_yaml(msg.state_topic, out);
    out << "\n";
  }

  // member: supported_commands
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "supported_commands: ";
    rosidl_generator_traits::value_to_yaml(msg.supported_commands, out);
    out << "\n";
  }

  // member: unit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unit: ";
    rosidl_generator_traits::value_to_yaml(msg.unit, out);
    out << "\n";
  }

  // member: min_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_position: ";
    rosidl_generator_traits::value_to_yaml(msg.min_position, out);
    out << "\n";
  }

  // member: max_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_position: ";
    rosidl_generator_traits::value_to_yaml(msg.max_position, out);
    out << "\n";
  }

  // member: state_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.state_ids.size() == 0) {
      out << "state_ids: []\n";
    } else {
      out << "state_ids:\n";
      for (auto item : msg.state_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: state_labels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.state_labels.size() == 0) {
      out << "state_labels: []\n";
    } else {
      out << "state_labels:\n";
      for (auto item : msg.state_labels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: state_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.state_positions.size() == 0) {
      out << "state_positions: []\n";
    } else {
      out << "state_positions:\n";
      for (auto item : msg.state_positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: requires_grasp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requires_grasp: ";
    rosidl_generator_traits::value_to_yaml(msg.requires_grasp, out);
    out << "\n";
  }

  // member: operable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operable: ";
    rosidl_generator_traits::value_to_yaml(msg.operable, out);
    out << "\n";
  }

  // member: unavailable_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unavailable_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.unavailable_reason, out);
    out << "\n";
  }

  // member: required_toolset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "required_toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.required_toolset, out);
    out << "\n";
  }

  // member: toolset_compatible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "toolset_compatible: ";
    rosidl_generator_traits::value_to_yaml(msg.toolset_compatible, out);
    out << "\n";
  }

  // member: adapter_validated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adapter_validated: ";
    rosidl_generator_traits::value_to_yaml(msg.adapter_validated, out);
    out << "\n";
  }

  // member: default_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_force: ";
    rosidl_generator_traits::value_to_yaml(msg.default_force, out);
    out << "\n";
  }

  // member: min_trigger_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_trigger_force: ";
    rosidl_generator_traits::value_to_yaml(msg.min_trigger_force, out);
    out << "\n";
  }

  // member: max_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_force: ";
    rosidl_generator_traits::value_to_yaml(msg.max_force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CabinetControl & msg, bool use_flow_style = false)
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
  const xczs_inspection_robot_interfaces::msg::CabinetControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::msg::CabinetControl & msg)
{
  return xczs_inspection_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::msg::CabinetControl>()
{
  return "xczs_inspection_robot_interfaces::msg::CabinetControl";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::msg::CabinetControl>()
{
  return "xczs_inspection_robot_interfaces/msg/CabinetControl";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::msg::CabinetControl>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::msg::CabinetControl>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::msg::CabinetControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__TRAITS_HPP_
