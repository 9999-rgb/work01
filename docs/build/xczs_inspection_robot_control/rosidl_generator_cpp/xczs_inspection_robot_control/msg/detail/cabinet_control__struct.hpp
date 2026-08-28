// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_control:msg/CabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__msg__CabinetControl __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__msg__CabinetControl __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CabinetControl_
{
  using Type = CabinetControl_<ContainerAllocator>;

  explicit CabinetControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->display_name = "";
      this->control_type = 0;
      this->joint_name = "";
      this->joint_state_topic = "";
      this->pressed_topic = "";
      this->state_topic = "";
      this->supported_commands = 0;
      this->unit = "";
      this->min_position = 0.0;
      this->max_position = 0.0;
      this->requires_grasp = false;
      this->operable = false;
      this->unavailable_reason = "";
      this->required_toolset = "";
      this->toolset_compatible = false;
      this->adapter_validated = false;
      this->default_force = 0.0;
      this->min_trigger_force = 0.0;
      this->max_force = 0.0;
    }
  }

  explicit CabinetControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control_id(_alloc),
    display_name(_alloc),
    joint_name(_alloc),
    joint_state_topic(_alloc),
    pressed_topic(_alloc),
    state_topic(_alloc),
    unit(_alloc),
    unavailable_reason(_alloc),
    required_toolset(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->display_name = "";
      this->control_type = 0;
      this->joint_name = "";
      this->joint_state_topic = "";
      this->pressed_topic = "";
      this->state_topic = "";
      this->supported_commands = 0;
      this->unit = "";
      this->min_position = 0.0;
      this->max_position = 0.0;
      this->requires_grasp = false;
      this->operable = false;
      this->unavailable_reason = "";
      this->required_toolset = "";
      this->toolset_compatible = false;
      this->adapter_validated = false;
      this->default_force = 0.0;
      this->min_trigger_force = 0.0;
      this->max_force = 0.0;
    }
  }

  // field types and members
  using _control_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_id_type control_id;
  using _display_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _display_name_type display_name;
  using _control_type_type =
    uint8_t;
  _control_type_type control_type;
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _joint_state_topic_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_state_topic_type joint_state_topic;
  using _pressed_topic_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pressed_topic_type pressed_topic;
  using _state_topic_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_topic_type state_topic;
  using _supported_commands_type =
    uint8_t;
  _supported_commands_type supported_commands;
  using _unit_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _unit_type unit;
  using _min_position_type =
    double;
  _min_position_type min_position;
  using _max_position_type =
    double;
  _max_position_type max_position;
  using _state_ids_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _state_ids_type state_ids;
  using _state_labels_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _state_labels_type state_labels;
  using _state_positions_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _state_positions_type state_positions;
  using _requires_grasp_type =
    bool;
  _requires_grasp_type requires_grasp;
  using _operable_type =
    bool;
  _operable_type operable;
  using _unavailable_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _unavailable_reason_type unavailable_reason;
  using _required_toolset_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _required_toolset_type required_toolset;
  using _toolset_compatible_type =
    bool;
  _toolset_compatible_type toolset_compatible;
  using _adapter_validated_type =
    bool;
  _adapter_validated_type adapter_validated;
  using _default_force_type =
    double;
  _default_force_type default_force;
  using _min_trigger_force_type =
    double;
  _min_trigger_force_type min_trigger_force;
  using _max_force_type =
    double;
  _max_force_type max_force;

  // setters for named parameter idiom
  Type & set__control_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_id = _arg;
    return *this;
  }
  Type & set__display_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->display_name = _arg;
    return *this;
  }
  Type & set__control_type(
    const uint8_t & _arg)
  {
    this->control_type = _arg;
    return *this;
  }
  Type & set__joint_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_name = _arg;
    return *this;
  }
  Type & set__joint_state_topic(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_state_topic = _arg;
    return *this;
  }
  Type & set__pressed_topic(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pressed_topic = _arg;
    return *this;
  }
  Type & set__state_topic(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state_topic = _arg;
    return *this;
  }
  Type & set__supported_commands(
    const uint8_t & _arg)
  {
    this->supported_commands = _arg;
    return *this;
  }
  Type & set__unit(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->unit = _arg;
    return *this;
  }
  Type & set__min_position(
    const double & _arg)
  {
    this->min_position = _arg;
    return *this;
  }
  Type & set__max_position(
    const double & _arg)
  {
    this->max_position = _arg;
    return *this;
  }
  Type & set__state_ids(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->state_ids = _arg;
    return *this;
  }
  Type & set__state_labels(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->state_labels = _arg;
    return *this;
  }
  Type & set__state_positions(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->state_positions = _arg;
    return *this;
  }
  Type & set__requires_grasp(
    const bool & _arg)
  {
    this->requires_grasp = _arg;
    return *this;
  }
  Type & set__operable(
    const bool & _arg)
  {
    this->operable = _arg;
    return *this;
  }
  Type & set__unavailable_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->unavailable_reason = _arg;
    return *this;
  }
  Type & set__required_toolset(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->required_toolset = _arg;
    return *this;
  }
  Type & set__toolset_compatible(
    const bool & _arg)
  {
    this->toolset_compatible = _arg;
    return *this;
  }
  Type & set__adapter_validated(
    const bool & _arg)
  {
    this->adapter_validated = _arg;
    return *this;
  }
  Type & set__default_force(
    const double & _arg)
  {
    this->default_force = _arg;
    return *this;
  }
  Type & set__min_trigger_force(
    const double & _arg)
  {
    this->min_trigger_force = _arg;
    return *this;
  }
  Type & set__max_force(
    const double & _arg)
  {
    this->max_force = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TYPE_BUTTON =
    0u;
  static constexpr uint8_t TYPE_KNOB =
    1u;
  static constexpr uint8_t TYPE_SWITCH =
    2u;
  static constexpr uint8_t TYPE_DOOR =
    3u;
  static constexpr uint8_t SUPPORT_PRESS =
    1u;
  static constexpr uint8_t SUPPORT_SET_STATE =
    2u;
  static constexpr uint8_t SUPPORT_SET_POSITION =
    4u;
  static constexpr uint8_t SUPPORT_TOGGLE =
    8u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__msg__CabinetControl
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__msg__CabinetControl
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CabinetControl_ & other) const
  {
    if (this->control_id != other.control_id) {
      return false;
    }
    if (this->display_name != other.display_name) {
      return false;
    }
    if (this->control_type != other.control_type) {
      return false;
    }
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->joint_state_topic != other.joint_state_topic) {
      return false;
    }
    if (this->pressed_topic != other.pressed_topic) {
      return false;
    }
    if (this->state_topic != other.state_topic) {
      return false;
    }
    if (this->supported_commands != other.supported_commands) {
      return false;
    }
    if (this->unit != other.unit) {
      return false;
    }
    if (this->min_position != other.min_position) {
      return false;
    }
    if (this->max_position != other.max_position) {
      return false;
    }
    if (this->state_ids != other.state_ids) {
      return false;
    }
    if (this->state_labels != other.state_labels) {
      return false;
    }
    if (this->state_positions != other.state_positions) {
      return false;
    }
    if (this->requires_grasp != other.requires_grasp) {
      return false;
    }
    if (this->operable != other.operable) {
      return false;
    }
    if (this->unavailable_reason != other.unavailable_reason) {
      return false;
    }
    if (this->required_toolset != other.required_toolset) {
      return false;
    }
    if (this->toolset_compatible != other.toolset_compatible) {
      return false;
    }
    if (this->adapter_validated != other.adapter_validated) {
      return false;
    }
    if (this->default_force != other.default_force) {
      return false;
    }
    if (this->min_trigger_force != other.min_trigger_force) {
      return false;
    }
    if (this->max_force != other.max_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const CabinetControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CabinetControl_

// alias to use template instance with default allocator
using CabinetControl =
  xczs_inspection_robot_control::msg::CabinetControl_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::TYPE_BUTTON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::TYPE_KNOB;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::TYPE_SWITCH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::TYPE_DOOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::SUPPORT_PRESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::SUPPORT_SET_STATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::SUPPORT_SET_POSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CabinetControl_<ContainerAllocator>::SUPPORT_TOGGLE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL__STRUCT_HPP_
