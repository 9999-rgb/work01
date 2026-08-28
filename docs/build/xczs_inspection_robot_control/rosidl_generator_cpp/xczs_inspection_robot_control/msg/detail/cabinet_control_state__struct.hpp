// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_control:msg/CabinetControlState.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_STATE__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlState __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlState __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CabinetControlState_
{
  using Type = CabinetControlState_<ContainerAllocator>;

  explicit CabinetControlState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->control_type = 0;
      this->valid = false;
      this->position = 0.0;
      this->velocity = 0.0;
      this->effort = 0.0;
      this->normalized_position = 0.0;
      this->state_id = "";
      this->activated = false;
      this->in_motion = false;
      this->transition_sequence = 0ull;
    }
  }

  explicit CabinetControlState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    control_id(_alloc),
    state_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->control_type = 0;
      this->valid = false;
      this->position = 0.0;
      this->velocity = 0.0;
      this->effort = 0.0;
      this->normalized_position = 0.0;
      this->state_id = "";
      this->activated = false;
      this->in_motion = false;
      this->transition_sequence = 0ull;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _control_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_id_type control_id;
  using _control_type_type =
    uint8_t;
  _control_type_type control_type;
  using _valid_type =
    bool;
  _valid_type valid;
  using _position_type =
    double;
  _position_type position;
  using _velocity_type =
    double;
  _velocity_type velocity;
  using _effort_type =
    double;
  _effort_type effort;
  using _normalized_position_type =
    double;
  _normalized_position_type normalized_position;
  using _state_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_id_type state_id;
  using _activated_type =
    bool;
  _activated_type activated;
  using _in_motion_type =
    bool;
  _in_motion_type in_motion;
  using _transition_sequence_type =
    uint64_t;
  _transition_sequence_type transition_sequence;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__control_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_id = _arg;
    return *this;
  }
  Type & set__control_type(
    const uint8_t & _arg)
  {
    this->control_type = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__position(
    const double & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const double & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const double & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__normalized_position(
    const double & _arg)
  {
    this->normalized_position = _arg;
    return *this;
  }
  Type & set__state_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state_id = _arg;
    return *this;
  }
  Type & set__activated(
    const bool & _arg)
  {
    this->activated = _arg;
    return *this;
  }
  Type & set__in_motion(
    const bool & _arg)
  {
    this->in_motion = _arg;
    return *this;
  }
  Type & set__transition_sequence(
    const uint64_t & _arg)
  {
    this->transition_sequence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlState
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlState
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CabinetControlState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->control_id != other.control_id) {
      return false;
    }
    if (this->control_type != other.control_type) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->normalized_position != other.normalized_position) {
      return false;
    }
    if (this->state_id != other.state_id) {
      return false;
    }
    if (this->activated != other.activated) {
      return false;
    }
    if (this->in_motion != other.in_motion) {
      return false;
    }
    if (this->transition_sequence != other.transition_sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const CabinetControlState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CabinetControlState_

// alias to use template instance with default allocator
using CabinetControlState =
  xczs_inspection_robot_control::msg::CabinetControlState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_STATE__STRUCT_HPP_
