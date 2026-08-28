// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_interfaces:srv/SwitchToolset.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Request __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Request __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SwitchToolset_Request_
{
  using Type = SwitchToolset_Request_<ContainerAllocator>;

  explicit SwitchToolset_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->toolset = "";
      this->expected_generation = 0ull;
    }
  }

  explicit SwitchToolset_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : toolset(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->toolset = "";
      this->expected_generation = 0ull;
    }
  }

  // field types and members
  using _toolset_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _toolset_type toolset;
  using _expected_generation_type =
    uint64_t;
  _expected_generation_type expected_generation;

  // setters for named parameter idiom
  Type & set__toolset(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->toolset = _arg;
    return *this;
  }
  Type & set__expected_generation(
    const uint64_t & _arg)
  {
    this->expected_generation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Request
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Request
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SwitchToolset_Request_ & other) const
  {
    if (this->toolset != other.toolset) {
      return false;
    }
    if (this->expected_generation != other.expected_generation) {
      return false;
    }
    return true;
  }
  bool operator!=(const SwitchToolset_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SwitchToolset_Request_

// alias to use template instance with default allocator
using SwitchToolset_Request =
  xczs_inspection_robot_interfaces::srv::SwitchToolset_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Response __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Response __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SwitchToolset_Response_
{
  using Type = SwitchToolset_Response_<ContainerAllocator>;

  explicit SwitchToolset_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->state = "";
      this->active_toolset = "";
      this->target_toolset = "";
      this->generation = 0ull;
      this->message = "";
    }
  }

  explicit SwitchToolset_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc),
    active_toolset(_alloc),
    target_toolset(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->state = "";
      this->active_toolset = "";
      this->target_toolset = "";
      this->generation = 0ull;
      this->message = "";
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _active_toolset_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _active_toolset_type active_toolset;
  using _target_toolset_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_toolset_type target_toolset;
  using _generation_type =
    uint64_t;
  _generation_type generation;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__active_toolset(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->active_toolset = _arg;
    return *this;
  }
  Type & set__target_toolset(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_toolset = _arg;
    return *this;
  }
  Type & set__generation(
    const uint64_t & _arg)
  {
    this->generation = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Response
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__srv__SwitchToolset_Response
    std::shared_ptr<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SwitchToolset_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->active_toolset != other.active_toolset) {
      return false;
    }
    if (this->target_toolset != other.target_toolset) {
      return false;
    }
    if (this->generation != other.generation) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SwitchToolset_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SwitchToolset_Response_

// alias to use template instance with default allocator
using SwitchToolset_Response =
  xczs_inspection_robot_interfaces::srv::SwitchToolset_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

struct SwitchToolset
{
  using Request = xczs_inspection_robot_interfaces::srv::SwitchToolset_Request;
  using Response = xczs_inspection_robot_interfaces::srv::SwitchToolset_Response;
};

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__STRUCT_HPP_
