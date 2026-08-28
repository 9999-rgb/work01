// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_control:srv/ManageOperationLease.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Request __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Request __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ManageOperationLease_Request_
{
  using Type = ManageOperationLease_Request_<ContainerAllocator>;

  explicit ManageOperationLease_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
      this->owner_id = "";
      this->lease_id = "";
      this->requested_duration = 0.0;
    }
  }

  explicit ManageOperationLease_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : owner_id(_alloc),
    lease_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
      this->owner_id = "";
      this->lease_id = "";
      this->requested_duration = 0.0;
    }
  }

  // field types and members
  using _command_type =
    uint8_t;
  _command_type command;
  using _owner_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _owner_id_type owner_id;
  using _lease_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _lease_id_type lease_id;
  using _requested_duration_type =
    double;
  _requested_duration_type requested_duration;

  // setters for named parameter idiom
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__owner_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->owner_id = _arg;
    return *this;
  }
  Type & set__lease_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->lease_id = _arg;
    return *this;
  }
  Type & set__requested_duration(
    const double & _arg)
  {
    this->requested_duration = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ACQUIRE =
    0u;
  static constexpr uint8_t RENEW =
    1u;
  static constexpr uint8_t RELEASE =
    2u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Request
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Request
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManageOperationLease_Request_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->owner_id != other.owner_id) {
      return false;
    }
    if (this->lease_id != other.lease_id) {
      return false;
    }
    if (this->requested_duration != other.requested_duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManageOperationLease_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManageOperationLease_Request_

// alias to use template instance with default allocator
using ManageOperationLease_Request =
  xczs_inspection_robot_control::srv::ManageOperationLease_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Request_<ContainerAllocator>::ACQUIRE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Request_<ContainerAllocator>::RENEW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Request_<ContainerAllocator>::RELEASE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace xczs_inspection_robot_control


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Response __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Response __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ManageOperationLease_Response_
{
  using Type = ManageOperationLease_Response_<ContainerAllocator>;

  explicit ManageOperationLease_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->status_code = 0;
      this->message = "";
      this->lease_id = "";
      this->owner_id = "";
      this->remaining_duration = 0.0;
    }
  }

  explicit ManageOperationLease_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    lease_id(_alloc),
    owner_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->status_code = 0;
      this->message = "";
      this->lease_id = "";
      this->owner_id = "";
      this->remaining_duration = 0.0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _status_code_type =
    uint8_t;
  _status_code_type status_code;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _lease_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _lease_id_type lease_id;
  using _owner_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _owner_id_type owner_id;
  using _remaining_duration_type =
    double;
  _remaining_duration_type remaining_duration;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__status_code(
    const uint8_t & _arg)
  {
    this->status_code = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__lease_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->lease_id = _arg;
    return *this;
  }
  Type & set__owner_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->owner_id = _arg;
    return *this;
  }
  Type & set__remaining_duration(
    const double & _arg)
  {
    this->remaining_duration = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GRANTED =
    0u;
  static constexpr uint8_t RESOURCE_BUSY =
    1u;
  static constexpr uint8_t INVALID_REQUEST =
    2u;
  static constexpr uint8_t NOT_OWNER =
    3u;
  static constexpr uint8_t RELEASED =
    4u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Response
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__srv__ManageOperationLease_Response
    std::shared_ptr<xczs_inspection_robot_control::srv::ManageOperationLease_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManageOperationLease_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->status_code != other.status_code) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->lease_id != other.lease_id) {
      return false;
    }
    if (this->owner_id != other.owner_id) {
      return false;
    }
    if (this->remaining_duration != other.remaining_duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManageOperationLease_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManageOperationLease_Response_

// alias to use template instance with default allocator
using ManageOperationLease_Response =
  xczs_inspection_robot_control::srv::ManageOperationLease_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Response_<ContainerAllocator>::GRANTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Response_<ContainerAllocator>::RESOURCE_BUSY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Response_<ContainerAllocator>::INVALID_REQUEST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Response_<ContainerAllocator>::NOT_OWNER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManageOperationLease_Response_<ContainerAllocator>::RELEASED;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace xczs_inspection_robot_control

namespace xczs_inspection_robot_control
{

namespace srv
{

struct ManageOperationLease
{
  using Request = xczs_inspection_robot_control::srv::ManageOperationLease_Request;
  using Response = xczs_inspection_robot_control::srv::ManageOperationLease_Response;
};

}  // namespace srv

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__STRUCT_HPP_
