// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_control:msg/CabinetControlCatalog.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_CATALOG__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_CATALOG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'controls'
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlCatalog __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlCatalog __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CabinetControlCatalog_
{
  using Type = CabinetControlCatalog_<ContainerAllocator>;

  explicit CabinetControlCatalog_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit CabinetControlCatalog_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _controls_type =
    std::vector<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>>>;
  _controls_type controls;

  // setters for named parameter idiom
  Type & set__controls(
    const std::vector<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<xczs_inspection_robot_control::msg::CabinetControl_<ContainerAllocator>>> & _arg)
  {
    this->controls = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlCatalog
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__msg__CabinetControlCatalog
    std::shared_ptr<xczs_inspection_robot_control::msg::CabinetControlCatalog_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CabinetControlCatalog_ & other) const
  {
    if (this->controls != other.controls) {
      return false;
    }
    return true;
  }
  bool operator!=(const CabinetControlCatalog_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CabinetControlCatalog_

// alias to use template instance with default allocator
using CabinetControlCatalog =
  xczs_inspection_robot_control::msg::CabinetControlCatalog_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_CATALOG__STRUCT_HPP_
