// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_control:srv/SetCabinetGrasp.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'robot_grasp_point'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Request __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Request __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetCabinetGrasp_Request_
{
  using Type = SetCabinetGrasp_Request_<ContainerAllocator>;

  explicit SetCabinetGrasp_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_grasp_point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->operation_lease_id = "";
      this->robot_model = "";
      this->robot_link = "";
      this->robot_base_link = "";
      this->attach = false;
    }
  }

  explicit SetCabinetGrasp_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control_id(_alloc),
    operation_lease_id(_alloc),
    robot_model(_alloc),
    robot_link(_alloc),
    robot_grasp_point(_alloc, _init),
    robot_base_link(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->operation_lease_id = "";
      this->robot_model = "";
      this->robot_link = "";
      this->robot_base_link = "";
      this->attach = false;
    }
  }

  // field types and members
  using _control_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_id_type control_id;
  using _operation_lease_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operation_lease_id_type operation_lease_id;
  using _robot_model_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_model_type robot_model;
  using _robot_link_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_link_type robot_link;
  using _robot_grasp_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _robot_grasp_point_type robot_grasp_point;
  using _robot_base_link_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_base_link_type robot_base_link;
  using _attach_type =
    bool;
  _attach_type attach;

  // setters for named parameter idiom
  Type & set__control_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_id = _arg;
    return *this;
  }
  Type & set__operation_lease_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operation_lease_id = _arg;
    return *this;
  }
  Type & set__robot_model(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_model = _arg;
    return *this;
  }
  Type & set__robot_link(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_link = _arg;
    return *this;
  }
  Type & set__robot_grasp_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->robot_grasp_point = _arg;
    return *this;
  }
  Type & set__robot_base_link(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_base_link = _arg;
    return *this;
  }
  Type & set__attach(
    const bool & _arg)
  {
    this->attach = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Request
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Request
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetCabinetGrasp_Request_ & other) const
  {
    if (this->control_id != other.control_id) {
      return false;
    }
    if (this->operation_lease_id != other.operation_lease_id) {
      return false;
    }
    if (this->robot_model != other.robot_model) {
      return false;
    }
    if (this->robot_link != other.robot_link) {
      return false;
    }
    if (this->robot_grasp_point != other.robot_grasp_point) {
      return false;
    }
    if (this->robot_base_link != other.robot_base_link) {
      return false;
    }
    if (this->attach != other.attach) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetCabinetGrasp_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetCabinetGrasp_Request_

// alias to use template instance with default allocator
using SetCabinetGrasp_Request =
  xczs_inspection_robot_control::srv::SetCabinetGrasp_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace xczs_inspection_robot_control


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Response __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Response __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetCabinetGrasp_Response_
{
  using Type = SetCabinetGrasp_Response_<ContainerAllocator>;

  explicit SetCabinetGrasp_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->distance = 0.0;
    }
  }

  explicit SetCabinetGrasp_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->distance = 0.0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _distance_type =
    double;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Response
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__srv__SetCabinetGrasp_Response
    std::shared_ptr<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetCabinetGrasp_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetCabinetGrasp_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetCabinetGrasp_Response_

// alias to use template instance with default allocator
using SetCabinetGrasp_Response =
  xczs_inspection_robot_control::srv::SetCabinetGrasp_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace xczs_inspection_robot_control

namespace xczs_inspection_robot_control
{

namespace srv
{

struct SetCabinetGrasp
{
  using Request = xczs_inspection_robot_control::srv::SetCabinetGrasp_Request;
  using Response = xczs_inspection_robot_control::srv::SetCabinetGrasp_Response;
};

}  // namespace srv

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__STRUCT_HPP_
