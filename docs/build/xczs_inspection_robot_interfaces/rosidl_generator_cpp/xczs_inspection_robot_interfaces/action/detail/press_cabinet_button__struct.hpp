// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_interfaces:action/PressCabinetButton.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__PRESS_CABINET_BUTTON__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__PRESS_CABINET_BUTTON__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_Goal_
{
  using Type = PressCabinetButton_Goal_<ContainerAllocator>;

  explicit PressCabinetButton_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->button_id = "";
      this->navigate_to_staging_pose = false;
    }
  }

  explicit PressCabinetButton_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : button_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->button_id = "";
      this->navigate_to_staging_pose = false;
    }
  }

  // field types and members
  using _button_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _button_id_type button_id;
  using _navigate_to_staging_pose_type =
    bool;
  _navigate_to_staging_pose_type navigate_to_staging_pose;

  // setters for named parameter idiom
  Type & set__button_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->button_id = _arg;
    return *this;
  }
  Type & set__navigate_to_staging_pose(
    const bool & _arg)
  {
    this->navigate_to_staging_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_Goal_ & other) const
  {
    if (this->button_id != other.button_id) {
      return false;
    }
    if (this->navigate_to_staging_pose != other.navigate_to_staging_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_Goal_

// alias to use template instance with default allocator
using PressCabinetButton_Goal =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Result __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Result __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_Result_
{
  using Type = PressCabinetButton_Result_<ContainerAllocator>;

  explicit PressCabinetButton_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0;
      this->message = "";
      this->max_travel = 0.0;
      this->failure_reason = "";
      this->physical_outcome_confirmed = false;
      this->final_state_verified = false;
      this->transport_succeeded = false;
      this->recovery_succeeded = false;
      this->grasp_released = false;
    }
  }

  explicit PressCabinetButton_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    failure_reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0;
      this->message = "";
      this->max_travel = 0.0;
      this->failure_reason = "";
      this->physical_outcome_confirmed = false;
      this->final_state_verified = false;
      this->transport_succeeded = false;
      this->recovery_succeeded = false;
      this->grasp_released = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_code_type =
    uint8_t;
  _error_code_type error_code;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _max_travel_type =
    double;
  _max_travel_type max_travel;
  using _failure_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _failure_reason_type failure_reason;
  using _physical_outcome_confirmed_type =
    bool;
  _physical_outcome_confirmed_type physical_outcome_confirmed;
  using _final_state_verified_type =
    bool;
  _final_state_verified_type final_state_verified;
  using _transport_succeeded_type =
    bool;
  _transport_succeeded_type transport_succeeded;
  using _recovery_succeeded_type =
    bool;
  _recovery_succeeded_type recovery_succeeded;
  using _grasp_released_type =
    bool;
  _grasp_released_type grasp_released;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint8_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__max_travel(
    const double & _arg)
  {
    this->max_travel = _arg;
    return *this;
  }
  Type & set__failure_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->failure_reason = _arg;
    return *this;
  }
  Type & set__physical_outcome_confirmed(
    const bool & _arg)
  {
    this->physical_outcome_confirmed = _arg;
    return *this;
  }
  Type & set__final_state_verified(
    const bool & _arg)
  {
    this->final_state_verified = _arg;
    return *this;
  }
  Type & set__transport_succeeded(
    const bool & _arg)
  {
    this->transport_succeeded = _arg;
    return *this;
  }
  Type & set__recovery_succeeded(
    const bool & _arg)
  {
    this->recovery_succeeded = _arg;
    return *this;
  }
  Type & set__grasp_released(
    const bool & _arg)
  {
    this->grasp_released = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SUCCESS =
    0u;
  static constexpr uint8_t INVALID_BUTTON =
    1u;
  static constexpr uint8_t NOT_READY =
    2u;
  static constexpr uint8_t NAVIGATION_FAILED =
    3u;
  static constexpr uint8_t PLANNING_FAILED =
    4u;
  static constexpr uint8_t EXECUTION_FAILED =
    5u;
  static constexpr uint8_t PRESS_NOT_DETECTED =
    6u;
  static constexpr uint8_t RELEASE_NOT_DETECTED =
    7u;
  static constexpr uint8_t CANCELED =
    8u;
  static constexpr uint8_t INTERNAL_ERROR =
    9u;
  static constexpr uint8_t RESOURCE_BUSY =
    10u;
  static constexpr uint8_t LEASE_LOST =
    11u;
  static constexpr uint8_t TOOLSET_MISMATCH =
    12u;
  static constexpr uint8_t ADAPTER_NOT_VALIDATED =
    13u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Result
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Result
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->max_travel != other.max_travel) {
      return false;
    }
    if (this->failure_reason != other.failure_reason) {
      return false;
    }
    if (this->physical_outcome_confirmed != other.physical_outcome_confirmed) {
      return false;
    }
    if (this->final_state_verified != other.final_state_verified) {
      return false;
    }
    if (this->transport_succeeded != other.transport_succeeded) {
      return false;
    }
    if (this->recovery_succeeded != other.recovery_succeeded) {
      return false;
    }
    if (this->grasp_released != other.grasp_released) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_Result_

// alias to use template instance with default allocator
using PressCabinetButton_Result =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::SUCCESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::INVALID_BUTTON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::NOT_READY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::NAVIGATION_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::PLANNING_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::EXECUTION_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::PRESS_NOT_DETECTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::RELEASE_NOT_DETECTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::CANCELED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::INTERNAL_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::RESOURCE_BUSY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::LEASE_LOST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::TOOLSET_MISMATCH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Result_<ContainerAllocator>::ADAPTER_NOT_VALIDATED;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_Feedback_
{
  using Type = PressCabinetButton_Feedback_<ContainerAllocator>;

  explicit PressCabinetButton_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->progress = 0.0f;
      this->button_travel = 0.0;
      this->message = "";
    }
  }

  explicit PressCabinetButton_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->progress = 0.0f;
      this->button_travel = 0.0;
      this->message = "";
    }
  }

  // field types and members
  using _phase_type =
    uint8_t;
  _phase_type phase;
  using _progress_type =
    float;
  _progress_type progress;
  using _button_travel_type =
    double;
  _button_travel_type button_travel;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__phase(
    const uint8_t & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__progress(
    const float & _arg)
  {
    this->progress = _arg;
    return *this;
  }
  Type & set__button_travel(
    const double & _arg)
  {
    this->button_travel = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t WAITING_FOR_SYSTEM =
    0u;
  static constexpr uint8_t NAVIGATING =
    1u;
  static constexpr uint8_t MOVING_TO_PREPRESS =
    2u;
  static constexpr uint8_t APPROACHING =
    3u;
  static constexpr uint8_t PRESSING =
    4u;
  static constexpr uint8_t VERIFYING_PRESS =
    5u;
  static constexpr uint8_t RETRACTING =
    6u;
  static constexpr uint8_t VERIFYING_RELEASE =
    7u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_Feedback_ & other) const
  {
    if (this->phase != other.phase) {
      return false;
    }
    if (this->progress != other.progress) {
      return false;
    }
    if (this->button_travel != other.button_travel) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_Feedback_

// alias to use template instance with default allocator
using PressCabinetButton_Feedback =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::WAITING_FOR_SYSTEM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::NAVIGATING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::MOVING_TO_PREPRESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::APPROACHING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::PRESSING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::VERIFYING_PRESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::RETRACTING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PressCabinetButton_Feedback_<ContainerAllocator>::VERIFYING_RELEASE;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "xczs_inspection_robot_interfaces/action/detail/press_cabinet_button__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_SendGoal_Request_
{
  using Type = PressCabinetButton_SendGoal_Request_<ContainerAllocator>;

  explicit PressCabinetButton_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PressCabinetButton_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_SendGoal_Request_

// alias to use template instance with default allocator
using PressCabinetButton_SendGoal_Request =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_SendGoal_Response_
{
  using Type = PressCabinetButton_SendGoal_Response_<ContainerAllocator>;

  explicit PressCabinetButton_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PressCabinetButton_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_SendGoal_Response_

// alias to use template instance with default allocator
using PressCabinetButton_SendGoal_Response =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces

namespace xczs_inspection_robot_interfaces
{

namespace action
{

struct PressCabinetButton_SendGoal
{
  using Request = xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request;
  using Response = xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response;
};

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_GetResult_Request_
{
  using Type = PressCabinetButton_GetResult_Request_<ContainerAllocator>;

  explicit PressCabinetButton_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PressCabinetButton_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_GetResult_Request_

// alias to use template instance with default allocator
using PressCabinetButton_GetResult_Request =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/press_cabinet_button__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_GetResult_Response_
{
  using Type = PressCabinetButton_GetResult_Response_<ContainerAllocator>;

  explicit PressCabinetButton_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PressCabinetButton_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_GetResult_Response_

// alias to use template instance with default allocator
using PressCabinetButton_GetResult_Response =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces

namespace xczs_inspection_robot_interfaces
{

namespace action
{

struct PressCabinetButton_GetResult
{
  using Request = xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request;
  using Response = xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response;
};

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/press_cabinet_button__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage __declspec(deprecated)
#endif

namespace xczs_inspection_robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PressCabinetButton_FeedbackMessage_
{
  using Type = PressCabinetButton_FeedbackMessage_<ContainerAllocator>;

  explicit PressCabinetButton_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PressCabinetButton_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage
    std::shared_ptr<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressCabinetButton_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressCabinetButton_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressCabinetButton_FeedbackMessage_

// alias to use template instance with default allocator
using PressCabinetButton_FeedbackMessage =
  xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace action
{

struct PressCabinetButton
{
  /// The goal message defined in the action definition.
  using Goal = xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal;
  /// The result message defined in the action definition.
  using Result = xczs_inspection_robot_interfaces::action::PressCabinetButton_Result;
  /// The feedback message defined in the action definition.
  using Feedback = xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PressCabinetButton PressCabinetButton;

}  // namespace action

}  // namespace xczs_inspection_robot_interfaces

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__PRESS_CABINET_BUTTON__STRUCT_HPP_
