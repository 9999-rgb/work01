// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xczs_inspection_robot_control:action/OperateCabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__STRUCT_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Goal __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Goal __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_Goal_
{
  using Type = OperateCabinetControl_Goal_<ContainerAllocator>;

  explicit OperateCabinetControl_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->command = 0;
      this->target_state = "";
      this->target_position = 0.0;
      this->use_target_position = false;
      this->force = 0.0;
      this->navigate_to_staging_pose = false;
    }
  }

  explicit OperateCabinetControl_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control_id(_alloc),
    target_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_id = "";
      this->command = 0;
      this->target_state = "";
      this->target_position = 0.0;
      this->use_target_position = false;
      this->force = 0.0;
      this->navigate_to_staging_pose = false;
    }
  }

  // field types and members
  using _control_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_id_type control_id;
  using _command_type =
    uint8_t;
  _command_type command;
  using _target_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_state_type target_state;
  using _target_position_type =
    double;
  _target_position_type target_position;
  using _use_target_position_type =
    bool;
  _use_target_position_type use_target_position;
  using _force_type =
    double;
  _force_type force;
  using _navigate_to_staging_pose_type =
    bool;
  _navigate_to_staging_pose_type navigate_to_staging_pose;

  // setters for named parameter idiom
  Type & set__control_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_id = _arg;
    return *this;
  }
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__target_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_state = _arg;
    return *this;
  }
  Type & set__target_position(
    const double & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__use_target_position(
    const bool & _arg)
  {
    this->use_target_position = _arg;
    return *this;
  }
  Type & set__force(
    const double & _arg)
  {
    this->force = _arg;
    return *this;
  }
  Type & set__navigate_to_staging_pose(
    const bool & _arg)
  {
    this->navigate_to_staging_pose = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t COMMAND_PRESS =
    0u;
  static constexpr uint8_t COMMAND_SET_STATE =
    1u;
  static constexpr uint8_t COMMAND_SET_POSITION =
    2u;
  static constexpr uint8_t COMMAND_TOGGLE =
    3u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Goal
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Goal
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_Goal_ & other) const
  {
    if (this->control_id != other.control_id) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->target_state != other.target_state) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->use_target_position != other.use_target_position) {
      return false;
    }
    if (this->force != other.force) {
      return false;
    }
    if (this->navigate_to_staging_pose != other.navigate_to_staging_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperateCabinetControl_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_Goal_

// alias to use template instance with default allocator
using OperateCabinetControl_Goal =
  xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Goal_<ContainerAllocator>::COMMAND_PRESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Goal_<ContainerAllocator>::COMMAND_SET_STATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Goal_<ContainerAllocator>::COMMAND_SET_POSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Goal_<ContainerAllocator>::COMMAND_TOGGLE;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace xczs_inspection_robot_control


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Result __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Result __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_Result_
{
  using Type = OperateCabinetControl_Result_<ContainerAllocator>;

  explicit OperateCabinetControl_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0;
      this->message = "";
      this->initial_position = 0.0;
      this->final_position = 0.0;
      this->peak_position = 0.0;
      this->final_state = "";
      this->requested_force = 0.0;
      this->estimated_force = 0.0;
      this->button_triggered = false;
      this->validation_performed = false;
      this->operation_executed = false;
      this->diagnostic_stage = "";
      this->path_fraction = 0.0;
      this->required_fraction = 0.0;
      this->moveit_error_code = 0l;
      this->policy_reason = "";
      this->failure_reason = "";
      this->physical_outcome_confirmed = false;
      this->final_state_verified = false;
      this->transport_succeeded = false;
      this->recovery_succeeded = false;
      this->grasp_released = false;
    }
  }

  explicit OperateCabinetControl_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    final_state(_alloc),
    diagnostic_stage(_alloc),
    policy_reason(_alloc),
    failure_reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0;
      this->message = "";
      this->initial_position = 0.0;
      this->final_position = 0.0;
      this->peak_position = 0.0;
      this->final_state = "";
      this->requested_force = 0.0;
      this->estimated_force = 0.0;
      this->button_triggered = false;
      this->validation_performed = false;
      this->operation_executed = false;
      this->diagnostic_stage = "";
      this->path_fraction = 0.0;
      this->required_fraction = 0.0;
      this->moveit_error_code = 0l;
      this->policy_reason = "";
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
  using _initial_position_type =
    double;
  _initial_position_type initial_position;
  using _final_position_type =
    double;
  _final_position_type final_position;
  using _peak_position_type =
    double;
  _peak_position_type peak_position;
  using _final_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _final_state_type final_state;
  using _requested_force_type =
    double;
  _requested_force_type requested_force;
  using _estimated_force_type =
    double;
  _estimated_force_type estimated_force;
  using _button_triggered_type =
    bool;
  _button_triggered_type button_triggered;
  using _validation_performed_type =
    bool;
  _validation_performed_type validation_performed;
  using _operation_executed_type =
    bool;
  _operation_executed_type operation_executed;
  using _diagnostic_stage_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _diagnostic_stage_type diagnostic_stage;
  using _path_fraction_type =
    double;
  _path_fraction_type path_fraction;
  using _required_fraction_type =
    double;
  _required_fraction_type required_fraction;
  using _moveit_error_code_type =
    int32_t;
  _moveit_error_code_type moveit_error_code;
  using _policy_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _policy_reason_type policy_reason;
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
  Type & set__initial_position(
    const double & _arg)
  {
    this->initial_position = _arg;
    return *this;
  }
  Type & set__final_position(
    const double & _arg)
  {
    this->final_position = _arg;
    return *this;
  }
  Type & set__peak_position(
    const double & _arg)
  {
    this->peak_position = _arg;
    return *this;
  }
  Type & set__final_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->final_state = _arg;
    return *this;
  }
  Type & set__requested_force(
    const double & _arg)
  {
    this->requested_force = _arg;
    return *this;
  }
  Type & set__estimated_force(
    const double & _arg)
  {
    this->estimated_force = _arg;
    return *this;
  }
  Type & set__button_triggered(
    const bool & _arg)
  {
    this->button_triggered = _arg;
    return *this;
  }
  Type & set__validation_performed(
    const bool & _arg)
  {
    this->validation_performed = _arg;
    return *this;
  }
  Type & set__operation_executed(
    const bool & _arg)
  {
    this->operation_executed = _arg;
    return *this;
  }
  Type & set__diagnostic_stage(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->diagnostic_stage = _arg;
    return *this;
  }
  Type & set__path_fraction(
    const double & _arg)
  {
    this->path_fraction = _arg;
    return *this;
  }
  Type & set__required_fraction(
    const double & _arg)
  {
    this->required_fraction = _arg;
    return *this;
  }
  Type & set__moveit_error_code(
    const int32_t & _arg)
  {
    this->moveit_error_code = _arg;
    return *this;
  }
  Type & set__policy_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->policy_reason = _arg;
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
  static constexpr uint8_t INVALID_CONTROL =
    1u;
  static constexpr uint8_t UNSUPPORTED_COMMAND =
    2u;
  static constexpr uint8_t NOT_READY =
    3u;
  static constexpr uint8_t NAVIGATION_FAILED =
    4u;
  static constexpr uint8_t PLANNING_FAILED =
    5u;
  static constexpr uint8_t EXECUTION_FAILED =
    6u;
  static constexpr uint8_t GRASP_FAILED =
    7u;
  static constexpr uint8_t TARGET_NOT_REACHED =
    8u;
  static constexpr uint8_t RELEASE_FAILED =
    9u;
  static constexpr uint8_t CANCELED =
    10u;
  static constexpr uint8_t INTERNAL_ERROR =
    11u;
  static constexpr uint8_t INVALID_FORCE =
    12u;
  static constexpr uint8_t INSUFFICIENT_FORCE =
    13u;
  static constexpr uint8_t UNREACHABLE =
    14u;
  static constexpr uint8_t CONTACT_DETECTION_TIMEOUT =
    15u;
  static constexpr uint8_t RESOURCE_BUSY =
    16u;
  static constexpr uint8_t LEASE_LOST =
    17u;
  static constexpr uint8_t TOOLSET_MISMATCH =
    18u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Result
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Result
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_Result_ & other) const
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
    if (this->initial_position != other.initial_position) {
      return false;
    }
    if (this->final_position != other.final_position) {
      return false;
    }
    if (this->peak_position != other.peak_position) {
      return false;
    }
    if (this->final_state != other.final_state) {
      return false;
    }
    if (this->requested_force != other.requested_force) {
      return false;
    }
    if (this->estimated_force != other.estimated_force) {
      return false;
    }
    if (this->button_triggered != other.button_triggered) {
      return false;
    }
    if (this->validation_performed != other.validation_performed) {
      return false;
    }
    if (this->operation_executed != other.operation_executed) {
      return false;
    }
    if (this->diagnostic_stage != other.diagnostic_stage) {
      return false;
    }
    if (this->path_fraction != other.path_fraction) {
      return false;
    }
    if (this->required_fraction != other.required_fraction) {
      return false;
    }
    if (this->moveit_error_code != other.moveit_error_code) {
      return false;
    }
    if (this->policy_reason != other.policy_reason) {
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
  bool operator!=(const OperateCabinetControl_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_Result_

// alias to use template instance with default allocator
using OperateCabinetControl_Result =
  xczs_inspection_robot_control::action::OperateCabinetControl_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::SUCCESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::INVALID_CONTROL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::UNSUPPORTED_COMMAND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::NOT_READY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::NAVIGATION_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::PLANNING_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::EXECUTION_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::GRASP_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::TARGET_NOT_REACHED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::RELEASE_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::CANCELED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::INTERNAL_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::INVALID_FORCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::INSUFFICIENT_FORCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::UNREACHABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::CONTACT_DETECTION_TIMEOUT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::RESOURCE_BUSY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::LEASE_LOST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Result_<ContainerAllocator>::TOOLSET_MISMATCH;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace xczs_inspection_robot_control


#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Feedback __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_Feedback_
{
  using Type = OperateCabinetControl_Feedback_<ContainerAllocator>;

  explicit OperateCabinetControl_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->progress = 0.0f;
      this->current_position = 0.0;
      this->target_position = 0.0;
      this->current_state = "";
      this->message = "";
    }
  }

  explicit OperateCabinetControl_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_state(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = 0;
      this->progress = 0.0f;
      this->current_position = 0.0;
      this->target_position = 0.0;
      this->current_state = "";
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
  using _current_position_type =
    double;
  _current_position_type current_position;
  using _target_position_type =
    double;
  _target_position_type target_position;
  using _current_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_state_type current_state;
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
  Type & set__current_position(
    const double & _arg)
  {
    this->current_position = _arg;
    return *this;
  }
  Type & set__target_position(
    const double & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__current_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_state = _arg;
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
  static constexpr uint8_t DOCKING =
    2u;
  static constexpr uint8_t MOVING_TO_READY =
    3u;
  static constexpr uint8_t APPROACHING =
    4u;
  static constexpr uint8_t GRASPING =
    5u;
  static constexpr uint8_t MANIPULATING =
    6u;
  static constexpr uint8_t VERIFYING =
    7u;
  static constexpr uint8_t RELEASING =
    8u;
  static constexpr uint8_t RETREATING =
    9u;

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Feedback
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_Feedback
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_Feedback_ & other) const
  {
    if (this->phase != other.phase) {
      return false;
    }
    if (this->progress != other.progress) {
      return false;
    }
    if (this->current_position != other.current_position) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->current_state != other.current_state) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperateCabinetControl_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_Feedback_

// alias to use template instance with default allocator
using OperateCabinetControl_Feedback =
  xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::WAITING_FOR_SYSTEM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::NAVIGATING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::DOCKING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::MOVING_TO_READY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::APPROACHING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::GRASPING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::MANIPULATING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::VERIFYING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::RELEASING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperateCabinetControl_Feedback_<ContainerAllocator>::RETREATING;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace xczs_inspection_robot_control


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_SendGoal_Request_
{
  using Type = OperateCabinetControl_SendGoal_Request_<ContainerAllocator>;

  explicit OperateCabinetControl_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit OperateCabinetControl_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const xczs_inspection_robot_control::action::OperateCabinetControl_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperateCabinetControl_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_SendGoal_Request_

// alias to use template instance with default allocator
using OperateCabinetControl_SendGoal_Request =
  xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_control


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_SendGoal_Response_
{
  using Type = OperateCabinetControl_SendGoal_Response_<ContainerAllocator>;

  explicit OperateCabinetControl_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit OperateCabinetControl_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperateCabinetControl_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_SendGoal_Response_

// alias to use template instance with default allocator
using OperateCabinetControl_SendGoal_Response =
  xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace xczs_inspection_robot_control
{

namespace action
{

struct OperateCabinetControl_SendGoal
{
  using Request = xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request;
  using Response = xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response;
};

}  // namespace action

}  // namespace xczs_inspection_robot_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_GetResult_Request_
{
  using Type = OperateCabinetControl_GetResult_Request_<ContainerAllocator>;

  explicit OperateCabinetControl_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit OperateCabinetControl_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperateCabinetControl_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_GetResult_Request_

// alias to use template instance with default allocator
using OperateCabinetControl_GetResult_Request =
  xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_control


// Include directives for member types
// Member 'result'
// already included above
// #include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_GetResult_Response_
{
  using Type = OperateCabinetControl_GetResult_Response_<ContainerAllocator>;

  explicit OperateCabinetControl_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit OperateCabinetControl_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const xczs_inspection_robot_control::action::OperateCabinetControl_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperateCabinetControl_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_GetResult_Response_

// alias to use template instance with default allocator
using OperateCabinetControl_GetResult_Response =
  xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace xczs_inspection_robot_control
{

namespace action
{

struct OperateCabinetControl_GetResult
{
  using Request = xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request;
  using Response = xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response;
};

}  // namespace action

}  // namespace xczs_inspection_robot_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage __declspec(deprecated)
#endif

namespace xczs_inspection_robot_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct OperateCabinetControl_FeedbackMessage_
{
  using Type = OperateCabinetControl_FeedbackMessage_<ContainerAllocator>;

  explicit OperateCabinetControl_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit OperateCabinetControl_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const xczs_inspection_robot_control::action::OperateCabinetControl_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage
    std::shared_ptr<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperateCabinetControl_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperateCabinetControl_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperateCabinetControl_FeedbackMessage_

// alias to use template instance with default allocator
using OperateCabinetControl_FeedbackMessage =
  xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace xczs_inspection_robot_control

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace xczs_inspection_robot_control
{

namespace action
{

struct OperateCabinetControl
{
  /// The goal message defined in the action definition.
  using Goal = xczs_inspection_robot_control::action::OperateCabinetControl_Goal;
  /// The result message defined in the action definition.
  using Result = xczs_inspection_robot_control::action::OperateCabinetControl_Result;
  /// The feedback message defined in the action definition.
  using Feedback = xczs_inspection_robot_control::action::OperateCabinetControl_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = xczs_inspection_robot_control::action::OperateCabinetControl_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct OperateCabinetControl OperateCabinetControl;

}  // namespace action

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__STRUCT_HPP_
