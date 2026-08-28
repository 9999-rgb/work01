// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_control:action/OperateCabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_Goal_navigate_to_staging_pose
{
public:
  explicit Init_OperateCabinetControl_Goal_navigate_to_staging_pose(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal navigate_to_staging_pose(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal::_navigate_to_staging_pose_type arg)
  {
    msg_.navigate_to_staging_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal msg_;
};

class Init_OperateCabinetControl_Goal_force
{
public:
  explicit Init_OperateCabinetControl_Goal_force(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Goal_navigate_to_staging_pose force(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_OperateCabinetControl_Goal_navigate_to_staging_pose(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal msg_;
};

class Init_OperateCabinetControl_Goal_use_target_position
{
public:
  explicit Init_OperateCabinetControl_Goal_use_target_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Goal_force use_target_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal::_use_target_position_type arg)
  {
    msg_.use_target_position = std::move(arg);
    return Init_OperateCabinetControl_Goal_force(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal msg_;
};

class Init_OperateCabinetControl_Goal_target_position
{
public:
  explicit Init_OperateCabinetControl_Goal_target_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Goal_use_target_position target_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_OperateCabinetControl_Goal_use_target_position(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal msg_;
};

class Init_OperateCabinetControl_Goal_target_state
{
public:
  explicit Init_OperateCabinetControl_Goal_target_state(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Goal_target_position target_state(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal::_target_state_type arg)
  {
    msg_.target_state = std::move(arg);
    return Init_OperateCabinetControl_Goal_target_position(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal msg_;
};

class Init_OperateCabinetControl_Goal_command
{
public:
  explicit Init_OperateCabinetControl_Goal_command(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Goal_target_state command(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_OperateCabinetControl_Goal_target_state(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal msg_;
};

class Init_OperateCabinetControl_Goal_control_id
{
public:
  Init_OperateCabinetControl_Goal_control_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperateCabinetControl_Goal_command control_id(::xczs_inspection_robot_control::action::OperateCabinetControl_Goal::_control_id_type arg)
  {
    msg_.control_id = std::move(arg);
    return Init_OperateCabinetControl_Goal_command(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_Goal>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_Goal_control_id();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_Result_grasp_released
{
public:
  explicit Init_OperateCabinetControl_Result_grasp_released(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result grasp_released(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_grasp_released_type arg)
  {
    msg_.grasp_released = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_recovery_succeeded
{
public:
  explicit Init_OperateCabinetControl_Result_recovery_succeeded(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_grasp_released recovery_succeeded(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_recovery_succeeded_type arg)
  {
    msg_.recovery_succeeded = std::move(arg);
    return Init_OperateCabinetControl_Result_grasp_released(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_transport_succeeded
{
public:
  explicit Init_OperateCabinetControl_Result_transport_succeeded(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_recovery_succeeded transport_succeeded(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_transport_succeeded_type arg)
  {
    msg_.transport_succeeded = std::move(arg);
    return Init_OperateCabinetControl_Result_recovery_succeeded(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_final_state_verified
{
public:
  explicit Init_OperateCabinetControl_Result_final_state_verified(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_transport_succeeded final_state_verified(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_final_state_verified_type arg)
  {
    msg_.final_state_verified = std::move(arg);
    return Init_OperateCabinetControl_Result_transport_succeeded(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_physical_outcome_confirmed
{
public:
  explicit Init_OperateCabinetControl_Result_physical_outcome_confirmed(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_final_state_verified physical_outcome_confirmed(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_physical_outcome_confirmed_type arg)
  {
    msg_.physical_outcome_confirmed = std::move(arg);
    return Init_OperateCabinetControl_Result_final_state_verified(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_failure_reason
{
public:
  explicit Init_OperateCabinetControl_Result_failure_reason(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_physical_outcome_confirmed failure_reason(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_failure_reason_type arg)
  {
    msg_.failure_reason = std::move(arg);
    return Init_OperateCabinetControl_Result_physical_outcome_confirmed(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_policy_reason
{
public:
  explicit Init_OperateCabinetControl_Result_policy_reason(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_failure_reason policy_reason(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_policy_reason_type arg)
  {
    msg_.policy_reason = std::move(arg);
    return Init_OperateCabinetControl_Result_failure_reason(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_moveit_error_code
{
public:
  explicit Init_OperateCabinetControl_Result_moveit_error_code(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_policy_reason moveit_error_code(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_moveit_error_code_type arg)
  {
    msg_.moveit_error_code = std::move(arg);
    return Init_OperateCabinetControl_Result_policy_reason(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_required_fraction
{
public:
  explicit Init_OperateCabinetControl_Result_required_fraction(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_moveit_error_code required_fraction(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_required_fraction_type arg)
  {
    msg_.required_fraction = std::move(arg);
    return Init_OperateCabinetControl_Result_moveit_error_code(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_path_fraction
{
public:
  explicit Init_OperateCabinetControl_Result_path_fraction(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_required_fraction path_fraction(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_path_fraction_type arg)
  {
    msg_.path_fraction = std::move(arg);
    return Init_OperateCabinetControl_Result_required_fraction(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_diagnostic_stage
{
public:
  explicit Init_OperateCabinetControl_Result_diagnostic_stage(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_path_fraction diagnostic_stage(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_diagnostic_stage_type arg)
  {
    msg_.diagnostic_stage = std::move(arg);
    return Init_OperateCabinetControl_Result_path_fraction(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_operation_executed
{
public:
  explicit Init_OperateCabinetControl_Result_operation_executed(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_diagnostic_stage operation_executed(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_operation_executed_type arg)
  {
    msg_.operation_executed = std::move(arg);
    return Init_OperateCabinetControl_Result_diagnostic_stage(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_validation_performed
{
public:
  explicit Init_OperateCabinetControl_Result_validation_performed(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_operation_executed validation_performed(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_validation_performed_type arg)
  {
    msg_.validation_performed = std::move(arg);
    return Init_OperateCabinetControl_Result_operation_executed(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_button_triggered
{
public:
  explicit Init_OperateCabinetControl_Result_button_triggered(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_validation_performed button_triggered(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_button_triggered_type arg)
  {
    msg_.button_triggered = std::move(arg);
    return Init_OperateCabinetControl_Result_validation_performed(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_estimated_force
{
public:
  explicit Init_OperateCabinetControl_Result_estimated_force(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_button_triggered estimated_force(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_estimated_force_type arg)
  {
    msg_.estimated_force = std::move(arg);
    return Init_OperateCabinetControl_Result_button_triggered(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_requested_force
{
public:
  explicit Init_OperateCabinetControl_Result_requested_force(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_estimated_force requested_force(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_requested_force_type arg)
  {
    msg_.requested_force = std::move(arg);
    return Init_OperateCabinetControl_Result_estimated_force(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_final_state
{
public:
  explicit Init_OperateCabinetControl_Result_final_state(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_requested_force final_state(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_final_state_type arg)
  {
    msg_.final_state = std::move(arg);
    return Init_OperateCabinetControl_Result_requested_force(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_peak_position
{
public:
  explicit Init_OperateCabinetControl_Result_peak_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_final_state peak_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_peak_position_type arg)
  {
    msg_.peak_position = std::move(arg);
    return Init_OperateCabinetControl_Result_final_state(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_final_position
{
public:
  explicit Init_OperateCabinetControl_Result_final_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_peak_position final_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_final_position_type arg)
  {
    msg_.final_position = std::move(arg);
    return Init_OperateCabinetControl_Result_peak_position(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_initial_position
{
public:
  explicit Init_OperateCabinetControl_Result_initial_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_final_position initial_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_initial_position_type arg)
  {
    msg_.initial_position = std::move(arg);
    return Init_OperateCabinetControl_Result_final_position(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_message
{
public:
  explicit Init_OperateCabinetControl_Result_message(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_initial_position message(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_OperateCabinetControl_Result_initial_position(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_error_code
{
public:
  explicit Init_OperateCabinetControl_Result_error_code(::xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Result_message error_code(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_OperateCabinetControl_Result_message(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

class Init_OperateCabinetControl_Result_success
{
public:
  Init_OperateCabinetControl_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperateCabinetControl_Result_error_code success(::xczs_inspection_robot_control::action::OperateCabinetControl_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_OperateCabinetControl_Result_error_code(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_Result>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_Result_success();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_Feedback_message
{
public:
  explicit Init_OperateCabinetControl_Feedback_message(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback message(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback msg_;
};

class Init_OperateCabinetControl_Feedback_current_state
{
public:
  explicit Init_OperateCabinetControl_Feedback_current_state(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Feedback_message current_state(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_OperateCabinetControl_Feedback_message(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback msg_;
};

class Init_OperateCabinetControl_Feedback_target_position
{
public:
  explicit Init_OperateCabinetControl_Feedback_target_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Feedback_current_state target_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_OperateCabinetControl_Feedback_current_state(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback msg_;
};

class Init_OperateCabinetControl_Feedback_current_position
{
public:
  explicit Init_OperateCabinetControl_Feedback_current_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Feedback_target_position current_position(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback::_current_position_type arg)
  {
    msg_.current_position = std::move(arg);
    return Init_OperateCabinetControl_Feedback_target_position(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback msg_;
};

class Init_OperateCabinetControl_Feedback_progress
{
public:
  explicit Init_OperateCabinetControl_Feedback_progress(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback & msg)
  : msg_(msg)
  {}
  Init_OperateCabinetControl_Feedback_current_position progress(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_OperateCabinetControl_Feedback_current_position(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback msg_;
};

class Init_OperateCabinetControl_Feedback_phase
{
public:
  Init_OperateCabinetControl_Feedback_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperateCabinetControl_Feedback_progress phase(::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_OperateCabinetControl_Feedback_progress(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_Feedback_phase();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_SendGoal_Request_goal
{
public:
  explicit Init_OperateCabinetControl_SendGoal_Request_goal(::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request goal(::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request msg_;
};

class Init_OperateCabinetControl_SendGoal_Request_goal_id
{
public:
  Init_OperateCabinetControl_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperateCabinetControl_SendGoal_Request_goal goal_id(::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_OperateCabinetControl_SendGoal_Request_goal(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_SendGoal_Request_goal_id();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_SendGoal_Response_stamp
{
public:
  explicit Init_OperateCabinetControl_SendGoal_Response_stamp(::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response stamp(::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response msg_;
};

class Init_OperateCabinetControl_SendGoal_Response_accepted
{
public:
  Init_OperateCabinetControl_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperateCabinetControl_SendGoal_Response_stamp accepted(::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_OperateCabinetControl_SendGoal_Response_stamp(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_SendGoal_Response_accepted();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_GetResult_Request_goal_id
{
public:
  Init_OperateCabinetControl_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request goal_id(::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_GetResult_Request_goal_id();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_GetResult_Response_result
{
public:
  explicit Init_OperateCabinetControl_GetResult_Response_result(::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response result(::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response msg_;
};

class Init_OperateCabinetControl_GetResult_Response_status
{
public:
  Init_OperateCabinetControl_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperateCabinetControl_GetResult_Response_result status(::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_OperateCabinetControl_GetResult_Response_result(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_GetResult_Response_status();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_OperateCabinetControl_FeedbackMessage_feedback
{
public:
  explicit Init_OperateCabinetControl_FeedbackMessage_feedback(::xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage feedback(::xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage msg_;
};

class Init_OperateCabinetControl_FeedbackMessage_goal_id
{
public:
  Init_OperateCabinetControl_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperateCabinetControl_FeedbackMessage_feedback goal_id(::xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_OperateCabinetControl_FeedbackMessage_feedback(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage>()
{
  return xczs_inspection_robot_control::action::builder::Init_OperateCabinetControl_FeedbackMessage_goal_id();
}

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__BUILDER_HPP_
