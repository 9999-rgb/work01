// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xczs_inspection_robot_control:action/PressCabinetButton.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__PRESS_CABINET_BUTTON__BUILDER_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__PRESS_CABINET_BUTTON__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xczs_inspection_robot_control/action/detail/press_cabinet_button__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_Goal_navigate_to_staging_pose
{
public:
  explicit Init_PressCabinetButton_Goal_navigate_to_staging_pose(::xczs_inspection_robot_control::action::PressCabinetButton_Goal & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_Goal navigate_to_staging_pose(::xczs_inspection_robot_control::action::PressCabinetButton_Goal::_navigate_to_staging_pose_type arg)
  {
    msg_.navigate_to_staging_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Goal msg_;
};

class Init_PressCabinetButton_Goal_button_id
{
public:
  Init_PressCabinetButton_Goal_button_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressCabinetButton_Goal_navigate_to_staging_pose button_id(::xczs_inspection_robot_control::action::PressCabinetButton_Goal::_button_id_type arg)
  {
    msg_.button_id = std::move(arg);
    return Init_PressCabinetButton_Goal_navigate_to_staging_pose(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_Goal>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_Goal_button_id();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_Result_grasp_released
{
public:
  explicit Init_PressCabinetButton_Result_grasp_released(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result grasp_released(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_grasp_released_type arg)
  {
    msg_.grasp_released = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_recovery_succeeded
{
public:
  explicit Init_PressCabinetButton_Result_recovery_succeeded(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_grasp_released recovery_succeeded(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_recovery_succeeded_type arg)
  {
    msg_.recovery_succeeded = std::move(arg);
    return Init_PressCabinetButton_Result_grasp_released(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_transport_succeeded
{
public:
  explicit Init_PressCabinetButton_Result_transport_succeeded(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_recovery_succeeded transport_succeeded(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_transport_succeeded_type arg)
  {
    msg_.transport_succeeded = std::move(arg);
    return Init_PressCabinetButton_Result_recovery_succeeded(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_final_state_verified
{
public:
  explicit Init_PressCabinetButton_Result_final_state_verified(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_transport_succeeded final_state_verified(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_final_state_verified_type arg)
  {
    msg_.final_state_verified = std::move(arg);
    return Init_PressCabinetButton_Result_transport_succeeded(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_physical_outcome_confirmed
{
public:
  explicit Init_PressCabinetButton_Result_physical_outcome_confirmed(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_final_state_verified physical_outcome_confirmed(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_physical_outcome_confirmed_type arg)
  {
    msg_.physical_outcome_confirmed = std::move(arg);
    return Init_PressCabinetButton_Result_final_state_verified(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_failure_reason
{
public:
  explicit Init_PressCabinetButton_Result_failure_reason(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_physical_outcome_confirmed failure_reason(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_failure_reason_type arg)
  {
    msg_.failure_reason = std::move(arg);
    return Init_PressCabinetButton_Result_physical_outcome_confirmed(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_max_travel
{
public:
  explicit Init_PressCabinetButton_Result_max_travel(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_failure_reason max_travel(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_max_travel_type arg)
  {
    msg_.max_travel = std::move(arg);
    return Init_PressCabinetButton_Result_failure_reason(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_message
{
public:
  explicit Init_PressCabinetButton_Result_message(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_max_travel message(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_PressCabinetButton_Result_max_travel(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_error_code
{
public:
  explicit Init_PressCabinetButton_Result_error_code(::xczs_inspection_robot_control::action::PressCabinetButton_Result & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Result_message error_code(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_PressCabinetButton_Result_message(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

class Init_PressCabinetButton_Result_success
{
public:
  Init_PressCabinetButton_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressCabinetButton_Result_error_code success(::xczs_inspection_robot_control::action::PressCabinetButton_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PressCabinetButton_Result_error_code(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_Result>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_Result_success();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_Feedback_message
{
public:
  explicit Init_PressCabinetButton_Feedback_message(::xczs_inspection_robot_control::action::PressCabinetButton_Feedback & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_Feedback message(::xczs_inspection_robot_control::action::PressCabinetButton_Feedback::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Feedback msg_;
};

class Init_PressCabinetButton_Feedback_button_travel
{
public:
  explicit Init_PressCabinetButton_Feedback_button_travel(::xczs_inspection_robot_control::action::PressCabinetButton_Feedback & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Feedback_message button_travel(::xczs_inspection_robot_control::action::PressCabinetButton_Feedback::_button_travel_type arg)
  {
    msg_.button_travel = std::move(arg);
    return Init_PressCabinetButton_Feedback_message(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Feedback msg_;
};

class Init_PressCabinetButton_Feedback_progress
{
public:
  explicit Init_PressCabinetButton_Feedback_progress(::xczs_inspection_robot_control::action::PressCabinetButton_Feedback & msg)
  : msg_(msg)
  {}
  Init_PressCabinetButton_Feedback_button_travel progress(::xczs_inspection_robot_control::action::PressCabinetButton_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_PressCabinetButton_Feedback_button_travel(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Feedback msg_;
};

class Init_PressCabinetButton_Feedback_phase
{
public:
  Init_PressCabinetButton_Feedback_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressCabinetButton_Feedback_progress phase(::xczs_inspection_robot_control::action::PressCabinetButton_Feedback::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_PressCabinetButton_Feedback_progress(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_Feedback>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_Feedback_phase();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_SendGoal_Request_goal
{
public:
  explicit Init_PressCabinetButton_SendGoal_Request_goal(::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Request goal(::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Request msg_;
};

class Init_PressCabinetButton_SendGoal_Request_goal_id
{
public:
  Init_PressCabinetButton_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressCabinetButton_SendGoal_Request_goal goal_id(::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PressCabinetButton_SendGoal_Request_goal(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Request>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_SendGoal_Request_goal_id();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_SendGoal_Response_stamp
{
public:
  explicit Init_PressCabinetButton_SendGoal_Response_stamp(::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Response stamp(::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Response msg_;
};

class Init_PressCabinetButton_SendGoal_Response_accepted
{
public:
  Init_PressCabinetButton_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressCabinetButton_SendGoal_Response_stamp accepted(::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PressCabinetButton_SendGoal_Response_stamp(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_SendGoal_Response>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_SendGoal_Response_accepted();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_GetResult_Request_goal_id
{
public:
  Init_PressCabinetButton_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Request goal_id(::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Request>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_GetResult_Request_goal_id();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_GetResult_Response_result
{
public:
  explicit Init_PressCabinetButton_GetResult_Response_result(::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Response result(::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Response msg_;
};

class Init_PressCabinetButton_GetResult_Response_status
{
public:
  Init_PressCabinetButton_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressCabinetButton_GetResult_Response_result status(::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PressCabinetButton_GetResult_Response_result(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_GetResult_Response>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_GetResult_Response_status();
}

}  // namespace xczs_inspection_robot_control


namespace xczs_inspection_robot_control
{

namespace action
{

namespace builder
{

class Init_PressCabinetButton_FeedbackMessage_feedback
{
public:
  explicit Init_PressCabinetButton_FeedbackMessage_feedback(::xczs_inspection_robot_control::action::PressCabinetButton_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::xczs_inspection_robot_control::action::PressCabinetButton_FeedbackMessage feedback(::xczs_inspection_robot_control::action::PressCabinetButton_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_FeedbackMessage msg_;
};

class Init_PressCabinetButton_FeedbackMessage_goal_id
{
public:
  Init_PressCabinetButton_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PressCabinetButton_FeedbackMessage_feedback goal_id(::xczs_inspection_robot_control::action::PressCabinetButton_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PressCabinetButton_FeedbackMessage_feedback(msg_);
  }

private:
  ::xczs_inspection_robot_control::action::PressCabinetButton_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xczs_inspection_robot_control::action::PressCabinetButton_FeedbackMessage>()
{
  return xczs_inspection_robot_control::action::builder::Init_PressCabinetButton_FeedbackMessage_goal_id();
}

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__PRESS_CABINET_BUTTON__BUILDER_HPP_
