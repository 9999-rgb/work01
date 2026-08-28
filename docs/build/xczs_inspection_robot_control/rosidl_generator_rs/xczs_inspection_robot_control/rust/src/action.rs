
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub use_target_position: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub force: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigate_to_staging_pose: bool,

}

impl OperateCabinetControl_Goal {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const COMMAND_PRESS: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const COMMAND_SET_STATE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const COMMAND_SET_POSITION: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const COMMAND_TOGGLE: u8 = 3;

}


impl Default for OperateCabinetControl_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_Goal {
  type RmwMsg = super::action::rmw::OperateCabinetControl_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        control_id: msg.control_id.as_str().into(),
        command: msg.command,
        target_state: msg.target_state.as_str().into(),
        target_position: msg.target_position,
        use_target_position: msg.use_target_position,
        force: msg.force,
        navigate_to_staging_pose: msg.navigate_to_staging_pose,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        control_id: msg.control_id.as_str().into(),
      command: msg.command,
        target_state: msg.target_state.as_str().into(),
      target_position: msg.target_position,
      use_target_position: msg.use_target_position,
      force: msg.force,
      navigate_to_staging_pose: msg.navigate_to_staging_pose,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      control_id: msg.control_id.to_string(),
      command: msg.command,
      target_state: msg.target_state.to_string(),
      target_position: msg.target_position,
      use_target_position: msg.use_target_position,
      force: msg.force,
      navigate_to_staging_pose: msg.navigate_to_staging_pose,
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error_code: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub initial_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_position: f64,

    /// Maximum absolute joint position observed during this operation.  This keeps
    /// negative knob travel observable while remaining identical to button travel.
    pub peak_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub requested_force: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub estimated_force: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_triggered: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub validation_performed: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operation_executed: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub diagnostic_stage: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path_fraction: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub required_fraction: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub moveit_error_code: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub policy_reason: std::string::String,

    /// Always empty on success.  On failure this is the actionable cause and is
    /// kept separate from the short, user-facing message for machine clients.
    pub failure_reason: std::string::String,

    /// Terminal evidence flags.  They are intentionally conservative: each is
    /// true only after the corresponding physical side effect or recovery phase
    /// has been verified.
    pub physical_outcome_confirmed: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_state_verified: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub transport_succeeded: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub recovery_succeeded: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub grasp_released: bool,

}

impl OperateCabinetControl_Result {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SUCCESS: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INVALID_CONTROL: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNSUPPORTED_COMMAND: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NOT_READY: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NAVIGATION_FAILED: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PLANNING_FAILED: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXECUTION_FAILED: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GRASP_FAILED: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TARGET_NOT_REACHED: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RELEASE_FAILED: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CANCELED: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INTERNAL_ERROR: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INVALID_FORCE: u8 = 12;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INSUFFICIENT_FORCE: u8 = 13;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNREACHABLE: u8 = 14;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CONTACT_DETECTION_TIMEOUT: u8 = 15;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESOURCE_BUSY: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LEASE_LOST: u8 = 17;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TOOLSET_MISMATCH: u8 = 18;

}


impl Default for OperateCabinetControl_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_Result::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_Result {
  type RmwMsg = super::action::rmw::OperateCabinetControl_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        error_code: msg.error_code,
        message: msg.message.as_str().into(),
        initial_position: msg.initial_position,
        final_position: msg.final_position,
        peak_position: msg.peak_position,
        final_state: msg.final_state.as_str().into(),
        requested_force: msg.requested_force,
        estimated_force: msg.estimated_force,
        button_triggered: msg.button_triggered,
        validation_performed: msg.validation_performed,
        operation_executed: msg.operation_executed,
        diagnostic_stage: msg.diagnostic_stage.as_str().into(),
        path_fraction: msg.path_fraction,
        required_fraction: msg.required_fraction,
        moveit_error_code: msg.moveit_error_code,
        policy_reason: msg.policy_reason.as_str().into(),
        failure_reason: msg.failure_reason.as_str().into(),
        physical_outcome_confirmed: msg.physical_outcome_confirmed,
        final_state_verified: msg.final_state_verified,
        transport_succeeded: msg.transport_succeeded,
        recovery_succeeded: msg.recovery_succeeded,
        grasp_released: msg.grasp_released,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      error_code: msg.error_code,
        message: msg.message.as_str().into(),
      initial_position: msg.initial_position,
      final_position: msg.final_position,
      peak_position: msg.peak_position,
        final_state: msg.final_state.as_str().into(),
      requested_force: msg.requested_force,
      estimated_force: msg.estimated_force,
      button_triggered: msg.button_triggered,
      validation_performed: msg.validation_performed,
      operation_executed: msg.operation_executed,
        diagnostic_stage: msg.diagnostic_stage.as_str().into(),
      path_fraction: msg.path_fraction,
      required_fraction: msg.required_fraction,
      moveit_error_code: msg.moveit_error_code,
        policy_reason: msg.policy_reason.as_str().into(),
        failure_reason: msg.failure_reason.as_str().into(),
      physical_outcome_confirmed: msg.physical_outcome_confirmed,
      final_state_verified: msg.final_state_verified,
      transport_succeeded: msg.transport_succeeded,
      recovery_succeeded: msg.recovery_succeeded,
      grasp_released: msg.grasp_released,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      error_code: msg.error_code,
      message: msg.message.to_string(),
      initial_position: msg.initial_position,
      final_position: msg.final_position,
      peak_position: msg.peak_position,
      final_state: msg.final_state.to_string(),
      requested_force: msg.requested_force,
      estimated_force: msg.estimated_force,
      button_triggered: msg.button_triggered,
      validation_performed: msg.validation_performed,
      operation_executed: msg.operation_executed,
      diagnostic_stage: msg.diagnostic_stage.to_string(),
      path_fraction: msg.path_fraction,
      required_fraction: msg.required_fraction,
      moveit_error_code: msg.moveit_error_code,
      policy_reason: msg.policy_reason.to_string(),
      failure_reason: msg.failure_reason.to_string(),
      physical_outcome_confirmed: msg.physical_outcome_confirmed,
      final_state_verified: msg.final_state_verified,
      transport_succeeded: msg.transport_succeeded,
      recovery_succeeded: msg.recovery_succeeded,
      grasp_released: msg.grasp_released,
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub phase: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub progress: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}

impl OperateCabinetControl_Feedback {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const WAITING_FOR_SYSTEM: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NAVIGATING: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DOCKING: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOVING_TO_READY: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APPROACHING: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GRASPING: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MANIPULATING: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VERIFYING: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RELEASING: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RETREATING: u8 = 9;

}


impl Default for OperateCabinetControl_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_Feedback {
  type RmwMsg = super::action::rmw::OperateCabinetControl_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        phase: msg.phase,
        progress: msg.progress,
        current_position: msg.current_position,
        target_position: msg.target_position,
        current_state: msg.current_state.as_str().into(),
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      phase: msg.phase,
      progress: msg.progress,
      current_position: msg.current_position,
      target_position: msg.target_position,
        current_state: msg.current_state.as_str().into(),
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      phase: msg.phase,
      progress: msg.progress,
      current_position: msg.current_position,
      target_position: msg.target_position,
      current_state: msg.current_state.to_string(),
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::OperateCabinetControl_Feedback,

}



impl Default for OperateCabinetControl_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_FeedbackMessage {
  type RmwMsg = super::action::rmw::OperateCabinetControl_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::OperateCabinetControl_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::OperateCabinetControl_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::OperateCabinetControl_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub button_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigate_to_staging_pose: bool,

}



impl Default for PressCabinetButton_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_Goal {
  type RmwMsg = super::action::rmw::PressCabinetButton_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        button_id: msg.button_id.as_str().into(),
        navigate_to_staging_pose: msg.navigate_to_staging_pose,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        button_id: msg.button_id.as_str().into(),
      navigate_to_staging_pose: msg.navigate_to_staging_pose,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      button_id: msg.button_id.to_string(),
      navigate_to_staging_pose: msg.navigate_to_staging_pose,
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error_code: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_travel: f64,

    /// Always empty on success.  On failure this contains the actionable cause.
    pub failure_reason: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub physical_outcome_confirmed: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub final_state_verified: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub transport_succeeded: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub recovery_succeeded: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub grasp_released: bool,

}

impl PressCabinetButton_Result {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SUCCESS: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INVALID_BUTTON: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NOT_READY: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NAVIGATION_FAILED: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PLANNING_FAILED: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXECUTION_FAILED: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PRESS_NOT_DETECTED: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RELEASE_NOT_DETECTED: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CANCELED: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INTERNAL_ERROR: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESOURCE_BUSY: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LEASE_LOST: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TOOLSET_MISMATCH: u8 = 12;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ADAPTER_NOT_VALIDATED: u8 = 13;

}


impl Default for PressCabinetButton_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_Result::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_Result {
  type RmwMsg = super::action::rmw::PressCabinetButton_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        error_code: msg.error_code,
        message: msg.message.as_str().into(),
        max_travel: msg.max_travel,
        failure_reason: msg.failure_reason.as_str().into(),
        physical_outcome_confirmed: msg.physical_outcome_confirmed,
        final_state_verified: msg.final_state_verified,
        transport_succeeded: msg.transport_succeeded,
        recovery_succeeded: msg.recovery_succeeded,
        grasp_released: msg.grasp_released,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      error_code: msg.error_code,
        message: msg.message.as_str().into(),
      max_travel: msg.max_travel,
        failure_reason: msg.failure_reason.as_str().into(),
      physical_outcome_confirmed: msg.physical_outcome_confirmed,
      final_state_verified: msg.final_state_verified,
      transport_succeeded: msg.transport_succeeded,
      recovery_succeeded: msg.recovery_succeeded,
      grasp_released: msg.grasp_released,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      error_code: msg.error_code,
      message: msg.message.to_string(),
      max_travel: msg.max_travel,
      failure_reason: msg.failure_reason.to_string(),
      physical_outcome_confirmed: msg.physical_outcome_confirmed,
      final_state_verified: msg.final_state_verified,
      transport_succeeded: msg.transport_succeeded,
      recovery_succeeded: msg.recovery_succeeded,
      grasp_released: msg.grasp_released,
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub phase: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub progress: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_travel: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}

impl PressCabinetButton_Feedback {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const WAITING_FOR_SYSTEM: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NAVIGATING: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOVING_TO_PREPRESS: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const APPROACHING: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PRESSING: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VERIFYING_PRESS: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RETRACTING: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VERIFYING_RELEASE: u8 = 7;

}


impl Default for PressCabinetButton_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_Feedback {
  type RmwMsg = super::action::rmw::PressCabinetButton_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        phase: msg.phase,
        progress: msg.progress,
        button_travel: msg.button_travel,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      phase: msg.phase,
      progress: msg.progress,
      button_travel: msg.button_travel,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      phase: msg.phase,
      progress: msg.progress,
      button_travel: msg.button_travel,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::PressCabinetButton_Feedback,

}



impl Default for PressCabinetButton_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_FeedbackMessage {
  type RmwMsg = super::action::rmw::PressCabinetButton_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::PressCabinetButton_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::PressCabinetButton_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::PressCabinetButton_Feedback::from_rmw_message(msg.feedback),
    }
  }
}






// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::OperateCabinetControl_Goal,

}



impl Default for OperateCabinetControl_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_SendGoal_Request {
  type RmwMsg = super::action::rmw::OperateCabinetControl_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::OperateCabinetControl_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::OperateCabinetControl_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::OperateCabinetControl_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for OperateCabinetControl_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_SendGoal_Response {
  type RmwMsg = super::action::rmw::OperateCabinetControl_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for OperateCabinetControl_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_GetResult_Request {
  type RmwMsg = super::action::rmw::OperateCabinetControl_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::OperateCabinetControl_Result,

}



impl Default for OperateCabinetControl_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::OperateCabinetControl_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_GetResult_Response {
  type RmwMsg = super::action::rmw::OperateCabinetControl_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::OperateCabinetControl_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::OperateCabinetControl_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::OperateCabinetControl_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::PressCabinetButton_Goal,

}



impl Default for PressCabinetButton_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_SendGoal_Request {
  type RmwMsg = super::action::rmw::PressCabinetButton_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::PressCabinetButton_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::PressCabinetButton_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::PressCabinetButton_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for PressCabinetButton_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_SendGoal_Response {
  type RmwMsg = super::action::rmw::PressCabinetButton_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for PressCabinetButton_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_GetResult_Request {
  type RmwMsg = super::action::rmw::PressCabinetButton_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::PressCabinetButton_Result,

}



impl Default for PressCabinetButton_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::PressCabinetButton_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_GetResult_Response {
  type RmwMsg = super::action::rmw::PressCabinetButton_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::PressCabinetButton_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::PressCabinetButton_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::PressCabinetButton_Result::from_rmw_message(msg.result),
    }
  }
}






#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct OperateCabinetControl_SendGoal;

impl rosidl_runtime_rs::Service for OperateCabinetControl_SendGoal {
    type Request = OperateCabinetControl_SendGoal_Request;
    type Response = OperateCabinetControl_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal() }
    }
}




#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct OperateCabinetControl_GetResult;

impl rosidl_runtime_rs::Service for OperateCabinetControl_GetResult {
    type Request = OperateCabinetControl_GetResult_Request;
    type Response = OperateCabinetControl_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__OperateCabinetControl_GetResult() }
    }
}




#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__PressCabinetButton_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct PressCabinetButton_SendGoal;

impl rosidl_runtime_rs::Service for PressCabinetButton_SendGoal {
    type Request = PressCabinetButton_SendGoal_Request;
    type Response = PressCabinetButton_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__PressCabinetButton_SendGoal() }
    }
}




#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__PressCabinetButton_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct PressCabinetButton_GetResult;

impl rosidl_runtime_rs::Service for PressCabinetButton_GetResult {
    type Request = PressCabinetButton_GetResult_Request;
    type Response = PressCabinetButton_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__action__PressCabinetButton_GetResult() }
    }
}






#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__xczs_inspection_robot_control__action__OperateCabinetControl() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__action__OperateCabinetControl
#[allow(missing_docs, non_camel_case_types)]
pub struct OperateCabinetControl;

impl rosidl_runtime_rs::Action for OperateCabinetControl {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = OperateCabinetControl_Goal;

  /// The result message defined in the action definition.
  type Result = OperateCabinetControl_Result;

  /// The feedback message defined in the action definition.
  type Feedback = OperateCabinetControl_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::OperateCabinetControl_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::OperateCabinetControl_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::OperateCabinetControl_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__xczs_inspection_robot_control__action__OperateCabinetControl() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::OperateCabinetControl_Goal,
  ) -> super::action::rmw::OperateCabinetControl_SendGoal_Request {
   super::action::rmw::OperateCabinetControl_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::OperateCabinetControl_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::OperateCabinetControl_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::OperateCabinetControl_SendGoal_Response {
   super::action::rmw::OperateCabinetControl_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::OperateCabinetControl_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::OperateCabinetControl_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::OperateCabinetControl_Feedback,
  ) -> super::action::rmw::OperateCabinetControl_FeedbackMessage {
    let mut message = super::action::rmw::OperateCabinetControl_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::OperateCabinetControl_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::OperateCabinetControl_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::OperateCabinetControl_GetResult_Request {
   super::action::rmw::OperateCabinetControl_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::OperateCabinetControl_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::OperateCabinetControl_Result,
  ) -> super::action::rmw::OperateCabinetControl_GetResult_Response {
   super::action::rmw::OperateCabinetControl_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::OperateCabinetControl_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::OperateCabinetControl_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__xczs_inspection_robot_control__action__PressCabinetButton() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__action__PressCabinetButton
#[allow(missing_docs, non_camel_case_types)]
pub struct PressCabinetButton;

impl rosidl_runtime_rs::Action for PressCabinetButton {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = PressCabinetButton_Goal;

  /// The result message defined in the action definition.
  type Result = PressCabinetButton_Result;

  /// The feedback message defined in the action definition.
  type Feedback = PressCabinetButton_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::PressCabinetButton_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::PressCabinetButton_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::PressCabinetButton_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__xczs_inspection_robot_control__action__PressCabinetButton() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::PressCabinetButton_Goal,
  ) -> super::action::rmw::PressCabinetButton_SendGoal_Request {
   super::action::rmw::PressCabinetButton_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::PressCabinetButton_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::PressCabinetButton_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::PressCabinetButton_SendGoal_Response {
   super::action::rmw::PressCabinetButton_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::PressCabinetButton_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::PressCabinetButton_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::PressCabinetButton_Feedback,
  ) -> super::action::rmw::PressCabinetButton_FeedbackMessage {
    let mut message = super::action::rmw::PressCabinetButton_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::PressCabinetButton_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::PressCabinetButton_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::PressCabinetButton_GetResult_Request {
   super::action::rmw::PressCabinetButton_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::PressCabinetButton_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::PressCabinetButton_Result,
  ) -> super::action::rmw::PressCabinetButton_GetResult_Response {
   super::action::rmw::PressCabinetButton_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::PressCabinetButton_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::PressCabinetButton_Result,
  ) {
    (response.status, response.result)
  }
}


