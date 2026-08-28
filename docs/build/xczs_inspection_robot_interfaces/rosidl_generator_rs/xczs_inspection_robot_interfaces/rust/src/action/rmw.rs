
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__init(msg: *mut OperateCabinetControl_Goal) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Goal>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Goal>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Goal>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_state: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__init(msg: *mut OperateCabinetControl_Result) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Result>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Result>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Result>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub message: rosidl_runtime_rs::String,


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
    pub final_state: rosidl_runtime_rs::String,


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
    pub diagnostic_stage: rosidl_runtime_rs::String,


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
    pub policy_reason: rosidl_runtime_rs::String,

    /// Always empty on success.  On failure this is the actionable cause and is
    /// kept separate from the short, user-facing message for machine clients.
    pub failure_reason: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_Result where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__init(msg: *mut OperateCabinetControl_Feedback) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Feedback>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Feedback>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_Feedback>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub current_state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__init(msg: *mut OperateCabinetControl_FeedbackMessage) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_FeedbackMessage>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_FeedbackMessage>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_FeedbackMessage>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::OperateCabinetControl_Feedback,

}



impl Default for OperateCabinetControl_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__init(msg: *mut PressCabinetButton_Goal) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Goal>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Goal>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Goal>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub button_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigate_to_staging_pose: bool,

}



impl Default for PressCabinetButton_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_Goal() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_Result() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__init(msg: *mut PressCabinetButton_Result) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Result>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Result>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Result>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_travel: f64,

    /// Always empty on success.  On failure this contains the actionable cause.
    pub failure_reason: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_Result where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_Result() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__init(msg: *mut PressCabinetButton_Feedback) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Feedback>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Feedback>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_Feedback>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub message: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_Feedback() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__init(msg: *mut PressCabinetButton_FeedbackMessage) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_FeedbackMessage>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_FeedbackMessage>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_FeedbackMessage>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::PressCabinetButton_Feedback,

}



impl Default for PressCabinetButton_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_FeedbackMessage() }
  }
}




#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__init(msg: *mut OperateCabinetControl_SendGoal_Request) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Request>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Request>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Request>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::OperateCabinetControl_Goal,

}



impl Default for OperateCabinetControl_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__init(msg: *mut OperateCabinetControl_SendGoal_Response) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Response>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Response>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_SendGoal_Response>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for OperateCabinetControl_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__init(msg: *mut OperateCabinetControl_GetResult_Request) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Request>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Request>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Request>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for OperateCabinetControl_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__init(msg: *mut OperateCabinetControl_GetResult_Response) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Response>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Response>);
    fn xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<OperateCabinetControl_GetResult_Response>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OperateCabinetControl_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::OperateCabinetControl_Result,

}



impl Default for OperateCabinetControl_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OperateCabinetControl_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OperateCabinetControl_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OperateCabinetControl_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/OperateCabinetControl_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__init(msg: *mut PressCabinetButton_SendGoal_Request) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Request>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Request>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Request>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::PressCabinetButton_Goal,

}



impl Default for PressCabinetButton_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Request() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__init(msg: *mut PressCabinetButton_SendGoal_Response) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Response>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Response>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_SendGoal_Response>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for PressCabinetButton_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal_Response() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__init(msg: *mut PressCabinetButton_GetResult_Request) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Request>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Request>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Request>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for PressCabinetButton_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Request() }
  }
}


#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__init(msg: *mut PressCabinetButton_GetResult_Response) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Response>, size: usize) -> bool;
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Response>);
    fn xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PressCabinetButton_GetResult_Response>) -> bool;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PressCabinetButton_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::PressCabinetButton_Result,

}



impl Default for PressCabinetButton_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PressCabinetButton_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PressCabinetButton_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PressCabinetButton_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_interfaces/action/PressCabinetButton_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult_Response() }
  }
}






#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct OperateCabinetControl_SendGoal;

impl rosidl_runtime_rs::Service for OperateCabinetControl_SendGoal {
    type Request = OperateCabinetControl_SendGoal_Request;
    type Response = OperateCabinetControl_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal() }
    }
}




#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct OperateCabinetControl_GetResult;

impl rosidl_runtime_rs::Service for OperateCabinetControl_GetResult {
    type Request = OperateCabinetControl_GetResult_Request;
    type Response = OperateCabinetControl_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult() }
    }
}




#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct PressCabinetButton_SendGoal;

impl rosidl_runtime_rs::Service for PressCabinetButton_SendGoal {
    type Request = PressCabinetButton_SendGoal_Request;
    type Response = PressCabinetButton_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_SendGoal() }
    }
}




#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct PressCabinetButton_GetResult;

impl rosidl_runtime_rs::Service for PressCabinetButton_GetResult {
    type Request = PressCabinetButton_GetResult_Request;
    type Response = PressCabinetButton_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__action__PressCabinetButton_GetResult() }
    }
}


