#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__msg__CabinetControl() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__msg__CabinetControl__init(msg: *mut CabinetControl) -> bool;
    fn xczs_inspection_robot_control__msg__CabinetControl__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CabinetControl>, size: usize) -> bool;
    fn xczs_inspection_robot_control__msg__CabinetControl__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CabinetControl>);
    fn xczs_inspection_robot_control__msg__CabinetControl__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CabinetControl>, out_seq: *mut rosidl_runtime_rs::Sequence<CabinetControl>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__msg__CabinetControl
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CabinetControl {

    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub display_name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_state_topic: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pressed_topic: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_topic: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub supported_commands: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub unit: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub min_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_ids: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_labels: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_positions: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub requires_grasp: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operable: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub unavailable_reason: rosidl_runtime_rs::String,

    /// The end-effector set required by this control (A or B).  This is exposed
    /// separately from ``operable`` so clients can provide an actionable error
    /// before attempting navigation when the other set is mounted.
    pub required_toolset: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub toolset_compatible: bool,

    /// True when this control is in the robot adapter's physically validated
    /// allowlist.  A false value means planning-only validation is allowed, but no
    /// physical cabinet command may be sent.
    pub adapter_validated: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub default_force: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub min_trigger_force: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_force: f64,

}

impl CabinetControl {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TYPE_BUTTON: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TYPE_KNOB: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TYPE_SWITCH: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TYPE_DOOR: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SUPPORT_PRESS: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SUPPORT_SET_STATE: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SUPPORT_SET_POSITION: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SUPPORT_TOGGLE: u8 = 8;

}


impl Default for CabinetControl {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__msg__CabinetControl__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__msg__CabinetControl__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CabinetControl {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControl__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControl__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControl__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CabinetControl {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CabinetControl where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/msg/CabinetControl";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__msg__CabinetControl() }
  }
}


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__msg__CabinetControlCatalog() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__msg__CabinetControlCatalog__init(msg: *mut CabinetControlCatalog) -> bool;
    fn xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CabinetControlCatalog>, size: usize) -> bool;
    fn xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CabinetControlCatalog>);
    fn xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CabinetControlCatalog>, out_seq: *mut rosidl_runtime_rs::Sequence<CabinetControlCatalog>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__msg__CabinetControlCatalog
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CabinetControlCatalog {

    // This member is not documented.
    #[allow(missing_docs)]
    pub controls: rosidl_runtime_rs::Sequence<super::super::msg::rmw::CabinetControl>,

}



impl Default for CabinetControlCatalog {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__msg__CabinetControlCatalog__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__msg__CabinetControlCatalog__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CabinetControlCatalog {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CabinetControlCatalog {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CabinetControlCatalog where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/msg/CabinetControlCatalog";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__msg__CabinetControlCatalog() }
  }
}


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__msg__CabinetControlState() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__msg__CabinetControlState__init(msg: *mut CabinetControlState) -> bool;
    fn xczs_inspection_robot_control__msg__CabinetControlState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CabinetControlState>, size: usize) -> bool;
    fn xczs_inspection_robot_control__msg__CabinetControlState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CabinetControlState>);
    fn xczs_inspection_robot_control__msg__CabinetControlState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CabinetControlState>, out_seq: *mut rosidl_runtime_rs::Sequence<CabinetControlState>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__msg__CabinetControlState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CabinetControlState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub effort: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub normalized_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub activated: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub in_motion: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub transition_sequence: u64,

}



impl Default for CabinetControlState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__msg__CabinetControlState__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__msg__CabinetControlState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CabinetControlState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControlState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControlState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__msg__CabinetControlState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CabinetControlState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CabinetControlState where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/msg/CabinetControlState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__msg__CabinetControlState() }
  }
}


