#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__ManageOperationLease_Request() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Request__init(msg: *mut ManageOperationLease_Request) -> bool;
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManageOperationLease_Request>, size: usize) -> bool;
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManageOperationLease_Request>);
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManageOperationLease_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ManageOperationLease_Request>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__srv__ManageOperationLease_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageOperationLease_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub owner_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lease_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub requested_duration: f64,

}

impl ManageOperationLease_Request {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACQUIRE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RENEW: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RELEASE: u8 = 2;

}


impl Default for ManageOperationLease_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__srv__ManageOperationLease_Request__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__srv__ManageOperationLease_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManageOperationLease_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManageOperationLease_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManageOperationLease_Request where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/srv/ManageOperationLease_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__ManageOperationLease_Request() }
  }
}


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__ManageOperationLease_Response() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Response__init(msg: *mut ManageOperationLease_Response) -> bool;
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManageOperationLease_Response>, size: usize) -> bool;
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManageOperationLease_Response>);
    fn xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManageOperationLease_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ManageOperationLease_Response>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__srv__ManageOperationLease_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageOperationLease_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status_code: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lease_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub owner_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub remaining_duration: f64,

}

impl ManageOperationLease_Response {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GRANTED: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESOURCE_BUSY: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INVALID_REQUEST: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NOT_OWNER: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RELEASED: u8 = 4;

}


impl Default for ManageOperationLease_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__srv__ManageOperationLease_Response__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__srv__ManageOperationLease_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManageOperationLease_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManageOperationLease_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManageOperationLease_Response where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/srv/ManageOperationLease_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__ManageOperationLease_Response() }
  }
}


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SetCabinetGrasp_Request() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__init(msg: *mut SetCabinetGrasp_Request) -> bool;
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCabinetGrasp_Request>, size: usize) -> bool;
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCabinetGrasp_Request>);
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCabinetGrasp_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCabinetGrasp_Request>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__srv__SetCabinetGrasp_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCabinetGrasp_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operation_lease_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_model: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_link: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_grasp_point: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_base_link: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub attach: bool,

}



impl Default for SetCabinetGrasp_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCabinetGrasp_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCabinetGrasp_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCabinetGrasp_Request where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/srv/SetCabinetGrasp_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SetCabinetGrasp_Request() }
  }
}


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SetCabinetGrasp_Response() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__init(msg: *mut SetCabinetGrasp_Response) -> bool;
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCabinetGrasp_Response>, size: usize) -> bool;
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCabinetGrasp_Response>);
    fn xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCabinetGrasp_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCabinetGrasp_Response>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__srv__SetCabinetGrasp_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCabinetGrasp_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f64,

}



impl Default for SetCabinetGrasp_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCabinetGrasp_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCabinetGrasp_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCabinetGrasp_Response where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/srv/SetCabinetGrasp_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SetCabinetGrasp_Response() }
  }
}


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SwitchToolset_Request() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__srv__SwitchToolset_Request__init(msg: *mut SwitchToolset_Request) -> bool;
    fn xczs_inspection_robot_control__srv__SwitchToolset_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SwitchToolset_Request>, size: usize) -> bool;
    fn xczs_inspection_robot_control__srv__SwitchToolset_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SwitchToolset_Request>);
    fn xczs_inspection_robot_control__srv__SwitchToolset_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SwitchToolset_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SwitchToolset_Request>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__srv__SwitchToolset_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SwitchToolset_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub toolset: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub expected_generation: u64,

}



impl Default for SwitchToolset_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__srv__SwitchToolset_Request__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__srv__SwitchToolset_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SwitchToolset_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SwitchToolset_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SwitchToolset_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SwitchToolset_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SwitchToolset_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SwitchToolset_Request where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/srv/SwitchToolset_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SwitchToolset_Request() }
  }
}


#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SwitchToolset_Response() -> *const std::ffi::c_void;
}

#[link(name = "xczs_inspection_robot_control__rosidl_generator_c")]
extern "C" {
    fn xczs_inspection_robot_control__srv__SwitchToolset_Response__init(msg: *mut SwitchToolset_Response) -> bool;
    fn xczs_inspection_robot_control__srv__SwitchToolset_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SwitchToolset_Response>, size: usize) -> bool;
    fn xczs_inspection_robot_control__srv__SwitchToolset_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SwitchToolset_Response>);
    fn xczs_inspection_robot_control__srv__SwitchToolset_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SwitchToolset_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SwitchToolset_Response>) -> bool;
}

// Corresponds to xczs_inspection_robot_control__srv__SwitchToolset_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SwitchToolset_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub active_toolset: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_toolset: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub generation: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SwitchToolset_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !xczs_inspection_robot_control__srv__SwitchToolset_Response__init(&mut msg as *mut _) {
        panic!("Call to xczs_inspection_robot_control__srv__SwitchToolset_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SwitchToolset_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SwitchToolset_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SwitchToolset_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { xczs_inspection_robot_control__srv__SwitchToolset_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SwitchToolset_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SwitchToolset_Response where Self: Sized {
  const TYPE_NAME: &'static str = "xczs_inspection_robot_control/srv/SwitchToolset_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__xczs_inspection_robot_control__srv__SwitchToolset_Response() }
  }
}






#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__srv__ManageOperationLease() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__srv__ManageOperationLease
#[allow(missing_docs, non_camel_case_types)]
pub struct ManageOperationLease;

impl rosidl_runtime_rs::Service for ManageOperationLease {
    type Request = ManageOperationLease_Request;
    type Response = ManageOperationLease_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__srv__ManageOperationLease() }
    }
}




#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__srv__SetCabinetGrasp() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__srv__SetCabinetGrasp
#[allow(missing_docs, non_camel_case_types)]
pub struct SetCabinetGrasp;

impl rosidl_runtime_rs::Service for SetCabinetGrasp {
    type Request = SetCabinetGrasp_Request;
    type Response = SetCabinetGrasp_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__srv__SetCabinetGrasp() }
    }
}




#[link(name = "xczs_inspection_robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__srv__SwitchToolset() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_control__srv__SwitchToolset
#[allow(missing_docs, non_camel_case_types)]
pub struct SwitchToolset;

impl rosidl_runtime_rs::Service for SwitchToolset {
    type Request = SwitchToolset_Request;
    type Response = SwitchToolset_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_control__srv__SwitchToolset() }
    }
}


