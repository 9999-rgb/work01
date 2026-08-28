#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageOperationLease_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub owner_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lease_id: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ManageOperationLease_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ManageOperationLease_Request {
  type RmwMsg = super::srv::rmw::ManageOperationLease_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: msg.command,
        owner_id: msg.owner_id.as_str().into(),
        lease_id: msg.lease_id.as_str().into(),
        requested_duration: msg.requested_duration,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      command: msg.command,
        owner_id: msg.owner_id.as_str().into(),
        lease_id: msg.lease_id.as_str().into(),
      requested_duration: msg.requested_duration,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      command: msg.command,
      owner_id: msg.owner_id.to_string(),
      lease_id: msg.lease_id.to_string(),
      requested_duration: msg.requested_duration,
    }
  }
}


// Corresponds to xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lease_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub owner_id: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ManageOperationLease_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ManageOperationLease_Response {
  type RmwMsg = super::srv::rmw::ManageOperationLease_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        status_code: msg.status_code,
        message: msg.message.as_str().into(),
        lease_id: msg.lease_id.as_str().into(),
        owner_id: msg.owner_id.as_str().into(),
        remaining_duration: msg.remaining_duration,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      status_code: msg.status_code,
        message: msg.message.as_str().into(),
        lease_id: msg.lease_id.as_str().into(),
        owner_id: msg.owner_id.as_str().into(),
      remaining_duration: msg.remaining_duration,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      status_code: msg.status_code,
      message: msg.message.to_string(),
      lease_id: msg.lease_id.to_string(),
      owner_id: msg.owner_id.to_string(),
      remaining_duration: msg.remaining_duration,
    }
  }
}


// Corresponds to xczs_inspection_robot_interfaces__srv__SetCabinetGrasp_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCabinetGrasp_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operation_lease_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_model: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_link: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_grasp_point: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_base_link: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub attach: bool,

}



impl Default for SetCabinetGrasp_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCabinetGrasp_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetCabinetGrasp_Request {
  type RmwMsg = super::srv::rmw::SetCabinetGrasp_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        control_id: msg.control_id.as_str().into(),
        operation_lease_id: msg.operation_lease_id.as_str().into(),
        robot_model: msg.robot_model.as_str().into(),
        robot_link: msg.robot_link.as_str().into(),
        robot_grasp_point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.robot_grasp_point)).into_owned(),
        robot_base_link: msg.robot_base_link.as_str().into(),
        attach: msg.attach,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        control_id: msg.control_id.as_str().into(),
        operation_lease_id: msg.operation_lease_id.as_str().into(),
        robot_model: msg.robot_model.as_str().into(),
        robot_link: msg.robot_link.as_str().into(),
        robot_grasp_point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.robot_grasp_point)).into_owned(),
        robot_base_link: msg.robot_base_link.as_str().into(),
      attach: msg.attach,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      control_id: msg.control_id.to_string(),
      operation_lease_id: msg.operation_lease_id.to_string(),
      robot_model: msg.robot_model.to_string(),
      robot_link: msg.robot_link.to_string(),
      robot_grasp_point: geometry_msgs::msg::Point::from_rmw_message(msg.robot_grasp_point),
      robot_base_link: msg.robot_base_link.to_string(),
      attach: msg.attach,
    }
  }
}


// Corresponds to xczs_inspection_robot_interfaces__srv__SetCabinetGrasp_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCabinetGrasp_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f64,

}



impl Default for SetCabinetGrasp_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCabinetGrasp_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetCabinetGrasp_Response {
  type RmwMsg = super::srv::rmw::SetCabinetGrasp_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
        distance: msg.distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      distance: msg.distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
      distance: msg.distance,
    }
  }
}


// Corresponds to xczs_inspection_robot_interfaces__srv__SwitchToolset_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SwitchToolset_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub toolset: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub expected_generation: u64,

}



impl Default for SwitchToolset_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SwitchToolset_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SwitchToolset_Request {
  type RmwMsg = super::srv::rmw::SwitchToolset_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        toolset: msg.toolset.as_str().into(),
        expected_generation: msg.expected_generation,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        toolset: msg.toolset.as_str().into(),
      expected_generation: msg.expected_generation,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      toolset: msg.toolset.to_string(),
      expected_generation: msg.expected_generation,
    }
  }
}


// Corresponds to xczs_inspection_robot_interfaces__srv__SwitchToolset_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SwitchToolset_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub active_toolset: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_toolset: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub generation: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SwitchToolset_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SwitchToolset_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SwitchToolset_Response {
  type RmwMsg = super::srv::rmw::SwitchToolset_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        state: msg.state.as_str().into(),
        active_toolset: msg.active_toolset.as_str().into(),
        target_toolset: msg.target_toolset.as_str().into(),
        generation: msg.generation,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        state: msg.state.as_str().into(),
        active_toolset: msg.active_toolset.as_str().into(),
        target_toolset: msg.target_toolset.as_str().into(),
      generation: msg.generation,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      state: msg.state.to_string(),
      active_toolset: msg.active_toolset.to_string(),
      target_toolset: msg.target_toolset.to_string(),
      generation: msg.generation,
      message: msg.message.to_string(),
    }
  }
}






#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__srv__ManageOperationLease() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_interfaces__srv__ManageOperationLease
#[allow(missing_docs, non_camel_case_types)]
pub struct ManageOperationLease;

impl rosidl_runtime_rs::Service for ManageOperationLease {
    type Request = ManageOperationLease_Request;
    type Response = ManageOperationLease_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__srv__ManageOperationLease() }
    }
}




#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__srv__SetCabinetGrasp() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_interfaces__srv__SetCabinetGrasp
#[allow(missing_docs, non_camel_case_types)]
pub struct SetCabinetGrasp;

impl rosidl_runtime_rs::Service for SetCabinetGrasp {
    type Request = SetCabinetGrasp_Request;
    type Response = SetCabinetGrasp_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__srv__SetCabinetGrasp() }
    }
}




#[link(name = "xczs_inspection_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__srv__SwitchToolset() -> *const std::ffi::c_void;
}

// Corresponds to xczs_inspection_robot_interfaces__srv__SwitchToolset
#[allow(missing_docs, non_camel_case_types)]
pub struct SwitchToolset;

impl rosidl_runtime_rs::Service for SwitchToolset {
    type Request = SwitchToolset_Request;
    type Response = SwitchToolset_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__xczs_inspection_robot_interfaces__srv__SwitchToolset() }
    }
}


