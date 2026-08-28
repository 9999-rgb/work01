#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to xczs_inspection_robot_interfaces__msg__CabinetControl

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CabinetControl {

    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub display_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_state_topic: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pressed_topic: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_topic: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub supported_commands: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub unit: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub min_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_position: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_ids: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_labels: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state_positions: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub requires_grasp: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operable: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub unavailable_reason: std::string::String,

    /// The end-effector set required by this control (A or B).  This is exposed
    /// separately from ``operable`` so clients can provide an actionable error
    /// before attempting navigation when the other set is mounted.
    pub required_toolset: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CabinetControl::default())
  }
}

impl rosidl_runtime_rs::Message for CabinetControl {
  type RmwMsg = super::msg::rmw::CabinetControl;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        control_id: msg.control_id.as_str().into(),
        display_name: msg.display_name.as_str().into(),
        control_type: msg.control_type,
        joint_name: msg.joint_name.as_str().into(),
        joint_state_topic: msg.joint_state_topic.as_str().into(),
        pressed_topic: msg.pressed_topic.as_str().into(),
        state_topic: msg.state_topic.as_str().into(),
        supported_commands: msg.supported_commands,
        unit: msg.unit.as_str().into(),
        min_position: msg.min_position,
        max_position: msg.max_position,
        state_ids: msg.state_ids
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        state_labels: msg.state_labels
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        state_positions: msg.state_positions.into(),
        requires_grasp: msg.requires_grasp,
        operable: msg.operable,
        unavailable_reason: msg.unavailable_reason.as_str().into(),
        required_toolset: msg.required_toolset.as_str().into(),
        toolset_compatible: msg.toolset_compatible,
        adapter_validated: msg.adapter_validated,
        default_force: msg.default_force,
        min_trigger_force: msg.min_trigger_force,
        max_force: msg.max_force,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        control_id: msg.control_id.as_str().into(),
        display_name: msg.display_name.as_str().into(),
      control_type: msg.control_type,
        joint_name: msg.joint_name.as_str().into(),
        joint_state_topic: msg.joint_state_topic.as_str().into(),
        pressed_topic: msg.pressed_topic.as_str().into(),
        state_topic: msg.state_topic.as_str().into(),
      supported_commands: msg.supported_commands,
        unit: msg.unit.as_str().into(),
      min_position: msg.min_position,
      max_position: msg.max_position,
        state_ids: msg.state_ids
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        state_labels: msg.state_labels
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        state_positions: msg.state_positions.as_slice().into(),
      requires_grasp: msg.requires_grasp,
      operable: msg.operable,
        unavailable_reason: msg.unavailable_reason.as_str().into(),
        required_toolset: msg.required_toolset.as_str().into(),
      toolset_compatible: msg.toolset_compatible,
      adapter_validated: msg.adapter_validated,
      default_force: msg.default_force,
      min_trigger_force: msg.min_trigger_force,
      max_force: msg.max_force,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      control_id: msg.control_id.to_string(),
      display_name: msg.display_name.to_string(),
      control_type: msg.control_type,
      joint_name: msg.joint_name.to_string(),
      joint_state_topic: msg.joint_state_topic.to_string(),
      pressed_topic: msg.pressed_topic.to_string(),
      state_topic: msg.state_topic.to_string(),
      supported_commands: msg.supported_commands,
      unit: msg.unit.to_string(),
      min_position: msg.min_position,
      max_position: msg.max_position,
      state_ids: msg.state_ids
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      state_labels: msg.state_labels
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      state_positions: msg.state_positions
          .into_iter()
          .collect(),
      requires_grasp: msg.requires_grasp,
      operable: msg.operable,
      unavailable_reason: msg.unavailable_reason.to_string(),
      required_toolset: msg.required_toolset.to_string(),
      toolset_compatible: msg.toolset_compatible,
      adapter_validated: msg.adapter_validated,
      default_force: msg.default_force,
      min_trigger_force: msg.min_trigger_force,
      max_force: msg.max_force,
    }
  }
}


// Corresponds to xczs_inspection_robot_interfaces__msg__CabinetControlCatalog

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CabinetControlCatalog {

    // This member is not documented.
    #[allow(missing_docs)]
    pub controls: Vec<super::msg::CabinetControl>,

}



impl Default for CabinetControlCatalog {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CabinetControlCatalog::default())
  }
}

impl rosidl_runtime_rs::Message for CabinetControlCatalog {
  type RmwMsg = super::msg::rmw::CabinetControlCatalog;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        controls: msg.controls
          .into_iter()
          .map(|elem| super::msg::CabinetControl::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        controls: msg.controls
          .iter()
          .map(|elem| super::msg::CabinetControl::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      controls: msg.controls
          .into_iter()
          .map(super::msg::CabinetControl::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to xczs_inspection_robot_interfaces__msg__CabinetControlState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CabinetControlState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_id: std::string::String,


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
    pub state_id: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CabinetControlState::default())
  }
}

impl rosidl_runtime_rs::Message for CabinetControlState {
  type RmwMsg = super::msg::rmw::CabinetControlState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        control_id: msg.control_id.as_str().into(),
        control_type: msg.control_type,
        valid: msg.valid,
        position: msg.position,
        velocity: msg.velocity,
        effort: msg.effort,
        normalized_position: msg.normalized_position,
        state_id: msg.state_id.as_str().into(),
        activated: msg.activated,
        in_motion: msg.in_motion,
        transition_sequence: msg.transition_sequence,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        control_id: msg.control_id.as_str().into(),
      control_type: msg.control_type,
      valid: msg.valid,
      position: msg.position,
      velocity: msg.velocity,
      effort: msg.effort,
      normalized_position: msg.normalized_position,
        state_id: msg.state_id.as_str().into(),
      activated: msg.activated,
      in_motion: msg.in_motion,
      transition_sequence: msg.transition_sequence,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      control_id: msg.control_id.to_string(),
      control_type: msg.control_type,
      valid: msg.valid,
      position: msg.position,
      velocity: msg.velocity,
      effort: msg.effort,
      normalized_position: msg.normalized_position,
      state_id: msg.state_id.to_string(),
      activated: msg.activated,
      in_motion: msg.in_motion,
      transition_sequence: msg.transition_sequence,
    }
  }
}


