// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_interfaces:action/OperateCabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__OPERATE_CABINET_CONTROL__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__OPERATE_CABINET_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'COMMAND_PRESS'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__COMMAND_PRESS = 0
};

/// Constant 'COMMAND_SET_STATE'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__COMMAND_SET_STATE = 1
};

/// Constant 'COMMAND_SET_POSITION'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__COMMAND_SET_POSITION = 2
};

/// Constant 'COMMAND_TOGGLE'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__COMMAND_TOGGLE = 3
};

// Include directives for member types
// Member 'control_id'
// Member 'target_state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal
{
  rosidl_runtime_c__String control_id;
  uint8_t command;
  rosidl_runtime_c__String target_state;
  double target_position;
  bool use_target_position;
  double force;
  bool navigate_to_staging_pose;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__Sequence;


// Constants defined in the message

/// Constant 'SUCCESS'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__SUCCESS = 0
};

/// Constant 'INVALID_CONTROL'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__INVALID_CONTROL = 1
};

/// Constant 'UNSUPPORTED_COMMAND'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__UNSUPPORTED_COMMAND = 2
};

/// Constant 'NOT_READY'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__NOT_READY = 3
};

/// Constant 'NAVIGATION_FAILED'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__NAVIGATION_FAILED = 4
};

/// Constant 'PLANNING_FAILED'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__PLANNING_FAILED = 5
};

/// Constant 'EXECUTION_FAILED'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__EXECUTION_FAILED = 6
};

/// Constant 'GRASP_FAILED'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__GRASP_FAILED = 7
};

/// Constant 'TARGET_NOT_REACHED'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__TARGET_NOT_REACHED = 8
};

/// Constant 'RELEASE_FAILED'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__RELEASE_FAILED = 9
};

/// Constant 'CANCELED'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__CANCELED = 10
};

/// Constant 'INTERNAL_ERROR'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__INTERNAL_ERROR = 11
};

/// Constant 'INVALID_FORCE'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__INVALID_FORCE = 12
};

/// Constant 'INSUFFICIENT_FORCE'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__INSUFFICIENT_FORCE = 13
};

/// Constant 'UNREACHABLE'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__UNREACHABLE = 14
};

/// Constant 'CONTACT_DETECTION_TIMEOUT'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__CONTACT_DETECTION_TIMEOUT = 15
};

/// Constant 'RESOURCE_BUSY'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__RESOURCE_BUSY = 16
};

/// Constant 'LEASE_LOST'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__LEASE_LOST = 17
};

/// Constant 'TOOLSET_MISMATCH'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__TOOLSET_MISMATCH = 18
};

// Include directives for member types
// Member 'message'
// Member 'final_state'
// Member 'diagnostic_stage'
// Member 'policy_reason'
// Member 'failure_reason'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result
{
  bool success;
  uint8_t error_code;
  rosidl_runtime_c__String message;
  double initial_position;
  double final_position;
  /// Maximum absolute joint position observed during this operation.  This keeps
  /// negative knob travel observable while remaining identical to button travel.
  double peak_position;
  rosidl_runtime_c__String final_state;
  double requested_force;
  double estimated_force;
  bool button_triggered;
  bool validation_performed;
  bool operation_executed;
  rosidl_runtime_c__String diagnostic_stage;
  double path_fraction;
  double required_fraction;
  int32_t moveit_error_code;
  rosidl_runtime_c__String policy_reason;
  /// Always empty on success.  On failure this is the actionable cause and is
  /// kept separate from the short, user-facing message for machine clients.
  rosidl_runtime_c__String failure_reason;
  /// Terminal evidence flags.  They are intentionally conservative: each is
  /// true only after the corresponding physical side effect or recovery phase
  /// has been verified.
  bool physical_outcome_confirmed;
  bool final_state_verified;
  bool transport_succeeded;
  bool recovery_succeeded;
  bool grasp_released;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__Sequence;


// Constants defined in the message

/// Constant 'WAITING_FOR_SYSTEM'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__WAITING_FOR_SYSTEM = 0
};

/// Constant 'NAVIGATING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__NAVIGATING = 1
};

/// Constant 'DOCKING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__DOCKING = 2
};

/// Constant 'MOVING_TO_READY'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__MOVING_TO_READY = 3
};

/// Constant 'APPROACHING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__APPROACHING = 4
};

/// Constant 'GRASPING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__GRASPING = 5
};

/// Constant 'MANIPULATING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__MANIPULATING = 6
};

/// Constant 'VERIFYING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__VERIFYING = 7
};

/// Constant 'RELEASING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__RELEASING = 8
};

/// Constant 'RETREATING'.
enum
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__RETREATING = 9
};

// Include directives for member types
// Member 'current_state'
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback
{
  uint8_t phase;
  float progress;
  double current_position;
  double target_position;
  rosidl_runtime_c__String current_state;
  rosidl_runtime_c__String message;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal goal;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response
{
  int8_t status;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result result;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"

/// Struct defined in action/OperateCabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback feedback;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage;

// Struct for a sequence of xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage.
typedef struct xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__OPERATE_CABINET_CONTROL__STRUCT_H_
