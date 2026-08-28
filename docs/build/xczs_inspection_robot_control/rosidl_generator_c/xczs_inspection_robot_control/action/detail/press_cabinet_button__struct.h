// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_control:action/PressCabinetButton.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__PRESS_CABINET_BUTTON__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__PRESS_CABINET_BUTTON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'button_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_Goal
{
  rosidl_runtime_c__String button_id;
  bool navigate_to_staging_pose;
} xczs_inspection_robot_control__action__PressCabinetButton_Goal;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_Goal.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_Goal__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_Goal__Sequence;


// Constants defined in the message

/// Constant 'SUCCESS'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__SUCCESS = 0
};

/// Constant 'INVALID_BUTTON'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__INVALID_BUTTON = 1
};

/// Constant 'NOT_READY'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__NOT_READY = 2
};

/// Constant 'NAVIGATION_FAILED'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__NAVIGATION_FAILED = 3
};

/// Constant 'PLANNING_FAILED'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__PLANNING_FAILED = 4
};

/// Constant 'EXECUTION_FAILED'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__EXECUTION_FAILED = 5
};

/// Constant 'PRESS_NOT_DETECTED'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__PRESS_NOT_DETECTED = 6
};

/// Constant 'RELEASE_NOT_DETECTED'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__RELEASE_NOT_DETECTED = 7
};

/// Constant 'CANCELED'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__CANCELED = 8
};

/// Constant 'INTERNAL_ERROR'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__INTERNAL_ERROR = 9
};

/// Constant 'RESOURCE_BUSY'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__RESOURCE_BUSY = 10
};

/// Constant 'LEASE_LOST'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__LEASE_LOST = 11
};

/// Constant 'TOOLSET_MISMATCH'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__TOOLSET_MISMATCH = 12
};

/// Constant 'ADAPTER_NOT_VALIDATED'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result__ADAPTER_NOT_VALIDATED = 13
};

// Include directives for member types
// Member 'message'
// Member 'failure_reason'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_Result
{
  bool success;
  uint8_t error_code;
  rosidl_runtime_c__String message;
  double max_travel;
  /// Always empty on success.  On failure this contains the actionable cause.
  rosidl_runtime_c__String failure_reason;
  bool physical_outcome_confirmed;
  bool final_state_verified;
  bool transport_succeeded;
  bool recovery_succeeded;
  bool grasp_released;
} xczs_inspection_robot_control__action__PressCabinetButton_Result;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_Result.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_Result__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_Result__Sequence;


// Constants defined in the message

/// Constant 'WAITING_FOR_SYSTEM'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__WAITING_FOR_SYSTEM = 0
};

/// Constant 'NAVIGATING'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__NAVIGATING = 1
};

/// Constant 'MOVING_TO_PREPRESS'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__MOVING_TO_PREPRESS = 2
};

/// Constant 'APPROACHING'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__APPROACHING = 3
};

/// Constant 'PRESSING'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__PRESSING = 4
};

/// Constant 'VERIFYING_PRESS'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__VERIFYING_PRESS = 5
};

/// Constant 'RETRACTING'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__RETRACTING = 6
};

/// Constant 'VERIFYING_RELEASE'.
enum
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback__VERIFYING_RELEASE = 7
};

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_Feedback
{
  uint8_t phase;
  float progress;
  double button_travel;
  rosidl_runtime_c__String message;
} xczs_inspection_robot_control__action__PressCabinetButton_Feedback;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_Feedback.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_Feedback__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "xczs_inspection_robot_control/action/detail/press_cabinet_button__struct.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  xczs_inspection_robot_control__action__PressCabinetButton_Goal goal;
} xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Request;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Request.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Request__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Response;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Response.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Response__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Request;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Request.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Request__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "xczs_inspection_robot_control/action/detail/press_cabinet_button__struct.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Response
{
  int8_t status;
  xczs_inspection_robot_control__action__PressCabinetButton_Result result;
} xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Response;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Response.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Response__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "xczs_inspection_robot_control/action/detail/press_cabinet_button__struct.h"

/// Struct defined in action/PressCabinetButton in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  xczs_inspection_robot_control__action__PressCabinetButton_Feedback feedback;
} xczs_inspection_robot_control__action__PressCabinetButton_FeedbackMessage;

// Struct for a sequence of xczs_inspection_robot_control__action__PressCabinetButton_FeedbackMessage.
typedef struct xczs_inspection_robot_control__action__PressCabinetButton_FeedbackMessage__Sequence
{
  xczs_inspection_robot_control__action__PressCabinetButton_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__action__PressCabinetButton_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__PRESS_CABINET_BUTTON__STRUCT_H_
