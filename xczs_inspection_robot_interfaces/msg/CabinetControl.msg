uint8 TYPE_BUTTON=0
uint8 TYPE_KNOB=1
uint8 TYPE_SWITCH=2
uint8 TYPE_DOOR=3
# A prismatic slider (linear push along its axis) with detent latching.  The
# robot presses the face, the plugin latches the door to the nearest detent.
uint8 TYPE_SLIDER=4

uint8 SUPPORT_PRESS=1
uint8 SUPPORT_SET_STATE=2
uint8 SUPPORT_SET_POSITION=4
uint8 SUPPORT_TOGGLE=8

string control_id
string display_name
uint8 control_type
string joint_name
string joint_state_topic
string pressed_topic
string state_topic
uint8 supported_commands
string unit
float64 min_position
float64 max_position
string[] state_ids
string[] state_labels
float64[] state_positions
bool requires_grasp
bool operable
string unavailable_reason
# The end-effector set required by this control (A or B).  This is exposed
# separately from ``operable`` so clients can provide an actionable error
# before attempting navigation when the other set is mounted.
string required_toolset
bool toolset_compatible
# True when this control is in the robot adapter's physically validated
# allowlist.  A false value means planning-only validation is allowed, but no
# physical cabinet command may be sent.
bool adapter_validated
float64 default_force
float64 min_trigger_force
float64 max_force
