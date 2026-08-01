uint8 TYPE_BUTTON=0
uint8 TYPE_KNOB=1
uint8 TYPE_SWITCH=2
uint8 TYPE_DOOR=3

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
