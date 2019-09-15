# Topic that is meant to reflect contents of command family "CurrentStatus" with command "GetStatusKeepAlive" from Automower TIF

# -1 = undefined
# 0 = Power-up state
# 1 = Waiting state
# 2 = Irregular cutting state
# 3 = Searching CS state
# 4 = Charging state
# 5 = Leaving CS state
# 6 = SW downloading state
# 7 = Fault state. Fatal Error
# 8 = Fault state. Error
int16 STATE_UNDEFINED = -1
int16 STATE_POWER_UP = 0
int16 STATE_WAITING = 1
int16 STATE_IRREGULAR_CUTTING = 2
int16 STATE_SEARCHING_CS = 3
int16 STATE_CHARGING = 4
int16 STATE_LEAVING_CS = 5
int16 STATE_SW_DOWNLOADING = 6
int16 STATE_FAULT_FATAL_ERROR = 7
int16 STATE_FAULT_ERROR = 8

# -1 = undefined
# 0 = No sub status
# 1 = Irregular moving
# 2 = Complex outside
# 3 = Complex wheels blocked
# 4 = Complex wheels slipped
# 5 = Complex outside at reverse
# 6 = Complex collision
# 7 = Complex lifted
# 8 = Local intensive cutting
# 9 = Follow guide wire 1 in
# 10 = Follow right boundary wire in
# 11 = Docking
# 12 = Fast charging
# 13 = Top-up charging
# 14 = Charging ready
# 15 = Exit angle
# 16 = Follow guide wire 1 out
# 17 = Follow right boundary wire out
# 18 = Complex No Loop Signal
# 19 = Complex Cutting Means blocked
# 20 = Complex Cutting Means blocked
# 21 = Follow guide wire 2 in
# 22 = Follow guide wire 3 in
# 23 = Follow guide wire 2 out
# 24 = Follow guide wire 3 out
# 25 = Follow left boundary wire in
# 26 = Follow left boundary wire out
# 27 = Complex too steep slope
int16 SUBSTATE_UNDEFINED = -1
int16 SUBSTATE_NONE = 0
int16 SUBSTATE_IRREGULAR_MOVING = 1
int16 SUBSTATE_COMPLEX_OUTSIDE = 2
int16 SUBSTATE_COMPLEX_WHEELS_BLOCKED = 3
int16 SUBSTATE_COMPLEX_WHEELS_SLIPPED = 4
int16 SUBSTATE_COMPLEX_OUTSIDE_AT_REVERSE = 5
int16 SUBSTATE_COMPLEX_COLLISION = 6
int16 SUBSTATE_COMPLEX_LIFTED = 7
int16 SUBSTATE_LOCAL_INTENSIVE_CUTTING = 8
int16 SUBSTATE_FOLLOW_GUIDE_WIRE_1_IN = 9
int16 SUBSTATE_FOLLOW_RIGHT_BOUNDARY_WIRE_IN = 10
int16 SUBSTATE_DOCKING = 11
int16 SUBSTATE_CHARGING_FAST = 12
int16 SUBSTATE_CHARGING_TOP_UP = 13
int16 SUBSTATE_CHARGING_READY = 14
int16 SUBSTATE_EXIT_ANGLE = 15
int16 SUBSTATE_FOLLOW_GUIDE_WIRE_1_OUT = 16
int16 SUBSTATE_FOLLOW_RIGHT_BOUNDARY_WIRE_OUT = 17
int16 SUBSTATE_COMPLEX_NO_LOOP_SIGNAL = 18
int16 SUBSTATE_COMPLEX_CUTTING_MEANS_BLOCKED_1 = 19
int16 SUBSTATE_COMPLEX_CUTTING_MEANS_BLOCKED_2 = 20
int16 SUBSTATE_FOLLOW_GUIDE_WIRE_2_IN = 21
int16 SUBSTATE_FOLLOW_GUIDE_WIRE_3_IN = 22
int16 SUBSTATE_FOLLOW_GUIDE_WIRE_2_OUT = 23
int16 SUBSTATE_FOLLOW_GUIDE_WIRE_3_OUT = 24
int16 SUBSTATE_FOLLOW_LEFT_BOUNDARY_WIRE_IN = 25
int16 SUBSTATE_FOLLOW_LEFT_BOUNDARY_WIRE_OUT = 26
int16 SUBSTATE_COMPLEX_TOO_STEEP_SLOPE = 27


# -1 = undefined
# 0 = Power-down mode
# 1 = Standby mode
# 2 = Sleep mode
# 3 = Sub -active mode
# 4 = Boot mode
# 5 = Idle mode
# 6 = Auto mode
# 7 = Manual mode
# 8 = Home mode
int16 MODE_UNDEFINED = -1
int16 MODE_POWER_DOWN = 0
int16 MODE_STANDBY = 1
int16 MODE_SLEEP = 2
int16 MODE_SUB_ACTIVE = 3
int16 MODE_BOOT = 4
int16 MODE_IDLE = 5
int16 MODE_AUTO = 6
int16 MODE_MANUAL = 7
int16 MODE_HOME = 8

Header header

int16 state
int16 subState
int16 mode
