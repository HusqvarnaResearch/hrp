# Topic that is meant to reflect contents of command family "CurrentStatus" with command "GetStatusKeepAlive" from Automower TIF

Header header

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
int16 state

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
int16 subState

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
int16 mode
