# To not break the API of /cmd_mode, this message is actually sent as a
# std_msgs/UInt16. The valid constants are however defined here.

# Enter manual (ROS controlled) mode
uint16 MODE_MANUAL = 144
# Enter random mode
uint16 MODE_RANDOM = 145

# Disable/enable grass cutting
uint16 MODE_CUTTING_DISC_OFF = 146
uint16 MODE_CUTTING_DISC_ON = 147

# Some predefined heights for the disc
uint16 MODE_CUTTING_HEIGHT_60 = 148
uint16 MODE_CUTTING_HEIGHT_40 = 149

# Enter park mode
uint16 MODE_PARK = 256

# Enable loop detection
uint16 MODE_LOOP_ON = 272
# Disable loop detection
uint16 MODE_LOOP_OFF = 273

# Inject collisions
uint16 MODE_COLLISION_INJECT = 274
uint16 MODE_COLLISIONS_DISABLED = 275
uint16 MODE_COLLISIONS_ENABLED = 276

# Various sounds
uint16 MODE_SOUND_KEY_CLICK = 1024
uint16 MODE_SOUND_CLICK = 1025
uint16 MODE_SOUND_CHARGING_CONNECTED = 1026
uint16 MODE_SOUND_CHARGING_DISCONNECTED = 1027
uint16 MODE_SOUND_DOUBLE_BEEP = 1028
uint16 MODE_SOUND_LONG_BEEP = 1029
uint16 MODE_SOUND_FAULT = 1030
uint16 MODE_SOUND_START_CUTTING = 1031
uint16 MODE_SOUND_ALARM_WARNING = 1032
uint16 MODE_SOUND_ALARM_1 = 1033
uint16 MODE_SOUND_ALARM_2 = 1034
uint16 MODE_SOUND_ALARM_5 = 1035
uint16 MODE_SOUND_ALARM_10 = 1036
uint16 MODE_SOUND_TONE_1 = 1037
uint16 MODE_SOUND_OFF = 1038

# Follow modes
uint16 MODE_FOLLOW_MANUAL = 1040
uint16 MODE_FOLLOW_G1 = 1041
uint16 MODE_FOLLOW_G2 = 1042
uint16 MODE_FOLLOW_G3 = 1043

# System commands
uint16 MODE_SHUTDOWN = 4096
uint16 MODE_REBOOT = 4097