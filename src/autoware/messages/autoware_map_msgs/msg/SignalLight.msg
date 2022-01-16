# This represents each lamps in signal light in a map.

# COLOR_TYPES
uint8 RED = 1
uint8 GREEN = 2
uint8 YELLOW = 3
uint8 RED_FLASHING = 4
uint8 GREEN_FLASHING = 5
uint8 YELLOW_FLASHING = 6
uint8 UNKOWN = 7

# ARROW_TYPES
uint8 NONE = 0
uint8 UP = 1
uint8 UP_RIGHT = 2
uint8 RIGHT = 3
uint8 DOWN_RIGHT = 4
uint8 DOWN = 5
uint8 DOWN_LEFT = 6
uint8 LEFT = 7
uint8 UP_LEFT = 8
uint8 UNKNOWN = 9

# Id of this SignalLight object. Must be uniqe among all SignalLight objects. 
int32 signal_light_id

# Id of Signal object which this SignalLight object belongs to
int32 signal_id

# Id of point that represents the position of this lamp. 
int32 point_id

# Horizontal direction which this SignalLight is facing in [deg].
# Angle is calculated from North in clockwise direction. 
float64 horizontal_angle

# Vertical direction which this SignalLight is facing in [deg].
# Angle is calculated from vertical rising direction. 
float64 vertical_angle

# Color of the lamp
# Must be one of COLOR_TYPES
int32 color_type

# Direction of the arrow (if the lamp has direction)
# Must be one of ARROW_TYPES
int32 arrow_type
