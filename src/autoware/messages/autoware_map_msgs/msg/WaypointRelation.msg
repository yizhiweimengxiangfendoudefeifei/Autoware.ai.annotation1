# This represents the connection between a waypoint and a lane.

# Id of refering Waypoint object
int32 waypoint_id

# Id of Waypoint object that follows the refering waypoint
int32 next_waypoint_id

# Direction from the refering waypoint to next waypoint in [rad].
# The angle is calculated from North in counter clockwise. 
float64 yaw

# Blinker that vehicle must light up when driving from waypoint to next_waypoint
int32 blinker

# Distance between waypoint and next_waypoint
float64 distance
