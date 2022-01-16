# This represents a waypoint in a map. 

# Id of this Waypoint object
int32 waypoint_id

# Id of Point that represents the position of this waypoint
int32 point_id

# reference velocity of this waypoint. [km/h]
float64 velocity

# describes whether vehicle must stop at this waypoint
# no_stop = 0, stop = 1
int32 stop_line

# distance to left border of the belonging lane in [m]
float64 left_width

# distance to right border of the belonging lane in [m]
float64 right_width

# height limit for the vehicle to drive this waypoint [m]
float64 height
