# This represents a lane in a map.

# Id of this Lane object. Must be unique among all lane objects.
int32 lane_id

# Id of the first waypoint that belongs to this lane
int32 start_waypoint_id

# Id of the last waypoint that belongs to this lane
int32 end_waypoint_id

# This describes how many lanes there are in left side of this lane.
# E.g. If there are 2 lanes on the left side, then lane_number will be 2. 
# If the road is single lane, then this will be 0. 
# This will be always 0 in intersection. 
int32 lane_number

# Total number of lanes present in road. 
int32 num_of_lanes

# Speed limit of this lane in [km/h]
float64 speed_limit

# Length of this lane in [m]
# i.e. accumulated length from start_waypoint to end_waypoint of this lane
float64 length

# Maximum width of vehicle that can drive this lane in [m]
float64 width_limit

# Maximum height of vehicle that can drive this lane in [m]. 
float64 height_limit

# Maximum weight of vehicle that can drive this lane in [kg]. 
float64 weight_limit
