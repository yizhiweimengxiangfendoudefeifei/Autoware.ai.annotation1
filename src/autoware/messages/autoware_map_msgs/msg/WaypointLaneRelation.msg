# This represents the connection between waypoints and lanes

# Id of refering Waypoint object
int32 waypoint_id

# Id of Lane object which refering waypoint belongs to 
int32 lane_id

# order of the waypoint in the lane. 
# e.g. starting waypoint of the lane will have order of 0. 
int32 order
