# This describes an attribute of lane in a map.

# ATTRIBUTE_TYPES
uint32 PLANE = 1
uint32 INTERSECTION = 2
uint32 CROSS_WALK = 3
uint32 DISABLE_LANE_CHANGE = 4
uint32 DISABLE_PARKING = 5
uint32 RAILROAD = 6
uint32 PEDESTRIAN_SPACE = 7
uint32 PARKING_AREA = 8

# Id of Lane object which attribute is applied.
int32 lane_id

# Attribute that is applied to lane
# Must be one of ATTRIBUTE_TYPES
int32 attribute_type

# Id of Area object that is relevant to the attribute (if exists). 
# e.g. Id of Area object that describes crosswalk when attribute_type = CROSS_WALK. 
int32 area_id
