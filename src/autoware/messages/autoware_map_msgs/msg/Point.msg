# This represents a point in a map.

# Id of this Point object. Must be unique among all points in a map.
int32 point_id

# Position of a point in MGRS coordinate [m]
# x = northing
# y = easting
# z = elevation
float64 x
float64 y
float64 z

# Codes that describe coordinate of the x,y,z values. 
int32 mgrs
int32 epsg

# PCD file name which that is relevant. 
string pcd

# latitude and longitude in WGS84 coordinate in [deg]
float64 lat
float64 lng
