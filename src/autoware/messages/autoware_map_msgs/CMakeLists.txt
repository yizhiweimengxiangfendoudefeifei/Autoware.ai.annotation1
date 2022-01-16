cmake_minimum_required(VERSION 2.8.3)
project(autoware_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
    Area.msg
    AreaArray.msg
    Lane.msg
    LaneArray.msg
    LaneAttributeRelation.msg
    LaneAttributeRelationArray.msg
    LaneChangeRelation.msg
    LaneChangeRelationArray.msg
    LaneRelation.msg
    LaneRelationArray.msg
    LaneSignalLightRelation.msg
    LaneSignalLightRelationArray.msg
    OppositeLaneRelation.msg
    OppositeLaneRelationArray.msg
    Point.msg
    PointArray.msg
    Signal.msg
    SignalArray.msg
    SignalLight.msg
    SignalLightArray.msg
    Wayarea.msg
    WayareaArray.msg
    Waypoint.msg
    WaypointArray.msg
    WaypointLaneRelation.msg
    WaypointLaneRelationArray.msg
    WaypointRelation.msg
    WaypointRelationArray.msg
    WaypointSignalRelation.msg
    WaypointSignalRelationArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
