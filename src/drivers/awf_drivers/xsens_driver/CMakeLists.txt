cmake_minimum_required(VERSION 2.8.3)
project(xsens_driver)

find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  diagnostic_msgs
  geometry_msgs
  gps_common
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  PROGRAMS
    src/mtnode.py
    src/mtdef.py
    src/mtdevice.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
