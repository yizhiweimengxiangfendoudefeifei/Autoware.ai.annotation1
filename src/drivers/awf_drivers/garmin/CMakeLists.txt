cmake_minimum_required(VERSION 2.8.3)
project(garmin)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nmea_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(garmin_gps_18x_lvc src/garmin_gps_18x_lvc.cc)

target_link_libraries(garmin_gps_18x_lvc ${catkin_LIBRARIES})

install(TARGETS garmin_gps_18x_lvc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
