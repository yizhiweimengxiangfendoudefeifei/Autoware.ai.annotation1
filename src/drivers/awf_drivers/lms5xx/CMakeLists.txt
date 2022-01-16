cmake_minimum_required(VERSION 2.8.3)
project(sick_lms5xx)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  nodes/lms511/include/lms511
)

add_executable(lms511_node
  nodes/lms511/src/lms5xx_node.cpp
  nodes/lms511/src/SickLMS5xx.cc
  nodes/lms511/src/SickLMS5xxBufferMonitor.cc
  nodes/lms511/src/SickLMS5xxMessage.cc
)
add_dependencies(lms511_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lms511_node
 ${catkin_LIBRARIES}
)

install(TARGETS lms511_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
