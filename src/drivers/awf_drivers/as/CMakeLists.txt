cmake_minimum_required(VERSION 2.8.3)
project(as)

find_package(autoware_build_flags REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS
  automotive_navigation_msgs
  automotive_platform_msgs
  autoware_msgs
  geometry_msgs
  message_filters
  ros_observer
  roscpp
  roslint
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ssc_interface
  src/ssc_interface_node.cpp
  src/ssc_interface.cpp
)

target_link_libraries(ssc_interface ${catkin_LIBRARIES})

add_dependencies(ssc_interface ${catkin_EXPORTED_TARGETS})

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

install(TARGETS ssc_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
