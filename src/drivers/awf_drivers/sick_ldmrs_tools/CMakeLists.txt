cmake_minimum_required(VERSION 2.8.3)
project(sick_ldmrs_tools)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  roslaunch
  sick_ldmrs_msgs
  tf
  visualization_msgs
)

find_package(PCL 1.7 REQUIRED REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(sick_ldmrs_object_marker src/sick_ldmrs_object_marker.cpp)
add_executable(sick_ldmrs_filter_layer src/sick_ldmrs_filter_layer.cpp)
add_executable(sick_ldmrs_filter_first src/sick_ldmrs_filter_first.cpp)
add_executable(sick_ldmrs_filter_last src/sick_ldmrs_filter_last.cpp)
add_executable(sick_ldmrs_all_layer_assembler src/sick_ldmrs_all_layer_assembler.cpp)
add_executable(sick_ldmrs_make_organized src/sick_ldmrs_make_organized.cpp)
add_executable(sick_ldmrs_remove_background src/sick_ldmrs_remove_background.cpp)
add_executable(sick_ldmrs_print_resolution src/sick_ldmrs_print_resolution.cpp)

add_dependencies(sick_ldmrs_object_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_filter_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_filter_first ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_filter_last ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_all_layer_assembler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_make_organized ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_remove_background ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sick_ldmrs_print_resolution ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sick_ldmrs_object_marker
  ${catkin_LIBRARIES}
)

target_link_libraries(sick_ldmrs_filter_layer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_filter_first
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_filter_last
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_all_layer_assembler
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_make_organized
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_remove_background
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(sick_ldmrs_print_resolution
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
  TARGETS
    sick_ldmrs_object_marker
    sick_ldmrs_filter_layer
    sick_ldmrs_filter_first
    sick_ldmrs_filter_last
    sick_ldmrs_all_layer_assembler
    sick_ldmrs_make_organized
    sick_ldmrs_remove_background
    sick_ldmrs_print_resolution
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

roslaunch_add_file_check(launch)
