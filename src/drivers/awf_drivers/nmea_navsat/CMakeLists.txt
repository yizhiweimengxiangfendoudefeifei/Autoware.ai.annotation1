cmake_minimum_required(VERSION 2.8.3)
project(nmea_navsat)

find_package(catkin REQUIRED COMPONENTS nmea_navsat_driver)

catkin_package()

#add_executable(javad_tty nodes/javad_tty/javad_tty.cpp)
#target_link_libraries(javad_tty
#   ${catkin_LIBRARIES}
#)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
