cmake_minimum_required(VERSION 2.8.3)
project(autoware_quickstart_examples)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
