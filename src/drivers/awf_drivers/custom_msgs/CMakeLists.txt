cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
 FILES
   BaroSensorSample.msg
   GnssSensorSample.msg
   ImuSensorSample.msg
   Internal.msg
   XsensQuaternion.msg
   baroSample.msg
   gnssSample.msg
   orientationEstimate.msg
   positionEstimate.msg       
   sensorSample.msg
   velocityEstimate.msg
 )

generate_messages(
 DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs  
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)
