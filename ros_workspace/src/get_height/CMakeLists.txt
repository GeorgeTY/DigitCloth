cmake_minimum_required(VERSION 3.0.2)
project(get_height)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Height.msg
  Odometer_result.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES get_height
CATKIN_DEPENDS rospy cv_bridge message_runtime std_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
