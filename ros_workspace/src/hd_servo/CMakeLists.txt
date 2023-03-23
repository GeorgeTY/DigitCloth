cmake_minimum_required(VERSION 3.0.2)
project(hd_servo)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
message_generation
  roscpp
  rospy
  std_msgs
)
add_message_files(
  FILES
  # motor_feedback.msg
  EndPos.msg
  JointSpeed.msg
  # FingerTip_offset.msg
 
)
add_service_files(
  FILES
  max_load.srv
 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES hd_servo  #添加后导致包test_hd 无法链接此库
 CATKIN_DEPENDS roscpp rospy std_msgs
 message_runtime
 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
    include/${PROJECT_NAME}
)

ADD_DEFINITIONS(-DOS_UNIX)



add_executable(${PROJECT_NAME}
src/main.cpp
src/hd_drive.cpp
src/matrix_func.cpp
src/calc_pathInterpolation.cpp
src/hd_kinematics.cpp
src/time.cpp
src/SCSCL.cpp
src/SMSCL.cpp
src/SCS.cpp
src/SMSBL.cpp
src/SCSerial.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)