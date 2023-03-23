cmake_minimum_required(VERSION 3.0.2)
project(control_hd)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  hd_servo
  get_height
  roscpp
  rospy
  std_msgs
  kukafri_hw
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  state.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES control_hd
 CATKIN_DEPENDS hd_servo roscpp rospy std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS} #引用每一个功能包中的include目录的头文件
  include/${PROJECT_NAME}
)


add_executable(${PROJECT_NAME}
src/auto_clothmanipulation.cpp
# src/kukaservo_lib.cpp
src/AutoMoveSingleCloth.cpp #单布自适应移动,订阅节点消息，消息内容为下一操作单元的目标移动距离
)

# add_executable(${PROJECT_NAME}_main
# src/use_automanipulation.cpp
# src/kukaservo_lib.cpp
# src/AutoMoveSingleCloth.cpp #单布自适应移动
# )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

