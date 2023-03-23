cmake_minimum_required(VERSION 3.0.2)
project(test_hd)

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
 LIBRARIES test_hd
 CATKIN_DEPENDS hd_servo roscpp rospy std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS} #引用每一个功能包中的include目录的头文件
  include/${PROJECT_NAME}
)


add_executable(${PROJECT_NAME}
# src/main.cpp
src/cloth_manipulate.cpp
# src/test_layer.cpp #主程序1：两布两层
src/kukaservo_lib.cpp
## src/test_kuka.cpp #测试kuka的程序
# src/test_TwoCloth.cpp #主程序2：两布四层
src/test_MoveSingleCloth.cpp #主程序3：单布移动

# src/test_4_19.cpp #测试
# src/test_4_26.cpp


)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

