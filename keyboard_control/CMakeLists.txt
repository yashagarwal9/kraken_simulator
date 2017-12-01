cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  msgs_stack
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES keyboard_control
  CATKIN_DEPENDS geometry_msgs roscpp rospy msgs_stack
  DEPENDS system_lib
)


add_executable(Keyboard_Control_cpp src/keyboard_control.cpp)
target_link_libraries(Keyboard_Control_cpp ${catkin_LIBRARIES})

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)
