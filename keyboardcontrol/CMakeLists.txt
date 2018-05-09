cmake_minimum_required(VERSION 2.8.3)
project(keyboardcontrol)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  msgs_stack
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES keyboard_control
  CATKIN_DEPENDS geometry_msgs roscpp rospy msgs_stack
  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(KeyboardControl_cpp src/keyboard_control.cpp)
target_link_libraries(KeyboardControl_cpp ${catkin_LIBRARIES})
add_dependencies(KeyboardControl_cpp   ${catkin_EXPORTED_TARGETS})

#install(TARGETS KeyboardControl_cpp
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
