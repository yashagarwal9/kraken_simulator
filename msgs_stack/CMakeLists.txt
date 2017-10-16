cmake_minimum_required(VERSION 2.8.3)
project(msgs_stack)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
std_msgs
geometry_msgs  
message_generation)

add_message_files(
  FILES
  thrusterData6.msg
)

generate_messages(
DEPENDENCIES
 std_msgs
 geometry_msgs
)


catkin_package(
CATKIN_DEPENDS 
message_runtime
)






