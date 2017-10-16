cmake_minimum_required(VERSION 2.8.3)
project(plugins)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  msgs_stack
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS} 
  LIBRARIES plugins 
  CATKIN_DEPENDS roscpp geometry_msgs msgs_stack
  DEPENDS system_lib  gazebo
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
  include 
  msgs_stack/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(_BouyancyPlugin src/BuoyancyPlugin.cpp)
target_link_libraries(_BouyancyPlugin ${catkin_LIBRARIES} ${GAZEBO_libraries})
add_dependencies(_BouyancyPlugin ${catkin_EXPORTED_TARGETS} ${GAZEBO_libraries})

add_library(KrakenControlPlugin src/KrakenControl.cpp)
target_link_libraries(KrakenControlPlugin ${catkin_LIBRARIES} ${GAZEBO_libraries})
add_dependencies(KrakenControlPlugin ${catkin_EXPORTED_TARGETS} ${GAZEBO_libraries})


## Mark executables and/or libraries for installation
install(TARGETS _BouyancyPlugin
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
 install(TARGETS KrakenControlPlugin
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".~" EXCLUDE
 )
