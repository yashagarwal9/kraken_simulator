cmake_minimum_required(VERSION 2.8.3)
project(plugins)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  geometry_msgs
  kraken_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES plugins
  CATKIN_DEPENDS roscpp geometry_msgs kraken_msgs tf sensor_msgs
  DEPENDS
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
  include
  #kraken_msgs/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(_BouyancyPlugin src/BuoyancyPlugin.cpp)
#target_link_libraries(_BouyancyPlugin ${catkin_LIBRARIES} ${GAZEBO_libraries})
#add_dependencies(_BouyancyPlugin ${catkin_EXPORTED_TARGETS} ${GAZEBO_libraries})

add_library(KrakenControlPlugin src/KrakenControl.cpp)
target_link_libraries(KrakenControlPlugin ${catkin_LIBRARIES} ${GAZEBO_libraries})
add_dependencies(KrakenControlPlugin ${catkin_EXPORTED_TARGETS} ${GAZEBO_libraries})

add_library(_Hydrodynamics src/Hydrodynamics.cpp)
target_link_libraries(_Hydrodynamics ${catkin_LIBRARIES} ${GAZEBO_libraries})
add_dependencies(_Hydrodynamics ${catkin_EXPORTED_TARGETS} ${GAZEBO_libraries})

add_executable(transform_br src/transformer.cpp)
target_link_libraries(transform_br ${catkin_LIBRARIES})
add_dependencies(transform_br ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation

 install(TARGETS KrakenControlPlugin
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(TARGETS _Hydrodynamics
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
