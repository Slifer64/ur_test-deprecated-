cmake_minimum_required(VERSION 2.8.3)
project(ur_robot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  io_lib
  ur_kinematics
  ur_modern_driver
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ur_modern_driver
  DEPENDS
)

set(SRC_LIST
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# *************************************************
# *************************************************

add_library(ur_robot
        src/ur_robot.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(ur_robot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(ur_robot
  ${catkin_LIBRARIES}
  armadillo
  io_lib
  ur10_kin
  ur_modern_driver
)


# *************************************************
# *************************************************
