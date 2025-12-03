cmake_minimum_required(VERSION 3.0.2)
project(robbie)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Find catkin and any ROS packages you need
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

## Find Gazebo
find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ClueboardHandler
  CATKIN_DEPENDS roscpp geometry_msgs
)

###########
## Build ##
###########

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Build the plugin as a shared library
add_library(ClueboardHandler
  src/ClueboardHandler.cpp
)

# Link against ROS and Gazebo
target_link_libraries(ClueboardHandler
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Make sure the plugin is installed so Gazebo can find it
install(TARGETS ClueboardHandler
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)