cmake_minimum_required(VERSION 2.8.3)
project(robbie)

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
  LIBRARIES SimpleForceDronePlugin
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
add_library(SimpleForceDronePlugin
  src/drone_plugin/SimpleForceDronePlugin.cpp
)

# Link against ROS and Gazebo
target_link_libraries(SimpleForceDronePlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Make sure the plugin is installed so Gazebo can find it
install(TARGETS SimpleForceDronePlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)