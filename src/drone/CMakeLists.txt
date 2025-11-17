cmake_minimum_required(VERSION 2.8.3)
project(drone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(SimpleBodyDroneControlPlugin
  drone_plugin/src/SimpleBodyDroneControlPlugin.cpp
)

target_link_libraries(SimpleBodyDroneControlPlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS SimpleBodyDroneControlPlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
