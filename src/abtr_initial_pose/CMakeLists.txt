cmake_minimum_required(VERSION 3.0.2)
project(abtr_initial_pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  cartographer_ros_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cartographer_initialpose_bridge
  src/cartographer_initialpose_bridge.cpp
)

target_link_libraries(cartographer_initialpose_bridge
  ${catkin_LIBRARIES}
)

