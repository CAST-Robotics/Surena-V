cmake_minimum_required(VERSION 3.0.2)
project(surenaV_gazebo)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gazebo_bridge_node src/gazebo_bridge.cpp)
target_link_libraries(gazebo_bridge_node ${catkin_LIBRARIES})