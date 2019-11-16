cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_kdl_ik)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  control_msgs
  kdl_parser
  kdl_parser_py
  orocos_kdl
  roscpp
  rospy
  trajectory_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kdl_ik src/kdl_ik.cpp)
target_link_libraries(kdl_ik ${catkin_LIBRARIES})