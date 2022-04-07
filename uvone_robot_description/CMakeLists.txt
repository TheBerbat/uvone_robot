cmake_minimum_required(VERSION 3.0.2)
project(uvone_robot_description)

find_package(catkin REQUIRED COMPONENTS urdf xacro)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)
