cmake_minimum_required(VERSION 3.0.2)
project(uvone_robot_navigation)

find_package(catkin REQUIRED 
  roscpp
  actionlib
  tf
  tf2
  std_msgs
  tf2_geometry_msgs
  uvone_robot_msgs
  sensor_door_msgs
  move_base_msgs
  roslib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(command_sender src/command_sender.cpp)
add_dependencies(command_sender ${catkin_EXPORTED_TARGETS})
target_link_libraries(command_sender ${catkin_LIBRARIES})

add_executable(command_writer src/command_writer.cpp)
add_dependencies(command_writer ${catkin_EXPORTED_TARGETS})
target_link_libraries(command_writer ${catkin_LIBRARIES})
