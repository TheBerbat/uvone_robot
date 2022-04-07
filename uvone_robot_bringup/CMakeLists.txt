cmake_minimum_required(VERSION 3.0.2)
project(uvone_robot_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  uvone_robot_msgs
  sensor_door_msgs
  kobuki_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(light_node src/light_node.cpp)
target_link_libraries(light_node ${catkin_LIBRARIES})
add_dependencies(light_node ${catkin_EXPORTED_TARGETS})

add_executable(light_battery_node src/light_battery_node.cpp)
target_link_libraries(light_battery_node ${catkin_LIBRARIES})
add_dependencies(light_battery_node ${catkin_EXPORTED_TARGETS})
