cmake_minimum_required(VERSION 3.0.2)
project(arduinobot_remote)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib_msgs                
  std_msgs                      
  message_generation 
  moveit_ros_planning_interface
)

add_action_files(
  DIRECTORY action
  FILES ArduinobotTask.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)


catkin_package(
  CATKIN_DEPENDS std_msgs actionlib_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(moveit_interface
  src/moveit_interface.cpp
)

target_link_libraries(moveit_interface
  ${catkin_LIBRARIES}
)

add_executable(task_server src/task_server.cpp)
add_dependencies(task_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  task_server
  ${catkin_LIBRARIES}
  moveit_interface
)
