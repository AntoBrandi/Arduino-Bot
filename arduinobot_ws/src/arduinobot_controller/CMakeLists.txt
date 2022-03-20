cmake_minimum_required(VERSION 3.0.2)
project(arduinobot_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  controller_manager
)

find_package(Boost REQUIRED COMPONENTS system)


add_service_files(
  FILES
  AnglesConverter.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(angles_converter_cpp src/angles_converter.cpp)
add_dependencies(angles_converter_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angles_converter_cpp ${catkin_LIBRARIES})

add_executable(arduinobot_interface_node src/arduinobot_interface.cpp)
add_dependencies(arduinobot_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arduinobot_interface_node ${catkin_LIBRARIES})