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
add_executable(basic_controller_interface_cpp src/basic_controller_interface.cpp)
add_executable(arduinobot_interface_node src/arduinobot_interface.cpp)

add_dependencies(angles_converter_cpp arduinobot_controller_gencpp)
add_dependencies(basic_controller_interface_cpp arduinobot_controller_gencpp)
add_dependencies(arduinobot_interface_node ${arduinobot_controller_EXPORTED_TARGETS})

target_link_libraries(angles_converter_cpp
  ${catkin_LIBRARIES}
)
target_link_libraries(basic_controller_interface_cpp
  ${catkin_LIBRARIES}
)
target_link_libraries(arduinobot_interface_node
  ${catkin_LIBRARIES}
)