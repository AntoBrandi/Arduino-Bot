cmake_minimum_required(VERSION 3.5)
project(arduinobot_controller)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

add_library(${PROJECT_NAME} SHARED 
  src/arduinobot_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}
 "${SERIAL_LDFLAGS}")
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface arduinobot_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()