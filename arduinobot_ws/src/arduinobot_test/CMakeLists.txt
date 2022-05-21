cmake_minimum_required(VERSION 3.0.2)
project(arduinobot_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
)

add_service_files(
  FILES
  AddTwoInts.srv
)

add_action_files(
  FILES
  Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    scripts/simple_publisher.py 
    scripts/simple_subscriber.py
    scripts/simple_timer.py
    scripts/simple_service.py
    scripts/simple_service_client.py
    scripts/simple_action_server.py
    scripts/simple_action_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(simple_cpp_publisher src/simple_publisher.cpp)
target_link_libraries(simple_cpp_publisher ${catkin_LIBRARIES})

add_executable(simple_cpp_subscriber src/simple_subscriber.cpp)
target_link_libraries(simple_cpp_subscriber ${catkin_LIBRARIES})

add_executable(simple_cpp_timer src/simple_timer.cpp)
target_link_libraries(simple_cpp_timer ${catkin_LIBRARIES})

add_executable(simple_cpp_service src/simple_service.cpp)
add_dependencies(simple_cpp_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_cpp_service ${catkin_LIBRARIES})

add_executable(simple_cpp_service_client src/simple_service_client.cpp)
add_dependencies(simple_cpp_service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_cpp_service_client ${catkin_LIBRARIES})

add_executable(simple_cpp_action_server src/simple_action_server.cpp)
add_dependencies(simple_cpp_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_cpp_action_server ${catkin_LIBRARIES})

add_executable(simple_cpp_action_client src/simple_action_client.cpp)
add_dependencies(simple_cpp_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_cpp_action_client ${catkin_LIBRARIES})