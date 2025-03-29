cmake_minimum_required(VERSION 3.8)
project(ros2_action_example)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Example.action"
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Action Server
add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  std_msgs
)
target_link_libraries(action_server
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Action Client
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client
  rclcpp
  rclcpp_action
  std_msgs
)
target_link_libraries(action_client
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()