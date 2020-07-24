cmake_minimum_required(VERSION 3.5)
project(ros2_contestant)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StampedBlob.msg"
  DEPENDENCIES builtin_interfaces
)

add_executable(publisher publisher.cpp)
ament_target_dependencies(publisher rclcpp)
rosidl_target_interfaces(publisher
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(subscriber subscriber.cpp)
ament_target_dependencies(subscriber rclcpp)
rosidl_target_interfaces(subscriber
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(echo echo.cpp)
ament_target_dependencies(echo rclcpp)
rosidl_target_interfaces(echo
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(ping ping.cpp)
ament_target_dependencies(ping rclcpp)
rosidl_target_interfaces(ping
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(table_tennis_receiver table_tennis_receiver.cpp)
ament_target_dependencies(table_tennis_receiver rclcpp)
rosidl_target_interfaces(table_tennis_receiver
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(table_tennis_server table_tennis_server.cpp)
ament_target_dependencies(table_tennis_server rclcpp)
rosidl_target_interfaces(table_tennis_server
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  publisher
  subscriber
  echo
  ping
  table_tennis_receiver
  table_tennis_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
