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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StampedBlob.msg"
)

add_executable(publisher publisher.cpp)
ament_target_dependencies(publisher rclcpp)
rosidl_target_interfaces(publisher
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(subscriber subscriber.cpp)
ament_target_dependencies(subscriber rclcpp)
rosidl_target_interfaces(subscriber
  ${PROJECT_NAME} rosidl_typesupport_cpp)


ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
