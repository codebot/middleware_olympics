cmake_minimum_required(VERSION 3.5)
project(zenoh_contestant)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) # -Wpedantic)
endif()

include_directories(../../build/zenoh/zenoh/zenoh-ffi/include)
link_directories(../../build/zenoh/zenoh/target/release)

add_executable(publisher publisher.cpp)
target_link_libraries(publisher zenoh_ffi)

add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber zenoh_ffi)

add_executable(echo echo.cpp)
target_link_libraries(echo zenoh_ffi)

add_executable(ping ping.cpp)
target_link_libraries(ping zenoh_ffi)

add_executable(table_tennis_server table_tennis_server.cpp)
target_link_libraries(table_tennis_server zenoh_ffi)

add_executable(table_tennis_receiver table_tennis_receiver.cpp)
target_link_libraries(table_tennis_receiver zenoh_ffi)

add_executable(discovery_publisher discovery_publisher.cpp)
target_link_libraries(discovery_publisher zenoh_ffi)

add_executable(discovery_subscriber discovery_subscriber.cpp)
target_link_libraries(discovery_subscriber zenoh_ffi)
