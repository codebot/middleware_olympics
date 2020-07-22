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
