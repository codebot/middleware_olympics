cmake_minimum_required(VERSION 2.8.3)
project(ros1_contestant)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
)

set(CMAKE_CXX_STANDARD 14)

add_message_files(
  FILES
    StampedBlob.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roslib
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(publisher publisher.cpp)
add_dependencies(publisher
  ${ros1_contestant_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(publisher ${catkin_LIBRARIES})

add_executable(subscriber subscriber.cpp)
add_dependencies(subscriber
  ${ros1_contestant_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber ${catkin_LIBRARIES})

add_executable(echo echo.cpp)
add_dependencies(echo
  ${ros1_contestant_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(echo ${catkin_LIBRARIES})

add_executable(ping ping.cpp)
add_dependencies(ping
  ${ros1_contestant_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(ping ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS publisher subscriber
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
