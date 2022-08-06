cmake_minimum_required(VERSION 3.0.2)
project(test_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package( Threads )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(./src SRCS)
add_executable(test_grid ${SRCS})
target_link_libraries(test_grid ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
