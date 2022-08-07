cmake_minimum_required(VERSION 3.0.2)
project(test_ros)
set( CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_definitions(-std=c++11) #该处可以根据自己需要进行修改，例如c++14...
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
