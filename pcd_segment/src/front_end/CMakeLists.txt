cmake_minimum_required(VERSION 3.0.2)
project(local-builder)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# 如果不需要点云显示功能 把此行注销
#add_definitions(-Dviewer_switch)

add_definitions(${PCL_DEFINITIONS})
add_library(local-builder local_builder.cpp)
target_link_libraries(local-builder   
${catkin_LIBRARIES}
${CERES_LIBRARIES}
${PCL_LIBRARIES})

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) #设置库文件的输出目录