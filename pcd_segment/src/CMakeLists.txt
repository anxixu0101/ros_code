cmake_minimum_required(VERSION 3.0.2)
project(tomato-slam)
SET(CMAKE_BUILD_TYPE “Debug”)


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nav_msgs
  visualization_msgs
  tf
  message_filters
  laser_geometry
  message_generation
  sensor_msgs
  std_srvs)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)




# Add:特征识别库
set(FEATURE_LIB ${CMAKE_CURRENT_SOURCE_DIR}/feature_check/lib)
ADD_SUBDIRECTORY(feature_check)
link_directories(${FEATURE_LIB})



include_directories(include feature_check)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${FEATURE_LIB}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar-check
  src/main.cpp
  src/laser_to_pcl.cpp
)
target_link_libraries(lidar-check
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  libfeature_check.so
)



## pcd测试用例
# add_executable(pcd_test src/pcd_rviz_main.cpp
# )
# target_link_libraries(pcd_test
#   ${catkin_LIBRARIES}
#   ${CERES_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
