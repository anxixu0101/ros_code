
/**
 * @file data_preprocess.h
 * @author Virtual虚函数
 * @brief  雷达数据预处理
 * @version 0.1
 * @date 2022-12-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef DATA_PROCESS_
#define DATA_PROCESS_
#include <vector>
#include <iostream>
#include <tuple>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

const int KEYFRAME = 50;

template<typename T = pcl::PointCloud<pcl::PointXYZ>::Ptr>
class DataProcess
{
  public:
  DataProcess(){}
 ~DataProcess(){}

  public:
  std::tuple<T,T> getKeyFrame(const T& input_frame); //提取关键帧

  private:
  int count_time_ = 0;
  std::vector<T> data_vector_;

};

#endif
