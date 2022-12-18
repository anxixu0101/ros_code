/*
 * @Description: 
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-12-18 11:57:06
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-12-18 12:03:25
 */
#ifndef LOCAL_BUILDER_HPP_
#define LOCAL_BUILDER_HPP_

#include <memory>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

template<typename T = pcl::PointCloud<pcl::PointXYZ>::Ptr>
class LocalBuilder
{
    LocalBuilder(){}
    ~LocalBuilder(){}
 void icpScanToScan(T& input_cloud,T& target_cloud);

};


#endif

