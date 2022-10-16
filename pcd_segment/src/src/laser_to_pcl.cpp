/*
 * @Description: 
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-10-16 08:43:24
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-10-16 09:07:26
 */

#include "laser_to_pcl.h"
#include <fstream>

My_Filter::My_Filter()
{
  //订阅　"/scan"
 // scan_sub_ = node_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &My_Filter::scanCallback_2, this);

  //发布LaserScan转换为PointCloud2的后的数据
  //point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud2", 100, false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr SB(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("2.pcd", *SB);
  feature_.lineTrack(SB);
  feature_.regionSegmentation(SB, 0.1);
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);
  point_cloud_publisher_.publish(cloud);
}


void My_Filter::scanCallback_2(const sensor_msgs::LaserScan::ConstPtr &scan)
{

  // number++;
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2 cloud_rviz;
  projector_.transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);

  int row_step = cloud.row_step;
  int height = cloud.height;

  /*将 sensor_msgs::PointCloud2 转换为　pcl::PointCloud<T> */
  //注意要用fromROSMsg函数需要引入pcl_versions（见头文件定义）
  pcl::PointCloud<pcl::PointXYZ> rawCloud;
  pcl::fromROSMsg(cloud, rawCloud);
}

void My_Filter::pclCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
  }
}