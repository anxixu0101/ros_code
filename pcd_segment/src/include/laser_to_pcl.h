/*
 * @Description: 
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-10-16 08:43:24
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-12-18 13:23:27
 */


#ifndef LASER_TO_
#define LASER_TO_
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../feature_check/feature_track.h" 
#include "../front_end/local_builder.hpp"
#include "data_preprocess.h"
#include <memory>
struct LaserData
{
   double x;
   double y;
   double angle;

} ;

class My_Filter {
     public:
        My_Filter();
        //订阅 LaserScan　数据，并发布 PointCloud2 点云 
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        //订阅 LaserScan 数据，先转换为 PointCloud2，再转换为 pcl::PointCloud
        void scanCallback_2(const sensor_msgs::LaserScan::ConstPtr& scan);

        //直接订阅 PointCloud2 然后自动转换为 pcl::PointCloud
        void pclCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_vector;
        int number=0;
        pcl::PointCloud<pcl::PointXYZ> cloud_out; 
        //发布　"PointCloud2"
        ros::Publisher point_cloud_publisher_;

        //订阅 "/scan"
        ros::Subscriber scan_sub_;

        //订阅 "/cloud2" -> "PointCloud2"
        ros::Subscriber pclCloud_sub_;

   private:
       std::unique_ptr<FeatureTrack<pcl::PointCloud<pcl::PointXYZ>::Ptr>> feature_;
       std::unique_ptr<DataProcess<pcl::PointCloud<pcl::PointXYZ>::Ptr>> data_process_;
       std::unique_ptr<LocalBuilder<pcl::PointCloud<pcl::PointXYZ>::Ptr>> local_builder_;
};
#endif