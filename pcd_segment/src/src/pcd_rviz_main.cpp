/*
 * @Description: 
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-10-16 08:43:24
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-10-16 08:57:53
 */

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
//which contains the required definitions to load and store point clouds to PCD and other file formats.
 
main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output1", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile ("2.pcd", cloud);//更换为自己的pcd文件路径
  //Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}





