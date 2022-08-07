/**
 * @file occupancy_grid_publish.cpp
 * @author anxixu
 * @brief
 * @version 0.1
 * @date 2022-08-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "../include/grid_map.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "gridMap");

    GridMap grid_map;
    //   ros::NodeHandle nh;
    //   ros::NodeHandle nh_private("~");

    //   ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
    //   nav_msgs::OccupancyGrid map;

    //   map.header.frame_id="grid";
    //   map.header.stamp = ros::Time::now();
    //   map.info.resolution = 0.5;         // float32
    //   map.info.width      = 20;           // uint32
    //   map.info.height     = 20;           // uint32

    //   int p[map.info.width*map.info.height] = {-1};   // [0,100]
    //   p[10] = 100;
    //   std::vector<signed char> a(p, p+400);
    //   map.data = a;

    //   while (ros::ok())
    //   {
    //       pub.publish(map);
    //   }

    ros::Rate loop_rate(10);
    ros::spin();
    //   return 0;
}
