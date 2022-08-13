
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "../include/grid_map.h"
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "gridMap");
    GridMap grid_map;
    ros::Rate loop_rate(10);
    ros::spin();
 
}
