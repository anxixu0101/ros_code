/**
 * @file grid_map.h
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

//此类负责接收雷达的话题，并将雷达的每一帧数据转化成栅格地图格式
class GridMap
{
public:
    struct GridIndex //栅格序号
    {
        int x;
        int y;

        void setIndex(int x_, int y_)
        {
            x = x_;
            y = y_;
        }
    };

public:
    GridMap();
    void static chatterCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void createOccupancy(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::vector<GridIndex> traceLine(int x_start, int y_start,
                                     int x_end, int y_end);

private:
    ros::NodeHandle nh_; //创建句柄节点
    ros::Subscriber sub_;
};
