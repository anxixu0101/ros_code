/*
 * @Description: 点云转栅格的基本程序
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-08-06 14:59:01
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-08-13 11:33:54
 */

#include "../include/grid_map.h"
#include <type_traits>

GridMap *GridMap::grid_;
GridMap::GridMap()
{
    ROS_INFO("Start to Listen Laser");
    grid_ = this;

    sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &GridMap::chatterCallback);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("laser_map", 1, true);
}

void GridMap::chatterCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    middleFunction(msg, grid_);
}

void GridMap::middleFunction(const sensor_msgs::LaserScan::ConstPtr &msg, GridMap *grid)
{
    grid->createOccupancy(msg);
}
void GridMap::createOccupancy(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    MapInitialize();
    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        double laser_x = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);
        double laser_y = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment);

        if (laser_x > 0 && laser_y > 0 && laser_x < 50 && laser_y < 50)
        {

            GridIndex map_index = convertWorldToGridIndex(laser_x, laser_y);

            std::vector<GridMap::GridIndex> free_index =
                traceLine( grid_map_param_.offset_x,  grid_map_param_.offset_y, map_index.x, map_index.y); //找到空闲栅格

            for (int k = 0; k < free_index.size(); k++)
            {
                GridIndex tmpIndex = free_index[k];
                //将空闲栅格的栅格序号，转化到数组序号,该数组用于存储每一个栅格的数据
                int linearIndex = gridIndexToLinearIndex(tmpIndex);
                //取出该栅格代表的数据
                if (linearIndex < grid_map_param_.height * grid_map_param_.width)
                {
                    map_ptr_[linearIndex] = 0;
                }
            }
            int occupy_index = gridIndexToLinearIndex(map_index);
            int data = map_ptr_[occupy_index];
            data = 100;
            map_ptr_[occupy_index] = data;
        }
    }

    publishMap();
    deleteMap();
}

std::vector<GridMap::GridIndex> GridMap::traceLine(int x_start, int y_start,
                                                   int x_end, int y_end)
{
    GridMap::GridIndex grid_index;
    std::vector<GridIndex> grid_index_vector;

    bool steep = abs(y_end - y_start) > abs(x_end - x_start);
    if (steep)
    {
        std::swap(x_start, y_start);
        std::swap(x_end, y_end);
    }
    if (x_start > x_end)
    {
        std::swap(x_start, x_end);
        std::swap(y_start, y_end);
    }

    int delta_x = x_end - x_start;
    int delta_y = abs(y_end - y_start);
    int error = 0;
    int y_step;
    int y = y_start;

    if (y_start < y_end)
    {
        y_step = 1;
    }
    else
    {
        y_step = -1;
    }

    int point_x;
    int point_y;
    for (int x = x_start; x <= x_end; x++)
    {
        if (steep)
        {
            point_x = y;
            point_y = x;
        }
        else
        {
            point_x = x;
            point_y = y;
        }

        error += delta_y;

        if (2 * error >= delta_x)
        {
            y += y_step;
            error -= delta_x;
        }

        if (point_x == x_end && point_y == y_end)
            continue;

        grid_index.setIndex(point_x, point_y);

        grid_index_vector.push_back(grid_index);
    }
    return grid_index_vector;
}

void GridMap::MapInitialize()
{
    grid_map_param_.log_occ = 1;       //击中栅格概率
    grid_map_param_.log_free = -1;     //空闲栅格概率
    grid_map_param_.resolution = 0.05; //地图分辨率
    grid_map_param_.origin_x = 0;      //地图起始点
    grid_map_param_.origin_y = 0;
    grid_map_param_.height = 300; //地图长和宽
    grid_map_param_.width = 300;
    grid_map_param_.offset_x = 0; //机器人起点
    grid_map_param_.offset_y = 0;

    map_ptr_ = new unsigned char[grid_map_param_.width * grid_map_param_.height];
    for (int i = 0; i < grid_map_param_.width * grid_map_param_.height; i++)
        map_ptr_[i] = 50; //地图概率初始化
}

GridMap::GridIndex GridMap::convertWorldToGridIndex(double x, double y)
{
    GridMap::GridIndex index;
    index.x = std::ceil((x - 0) / 0.05) + 0;
    index.y = std::ceil((y - 0) / 0.05) + 0;
    return index;
}

int GridMap::gridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * grid_map_param_.width;
}

void GridMap::publishMap()
{
    nav_msgs::OccupancyGrid ros_map;

    ros_map.info.resolution = grid_map_param_.resolution;
    ros_map.info.origin.orientation.x = 0.0;
    ros_map.info.origin.orientation.y = 0.0;
    ros_map.info.origin.orientation.z = 0.0;
    ros_map.info.origin.orientation.w = 1.0;

    ros_map.info.origin.position.x = grid_map_param_.origin_x;
    ros_map.info.origin.position.y = grid_map_param_.origin_y;
    ros_map.info.width = grid_map_param_.width;
    ros_map.info.height = grid_map_param_.height;
    ros_map.data.resize(ros_map.info.width * ros_map.info.height);

    // 0~100
    for (int i = 0; i < grid_map_param_.width * grid_map_param_.height; i++)
    {
        if (map_ptr_[i] == 50) //未知栅格
        {
            ros_map.data[i] = -1.0;
        }
        else if (map_ptr_[i] < 50) //空闲栅格
        {
            ros_map.data[i] = map_ptr_[i]; 
        }
        else if (map_ptr_[i] > 50) //击中栅格
        {
            ros_map.data[i] = map_ptr_[i];
        }
    }

    ros_map.header.stamp = ros::Time::now();
    ros_map.header.frame_id = "map";

    map_pub_.publish(ros_map);
}

void GridMap::deleteMap()
{
    if (map_ptr_ != NULL)
        delete map_ptr_;
}