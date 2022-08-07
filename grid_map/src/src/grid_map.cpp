#include "../include/grid_map.h"

GridMap::GridMap()
{
    ROS_INFO("Start to Listen Laser");
    sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &GridMap::chatterCallback);
}

void GridMap::chatterCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
}

void GridMap::createOccupancy(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    MapInitialize();

    for (auto i = 0; i < msg->ranges.size(); i++)
    {
        double laser_x = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);
        double laser_y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);
        GridIndex map_index = convertWorldToGridIndex(laser_x, laser_y);
        std::vector<GridMap::GridIndex> free_index =
            traceLine(0, 0, map_index.x, map_index.y); //找到空闲栅格

        for (int k = 0; k < free_index.size(); k++)
        {
            GridIndex tmpIndex = free_index[k];
            //将空闲栅格的栅格序号，转化到数组序号,该数组用于存储每一个栅格的数据
            int linearIndex = gridIndexToLinearIndex(tmpIndex);
            //取出该栅格代表的数据
            int data = map_ptr_[linearIndex];
            //根据栅格空闲规则，执行data += mapParams.log_free;
            if (data > 0)                   //默认data=50
                data += grid_map_param_.log_free; // log_free=-1，data将变小
            else
                data = 0;
            //给该空闲栅格赋新值，最小为0
            map_ptr_[linearIndex] = data;
        }
    }
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
}

void GridMap::MapInitialize()
{

    map_ptr_ = new unsigned char[grid_map_param_.width * grid_map_param_.height];
    for (int i = 0; i < grid_map_param_.width * grid_map_param_.height; i++)
        map_ptr_[i] = 50; //地图概率初始化
}

GridMap::GridIndex GridMap::convertWorldToGridIndex(double x, double y)
{
    GridMap::GridIndex index;
    index.x = std::ceil((x - grid_map_param_.origin_x) / grid_map_param_.resolution) + grid_map_param_.offset_x;
    index.y = std::ceil((y - grid_map_param_.origin_y) / grid_map_param_.resolution) + grid_map_param_.offset_y;

    return index;
}

int GridMap::gridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * grid_map_param_.width;
}


void GridMap::publishMap()
{

}