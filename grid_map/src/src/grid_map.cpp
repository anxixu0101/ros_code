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