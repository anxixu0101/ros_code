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

        void setIndex(int x_input, int y_input)
        {
            x = x_input;
            y = y_input;
        }
    };

    struct MapParams
    {
        double log_occ = 1;       //击中栅格概率
        double log_free = -1;     //空闲栅格概率
        double resolution = 0.05; //地图分辨率
        double origin_x = 0 ;      //地图起始点
        double origin_y = 0;
        int height = 500; //地图长和宽
        int width = 500;
        int offset_x=0; //机器人起点
        int offset_y=0; 
    };

public:
    GridMap();
    void static chatterCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void createOccupancy(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::vector<GridIndex> traceLine(int x_start, int y_start,
                                     int x_end, int y_end); 

    void MapInitialize();//地图初始化
    GridIndex convertWorldToGridIndex(double x,double y); //世界坐标系转栅格坐标系
    int gridIndexToLinearIndex(GridIndex index);
    void publishMap();

private:
    ros::NodeHandle nh_; //创建句柄节点
    ros::Subscriber sub_;
    ros::Publisher map_pub_;
    unsigned char* map_ptr_; //声明地图指针
    GridMap::MapParams grid_map_param_; //声明地图对象
    
};
