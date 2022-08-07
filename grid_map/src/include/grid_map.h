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
        double log_occ ;       //击中栅格概率
        double log_free ;     //空闲栅格概率
        double resolution ; //地图分辨率
        double origin_x  ;      //地图起始点
        double origin_y ;
        int height ; //地图长和宽
        int width ;
        int offset_x; //机器人起点
        int offset_y; 
    };

public:
    GridMap();
    void static chatterCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void static middleFunction(const sensor_msgs::LaserScan::ConstPtr &msg,GridMap *grid);
    void createOccupancy(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::vector<GridIndex> traceLine(int x_start, int y_start,
                                     int x_end, int y_end); 

    void MapInitialize();//地图初始化
    GridIndex convertWorldToGridIndex(double x,double y); //世界坐标系转栅格坐标系
    int gridIndexToLinearIndex(GridIndex index);
    void publishMap();
    void deleteMap();

private:
    ros::NodeHandle nh_; //创建句柄节点
    ros::Subscriber sub_;
    ros::Publisher map_pub_;
    unsigned char* map_ptr_; //声明地图指针
    GridMap::MapParams grid_map_param_; //声明地图对象
    static GridMap* grid_;
    
};
