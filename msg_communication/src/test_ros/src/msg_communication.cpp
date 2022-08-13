/*
 * @Description: 利用异步高并法同时实现ros下的雷达消息订阅以及pointcloud类型
 * 的发布
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-07-31 12:13:32
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-08-13 11:35:14
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>
#include <memory>
#include <future>
class LaserSub
{
public:
    LaserSub()
    {
        ROS_INFO("Start to Listen Laser");
        sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &LaserSub::chatterCallback);
    }
    static void chatterCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        ROS_INFO("Laser have been heared!"); //将接收到的消息打印出来
    }

private:
    ros::NodeHandle nh_; //创建句柄节点
    ros::Subscriber sub_;
};

class LaserPub
{
public:
    LaserPub()
    {
        ROS_INFO("Start to Publish Laser");
        pub_ = nh_.advertise<sensor_msgs::PointCloud>("/point", 1000);
        
    }
    void chatterCallback(int number)
    {
        while (ros::ok())
        {

            sensor_msgs::PointCloud point;

            pub_.publish(point);
            ROS_INFO("Point has been Published!");
            ros::Rate loop_rate(10);
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_; //创建句柄节点
    ros::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher"); //初始化ROS节点
    LaserPub laser_pub;
    std::shared_ptr<LaserSub> laser_sub = std::make_shared<LaserSub>();
    std::future<void> result = std::async(std::launch::async, &LaserPub::chatterCallback, &laser_pub, 1);
    ros::Rate loop_rate(10);
    ros::spin();

    return 0;
}
