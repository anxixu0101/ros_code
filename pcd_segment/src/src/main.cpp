/*
 * @Description: 
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-10-16 08:43:24
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-10-16 08:55:34
 */

#include "laser_to_pcl.h"

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "gridMap");
 
  My_Filter l;
  
  ros::spin();
  return 0;

}