/*
 * @Description:
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-12-18 11:57:06
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-12-18 12:31:15
 */
#ifndef LOCAL_BUILDER_HPP_
#define LOCAL_BUILDER_HPP_

#include <memory>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
template <typename T>
class LocalBuilder
{
public:
    LocalBuilder() {}
    ~LocalBuilder() {}
    void icpScanToScan(T &input_cloud, T &target_cloud,T &output_cloud,double max_distance,int iteration);
private:
    
};

#endif
