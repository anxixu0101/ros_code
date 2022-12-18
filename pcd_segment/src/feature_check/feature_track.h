/*
 * @Description:
 * @Version: 1.0
 * @Autor: David.an
 * @Date: 2022-07-15 17:44:22
 * @LastEditors: Virtual虚函数
 */

#ifndef FEATURE_TRACK_
#define FEATURE_TRACK_
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // 拟合直线
#include <pcl/visualization/pcl_visualizer.h>
template <typename feature_T=pcl::PointCloud<pcl::PointXYZ>::Ptr>
class FeatureTrack
{
public:
    struct PointIndex
    {
        double data;
        int index;
    };

public:
    //直线提取
    void lineTrack(feature_T& cloud);
    //区域分割
    feature_T regionSegmentation(const feature_T& laser_point,
                            double max_distance);
    //拐点提取
    feature_T inflectionPointTrack(const feature_T&& laser_point);

    //点到直线距离公式
    double getDistance(double x, double y, double a, double b, double c);

private:
    int order; //拟合多项式的阶数
};

#endif