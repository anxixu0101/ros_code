/*
 * @Description: 
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-12-18 11:57:17
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-12-18 12:31:41
 */
#include "local_builder.hpp"
template class LocalBuilder<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
template<typename T>

void LocalBuilder<T>::icpScanToScan(T &input_cloud, T &target_cloud,T &output_cloud,double max_distance,int iteration)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);

	tgt = target_cloud;
	src = input_cloud;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(max_distance);
	icp.setTransformationEpsilon(1e-10);
	//icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations (iteration);

	icp.setInputSource (src);
	icp.setInputTarget (tgt);
	icp.align (*output_cloud);
//	std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
		
	output_cloud->resize(tgt->size()+output_cloud->size());
	for (int i=0;i<tgt->size();i++)
	{
		output_cloud->push_back(tgt->points[i]);
	}
	std::cout<<"After registration using ICP:"<<output_cloud->size()<<std::endl;
}