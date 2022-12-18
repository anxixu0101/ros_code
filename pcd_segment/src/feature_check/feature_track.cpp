
#include <iostream>
#include <vector>
#include "feature_track.h"
#include <math.h>
using namespace std;

template class FeatureTrack <pcl::PointCloud<pcl::PointXYZ>::Ptr>;

//直线分割
template <typename feature_T>
void FeatureTrack<feature_T>::lineTrack(feature_T& cloud)
{
	cout << "点云点数为：" << cloud->points.size() << endl;
	//拟合直线
	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
	ransac.setDistanceThreshold(0.02); //内点到模型的最大距离
	ransac.setMaxIterations(100);	   //最大迭代次数
	ransac.computeModel();			   //直线拟合
	//根据索引提取内点
	vector<int> inliers;
	ransac.getInliers(inliers);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_line);

	//输出模型参数
	Eigen::VectorXf coef;
	ransac.getModelCoefficients(coef);
	cout << "直线方程为：\n"
		 << "   (x - " << coef[0] << ") / " << coef[3]
		 << " = (y - " << coef[1] << ") / " << coef[4]
		 << " = (z - " << coef[2] << ") / " << coef[5] << endl;
	//结果可视化
	#ifdef viewer_switch
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	viewer->addPointCloud<pcl::PointXYZ>(cloud_line, "fitline");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "fitline");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fitline");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	#endif
}

bool myfunction(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> i,
				std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> j)
{
	return (i[0]->points.size() > j[0]->points.size());
}


bool sort_point_index(FeatureTrack<pcl::PointCloud<pcl::PointXYZ>::Ptr>::PointIndex i,
                      FeatureTrack<pcl::PointCloud<pcl::PointXYZ>::Ptr>::PointIndex j)
{ return(i.data>j.data);}

//区域分割
template <typename feature_T>
feature_T FeatureTrack<feature_T>::regionSegmentation(const feature_T& laser_point,
									  double max_distance)
{
	std::cout<<"开始执行区域分割！！"<<std::endl;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> laser_area;
	int start_point = 0;
	for (auto i = 0; i < laser_point->points.size() - 1; i++)
	{

		double x_1 = laser_point->points[i].x;
		double y_1 = laser_point->points[i].y;
		double x_2 = laser_point->points[i + 1].x;
		double y_2 = laser_point->points[i + 1].y;
		double distance = sqrt(pow((x_1 - x_2), 2) + pow((y_1 - y_2), 2));

		if (distance > max_distance)
		{
			int end_point = i;
			pcl::PointCloud<pcl::PointXYZ>::Ptr point_area(new pcl::PointCloud<pcl::PointXYZ>);
			point_area->width = 1;
			point_area->height = end_point - start_point;
			point_area->resize(end_point - start_point);

			for (auto n = start_point; n < end_point; n++)
			{
				point_area->points[n - start_point].x = laser_point->points[n].x;
				point_area->points[n - start_point].y = laser_point->points[n].y;
			}
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_vector;
			point_vector.push_back(point_area);
			laser_area.push_back(point_vector);
			point_vector.clear();
			start_point = i + 1;
		}
	}
	std::sort(laser_area.begin(), laser_area.end(), myfunction);

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr area_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr area_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr area_3(new pcl::PointCloud<pcl::PointXYZ>);
	//取3个点云数量最多的连通区域来代表这帧点云
	if (laser_area.size() > 3)
	{
        pcl::PointCloud<pcl::PointXYZ>::Ptr after_segmentation(new pcl::PointCloud<pcl::PointXYZ>);
		area_1 = laser_area[0][0];
		area_2 = laser_area[1][0];
		area_3 = laser_area[2][0];
		*after_segmentation = *area_1+*area_2+*area_3;
		pcl::PointCloud<pcl::PointXYZ>::Ptr inflection_point(new pcl::PointCloud<pcl::PointXYZ>);
		inflection_point=inflectionPointTrack(std::move(area_2));
		#ifdef viewer_switch
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->addPointCloud<pcl::PointXYZ>(laser_point, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

		viewer->addPointCloud<pcl::PointXYZ>(area_1, "area_1");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "area_1");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "area_1");

		viewer->addPointCloud<pcl::PointXYZ>(area_2, "area_2");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "area_2");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "area_2");

		// viewer->addPointCloud<pcl::PointXYZ>(area_3, "area_3");
		// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "area_3");
		// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "area_3");

        viewer->addPointCloud<pcl::PointXYZ>(inflection_point, "point");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "point");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
       #endif
	    
		laser_area.clear();
		return after_segmentation;
	}
	else
	{
		std::cout<<"点云较为稠密，间断点较少"<<std::endl;
		return laser_point;
		
	}
}

//拐点检测
template <typename feature_T>
feature_T FeatureTrack<feature_T>::inflectionPointTrack(const feature_T&&
											laser_point)
{
	//首点和尾点连接构成直线
	double x_first=laser_point->points[0].x;
	double y_first=laser_point->points[0].y;
	double x_end=laser_point->points[laser_point->points.size()-1].x;
	double y_end=laser_point->points[laser_point->points.size()-1].y;
	double k=(y_end-y_first)/(x_end-x_first);
	double b=y_first-k*x_first;

	double A=k;
	double B=-1;
	double C=b;
	
	std::vector<FeatureTrack::PointIndex> distance_vector;
	
	for(auto i=0;i<laser_point->points.size();i++)
	{
	 
	 FeatureTrack::PointIndex point_index;
	 double x=laser_point->points[i].x;
	 double y=laser_point->points[i].y;
	 double distance=FeatureTrack::getDistance(x,y,A,B,C);
	 point_index.data=distance;
	 point_index.index=i;
     distance_vector.push_back(point_index);
	}
	
	std::sort(distance_vector.begin(),distance_vector.end(),sort_point_index);
	int point_index=distance_vector[0].index;//拐点序号

    //显示拐点
	pcl::PointCloud<pcl::PointXYZ>::Ptr inflection_point(new pcl::PointCloud<pcl::PointXYZ>);
	inflection_point->width=1;
	inflection_point->height=1;
	inflection_point->resize(1);
	inflection_point->points[0].x=laser_point->points[point_index].x;
	inflection_point->points[0].y=laser_point->points[point_index].y;
    distance_vector.clear();
    return inflection_point;
	
}

//点到直线距离
template <typename feature_T>
 double FeatureTrack<feature_T>::getDistance(double x,double y,double a,double b,double c)
 {
	 double distance=abs(a*x+b*y+c)/sqrt(a*a+b*b);
	 return distance;
 }