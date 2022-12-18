/*
 * @Description:
 * @Version: 2.0
 * @Author: Virtual虚函数
 * @Date: 2022-12-17 19:36:21
 * @LastEditors: Virtual虚函数
 * @LastEditTime: 2022-12-18 11:33:15
 */
#include "../include/data_preprocess.h"
template class DataProcess<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
template <typename T>
std::tuple<T, T> DataProcess<T>::getKeyFrame(const T &input_frame)
{
    count_time_++;
    T frame_1(new pcl::PointCloud<pcl::PointXYZ>);
    T frame_2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*input_frame, *frame_1);
    if (count_time_ < KEYFRAME)
    {
        data_vector_.push_back(input_frame);
    }
    if (count_time_ > KEYFRAME)
    {
        std::tuple<T, T> key_frame(data_vector_[0], data_vector_[KEYFRAME-2]);
        data_vector_.clear();
        count_time_ = 0;
        return key_frame;

    }
    else{
        std::tuple<T, T> key_frame(frame_1, frame_2);
        return key_frame;
    }
}