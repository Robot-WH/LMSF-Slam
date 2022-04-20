/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-28 12:53:12
 * @Description: 
 * @Others: 
 */

#pragma once 
#include "filter_base.hpp"

namespace Algorithm
{
    template<typename _PointType>
    class DistanceFilter : public FilterBase<_PointType>
    {
        public:
            DistanceFilter(float distance_near_thresh = 0, float distance_far_thresh = 0)
            : distance_near_thresh_(distance_near_thresh), distance_far_thresh_(distance_far_thresh)
            {}
             // 距离滤波
            virtual PointCloudPtr<_PointType> Filter(
                    const PointCloudConstPtr<_PointType> &cloud_in) const override
            {
                if (distance_near_thresh_ == 0 && distance_far_thresh_ == 0)
                {
                    return PointCloudPtr<_PointType>(
                        new pcl::PointCloud<_PointType>(*cloud_in)); // 直接返回原点云
                }
                PointCloudPtr<_PointType> cloud_out(new pcl::PointCloud<_PointType>());  
                // 距离滤波
                std::copy_if(cloud_in->begin(), cloud_in->end(), std::back_inserter(cloud_out->points),
                            [&](const _PointType &p) { // 外部变量按引用传递
                                double d = p.getVector3fMap().norm();
                                if (d > distance_near_thresh_ && d < distance_far_thresh_)
                                {
                                    return true;
                                }
                                else
                                    return false;
                            });
                return cloud_out;
            }
        private:
            float distance_near_thresh_;
            float distance_far_thresh_;  
    };
}