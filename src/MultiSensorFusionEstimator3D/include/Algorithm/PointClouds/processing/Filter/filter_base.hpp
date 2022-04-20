/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-28 12:01:51
 * @Description: 
 * @Others: 
 */
#pragma once 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace Algorithm
{
    template <typename _PointType>
    using PointCloudPtr = typename pcl::PointCloud<_PointType>::Ptr;  
    template <typename _PointType>
    using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr; 

    template<typename _PointType>
    class FilterBase
    {
        public:
            FilterBase() : filter_ptr_(nullptr) {}
            virtual ~FilterBase() {}
            /**
             * @brief:  滤波流程
             * @param[in] cloud_in 输入的点云 
             * @param[out] cloud_out 处理后的点云 
             */        
            virtual PointCloudPtr<_PointType> Filter(
                const PointCloudConstPtr<_PointType> &cloud_in) const
            {
                PointCloudPtr<_PointType> cloud_out(
                    new pcl::PointCloud<_PointType>(*cloud_in));  
                if (filter_ptr_ == nullptr)
                    return cloud_out; 
                filter_ptr_->setInputCloud(cloud_in);
                filter_ptr_->filter(*cloud_out);
                cloud_out->header = cloud_in->header;
                return cloud_out;  
            }

            void SetFilter(typename pcl::Filter<_PointType>::Ptr const& filter_ptr)
            {
                filter_ptr_ = filter_ptr;   
            }
            
        private:
            typename pcl::Filter<_PointType>::Ptr filter_ptr_; 
    }; // class FilterBase

}
