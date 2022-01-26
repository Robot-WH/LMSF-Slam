/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-24 16:31:41
 * @Description: 
 * @Others: 
 */

#ifndef _COMMON_PROCESSING_HPP_
#define _COMMON_PROCESSING_HPP_

#include "processing_base.hpp"
#include "factory/processing/pointcloud/filter/filter_factory.hpp"

namespace Algorithm {

    template<typename _PointType>
    using PclType = pcl::PointCloud<_PointType>;
    template<typename _PointType>
    using PclPtr = typename pcl::PointCloud<_PointType>::Ptr;
    template<typename _PointType>
    using PclConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;

    /**
     * @brief: 通用的点云处理方法
     * @param _PointType pcl 点云的点类型 
     */    
    template<typename _PointType>
    class PclCommonProcessing : public PointCloudProcessingBase<PclType<_PointType>, PclPtr<_PointType>> 
    {
        public:
            PclCommonProcessing() = delete;  
            PclCommonProcessing(bool removal_nan = false) : removal_nan_(removal_nan),
                                                                                                                    downsample_filter_(nullptr), 
                                                                                                                    outlier_removal_filter_(nullptr) {}

            template<typename... _ParamType>
            void SetVoxelGrid(string const& name, _ParamType... params) 
            {
                if (name == "VoxelGrid") 
                {
                    downsample_filter_ = Factory::make_voxelGrid<_PointType>(params...);  
                } 
                else if (name == "ApproximateVoxelGrid") 
                {
                }
            }

            template<typename... _ParamType>
            void SetOutlierRemoval(string const& name, _ParamType... params) 
            {
                if (name == "radiusOutlierRemoval") 
                {
                    outlier_removal_filter_ = Factory::make_radiusOutlierRemoval<_PointType>(params...); 
                } 
                else if (name == "statisticalOutlierRemoval") 
                {
                    outlier_removal_filter_ = Factory::make_statisticalOutlierRemoval<_PointType>(params...); 
                }
            }

            void SetDistanceFilter(float const& distance_near_thresh, float const& distance_far_thresh) 
            {
                distance_near_thresh_ = distance_near_thresh;
                distance_far_thresh_ = distance_far_thresh;
            }

            void Processing(PclType<_PointType> const& data_in, PclPtr<_PointType> &data_out) override 
            {
                if (removal_nan_) 
                {
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(data_in, *data_out, indices);
                }
                // 降采样
                data_out = downSample(data_out); 
                // 离群点滤波
                data_out = outlierRemoval(data_out); 
                // 距离滤波 
                data_out = distanceFilter(data_out); 
                return;  
            } 

            // 滤波
            PclPtr<_PointType> Filter(typename pcl::Filter<_PointType>::Ptr filter_ptr,
                                                    const PclConstPtr<_PointType>& cloud) const 
            {
                PclPtr<_PointType> filtered(new PclType<_PointType>(*cloud));
                if (filter_ptr == nullptr) return filtered;   // 如果 滤波器 未设置 直接将原点云返回出来  
                filter_ptr->setInputCloud(cloud);
                filter_ptr->filter(*filtered);
                filtered->header = cloud->header;
                return filtered;
            }

        protected:
            // 降采样滤波
            PclPtr<_PointType> downSample(const PclConstPtr<_PointType>& cloud) const 
            {
                PclPtr<_PointType> filtered(new PclType<_PointType>(*cloud));
                if (downsample_filter_ == nullptr) return filtered;   // 如果 滤波器 未设置 直接将原点云返回出来  
                downsample_filter_->setInputCloud(cloud);
                downsample_filter_->filter(*filtered);
                filtered->header = cloud->header;
                return filtered;
            }

            // 离群点去除
            PclPtr<_PointType> outlierRemoval(const PclConstPtr<_PointType>& cloud) const 
            {
                PclPtr<_PointType> filtered(new PclType<_PointType>(*cloud));
                if (outlier_removal_filter_ == nullptr) return filtered;   // 如果 滤波器 未设置 直接将原点云返回出来  
                outlier_removal_filter_->setInputCloud(cloud);
                outlier_removal_filter_->filter(*filtered);
                filtered->header = cloud->header;
                return filtered;
            }

            // 距离滤波 
            PclPtr<_PointType> distanceFilter(const PclConstPtr<_PointType>& cloud) const 
            {
                PclPtr<_PointType> filtered(new PclType<_PointType>(*cloud));
                if (distance_near_thresh_ == 0 && distance_far_thresh_ == 0) 
                {
                    return filtered;       // 直接返回原点云 
                } 
                // 距离滤波
                std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
                [&](const _PointType& p) 
                {           // 外部变量按引用传递   
                    double d = p.getVector3fMap().norm();
                    if(d > distance_near_thresh_ && d < distance_far_thresh_) 
                    {   
                        return true;
                    }
                    else return false;
                });
                return filtered;  
            }

        protected:
            bool removal_nan_;   // 是否去除NaN
            float distance_near_thresh_ = 0;
            float distance_far_thresh_ = 0;
            typename pcl::Filter<_PointType>::Ptr downsample_filter_;         // 降采样滤波器对象
            typename pcl::Filter<_PointType>::Ptr outlier_removal_filter_;    // 离群点滤波对象
    }; // class PclCommonProcessing
};
#endif
