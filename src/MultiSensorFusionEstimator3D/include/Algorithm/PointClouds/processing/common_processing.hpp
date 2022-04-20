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

#include "process_base.hpp"
#include "factory/processing/pointcloud/filter/filter_factory.hpp"
#include "Filter/voxel_grid.hpp"
#include "Filter/distance_filter.hpp"
#include "Filter/outlier_removal.hpp"

namespace Algorithm
{
    using Slam3D::LidarData; 
    using Slam3D::FeatureInfo;  

    template <typename _PointType>
    using PointCloudType = pcl::PointCloud<_PointType>;
    template <typename _PointType>
    using PointCloudPtr = typename pcl::PointCloud<_PointType>::Ptr;   // boost 库  
    template <typename _PointType>
    using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;  // boost 库 

    /**
     * @brief: 针对PCL 通用的点云处理流程 
     * @details 去除NaN点 -> 降采样 -> 去离群点 -> 距离滤波  
     * @param _PointType pcl 点云的点类型 
     * @param PointCloudType<_PointType> 输入点云数据类型
     * @param PointCloudPtr<_PointType> 输出点云数据类型   
     */
    template <typename _PointType>
    class PointCloudCommonProcess : public PointCloudProcessBase<_PointType, _PointType>
    {
        public:
            using Base = PointCloudProcessBase<_PointType, _PointType>;  
            PointCloudCommonProcess(string const& output_name, bool removal_nan = false) 
            : output_name_(output_name), removal_nan_(removal_nan) {}

            /**
             * @brief:  设置降采样滤波  
             * @details 只用通过该函数设置才能正常使用滤波器  
             * @param name 名称 : VoxelGrid: 重心     ApproximateVoxelGrid：近似中心点  
             * @param params 设置参数  
             */        
            template <typename... _ParamType>
            void SetVoxelGrid(string const &name, _ParamType... params)
            {
                downsample_filter_.Reset(name, params...); 
            }

            /**
             * @brief:  设置离群点滤波  
             * @details 只用通过该函数设置才能正常使用滤波器  
             * @param name 识别名称 ， radiusOutlierRemoval: 几何距离    statisticalOutlierRemoval：统计法   
             * @param params 设置参数  
             */        
            template <typename... _ParamType>
            void SetOutlierRemoval(string const &name, _ParamType... params)
            {
                outlier_removal_.Reset(name, params...);
            }
            /**
             * @brief:  设置距离滤波  
             * @details 只用通过该函数设置才能正常使用滤波器  
             * @param distance_near_thresh 最近距离
             * @param distance_far_thresh 最远距离 
             */        
            void SetDistanceFilter(float const &distance_near_thresh, float const &distance_far_thresh)
            {
                distance_filter_ = DistanceFilter<_PointType>(distance_near_thresh, distance_far_thresh); 
            }

            /**
             * @brief:  滤波处理函数 
             * @details 本函数的处理内容为 1、去除NaN  2、降采样  3、离群点去除  4、距离滤波  
             * @param[in] data_in 输入点云 
             * @param[out] data_out 滤波后的结果  
             */        
            void Process(LidarData<_PointType> const& data_in, 
                FeatureInfo<_PointType> &data_out) override
            {
                typename pcl::PointCloud<_PointType>::Ptr point_cloud(
                    new pcl::PointCloud<_PointType>());
                // 如果进行NaN点滤除  
                if (removal_nan_)
                {
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(data_in.point_cloud, *point_cloud, indices);
                }
                else
                {
                    *point_cloud = data_in.point_cloud;  
                }
                //std::cout<<"before filter "<<point_cloud->size()<<std::endl;
                // 降采样
                point_cloud = downsample_filter_.Filter(point_cloud);  
                // 离群点滤波
                point_cloud = outlier_removal_.Filter(point_cloud);  
                // 距离滤波
                point_cloud = distance_filter_.Filter(point_cloud);
                data_out.pointcloud_data_.insert(make_pair(output_name_, point_cloud)); 
                //std::cout<<"after filter "<<point_cloud->size()<<" output_name_: "<<output_name_<<std::endl;
                return;
            }

        private:
            bool removal_nan_;
            float distance_near_thresh_ = 0;
            float distance_far_thresh_ = 0;
            string output_name_;  
            VoxelGridFilter<_PointType> downsample_filter_;      // 降采样滤波器对象
            DistanceFilter<_PointType> distance_filter_; 
            OutLierRemovalFilter<_PointType> outlier_removal_; 
    }; // class PclCommonProcessing
};
#endif
