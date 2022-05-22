/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-03 12:59:59
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>
#include "Sensor/lidar_data_type.h"

namespace Algorithm
{
    using Slam3D::FeaturePointCloudContainer;

    template<typename _PointType>
    class RegistrationBase
    {
        public:
            using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;  
            using SourceInput = std::pair<std::string, PointCloudConstPtr>;     
        public:
            virtual void SetInputSource(SourceInput const& source_input) = 0;  
            virtual void SetInputTarget(FeaturePointCloudContainer<_PointType> const& target_input)  = 0;  
            virtual void Solve(Eigen::Isometry3d &T) = 0; 
    }; // class LineSurfFeatureRegistration 
} // namespace Algorithm
