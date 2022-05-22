/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-04 20:04:20
 * @Description: 
 * @Others: 
 */

#pragma once 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>

namespace Algorithm
{

    template<typename _PointT>
    class FeatureMatch
    {
        protected:
            using PointCloudConstPtr = typename pcl::PointCloud<_PointT>::ConstPtr;  
            //kd-tree
            typename pcl::KdTreeFLANN<_PointT>::Ptr search_tree_;
            typename pcl::PointCloud<_PointT>::ConstPtr source_points_;
            float search_thresh_ = 1.0;    // 最远的特征点搜索范围
        
        public:
            
            FeatureMatch() : source_points_(nullptr)
            {
                search_tree_ = typename pcl::KdTreeFLANN<_PointT>::Ptr(
                    new pcl::KdTreeFLANN<_PointT>());
            }
            virtual ~FeatureMatch() {}

            virtual void SetSearchTarget(PointCloudConstPtr const& source_ptr)
            {
                search_tree_->setInputCloud(source_ptr);
                source_points_ = source_ptr;
            } 
    };
}
