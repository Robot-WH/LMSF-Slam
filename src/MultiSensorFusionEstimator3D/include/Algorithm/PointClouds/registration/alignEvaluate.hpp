/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-05-08 17:49:17
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Slam3D
{
    /**
     * @brief: 点评估方法
     * @details: 
     */    
    template<typename _PointT>
    class PointCloudAlignmentEvaluate
    {
        public:
            using Ptr = std::shared_ptr<PointCloudAlignmentEvaluate<_PointT>>;  
            PointCloudAlignmentEvaluate() : search_tree_(new pcl::KdTreeFLANN<_PointT>())
            {
            }

            void SetTargetPoints(typename pcl::PointCloud<_PointT>::ConstPtr const& cloud)
            {
                set_target_ = true;  
                search_tree_->setInputCloud(cloud);
            }

            /**
             * @brief: 计算对齐得分  
             * @details: 
             * @param inlier_thresh 内点的距离阈值
             * @param inlier_ratio_thresh 
             * @return 匹配得分  平方距离的均值   
             */            
            double AlignmentScore(typename pcl::PointCloud<_PointT>::ConstPtr const& cloud, 
                                                                Eigen::Matrix4f const& relpose, 
                                                                double const& inlier_thresh,
                                                                double const& inlier_ratio_thresh) 
            {
                assert(set_target_ == true); 
                if (cloud->empty()) return (std::numeric_limits<double>::max());
                double fitness_score = 0.0;
                pcl::PointCloud<_PointT> input_transformed;
                // cloud 通过  relpose 转到  input_transformed  
                pcl::transformPointCloud (*cloud, input_transformed, relpose);
                std::vector<int> nn_indices (1);
                std::vector<float> nn_dists (1);
                int nr = 0;

                for (size_t i = 0; i < input_transformed.points.size(); ++i)
                {
                    // Find its nearest neighbor in the target
                    search_tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);
                    // Deal with occlusions (incomplete targets)    如果最近的点小于阈值
                    if (nn_dists[0] <= inlier_thresh)    
                    {
                        fitness_score += nn_dists[0];   // 只考虑距离合适的点   这样可以避免动态障碍的影响 
                        nr++;
                    }
                }
                // 计算好点的占比   
                overlap_ratio_ = (double) nr / input_transformed.points.size();
                std::cout<<"overlap_ratio_: "<<overlap_ratio_<<std::endl;
                // 如果内点数量不足        
                if (overlap_ratio_ > inlier_ratio_thresh)    
                    return (fitness_score / nr);          // 距离合适的点的平均距离   
                else
                    return (std::numeric_limits<double>::max());
            }

        protected:
            bool set_target_ = false;  
            double overlap_ratio_ = 0.0;    // 匹配的重合度 
            typename pcl::KdTreeFLANN<_PointT>::Ptr search_tree_;  

    }; // class 
} // namespace 

