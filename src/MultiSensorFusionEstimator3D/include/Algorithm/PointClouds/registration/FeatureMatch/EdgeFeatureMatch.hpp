/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-04 15:11:51
 * @Description: 
 * @Others: 
 */
#pragma once 

#include "FeatureMatchBase.hpp"

namespace Algorithm
{

    template<typename _PointT>
    class EdgeFeatureMatch : public FeatureMatch<_PointT>
    {
        public:
            // 特征匹配后的信息    
            struct EdgeCostFactorInfo
            {
                Eigen::Vector3d norm_ = {0, 0, 0};
                double residuals_ = 0;
                double s_ = 0;     // 权重
                std::vector<Eigen::Vector3d> points_set_;  
            }; 
        private:
            using base = FeatureMatch<_PointT>;  
        public:

            bool Match(_PointT const& point, EdgeCostFactorInfo &res) 
            {
                if (base::source_points_ == nullptr) return false;  
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                base::search_tree_->nearestKSearch(point, 5, pointSearchInd, pointSearchSqDis); 

                if (pointSearchSqDis[4] < base::search_thresh_)
                {
                    // 进行PCA
                    std::vector<Eigen::Vector3d> nearCorners;
                    Eigen::Vector3d center(0, 0, 0);

                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Vector3d tmp(base::source_points_->points[pointSearchInd[j]].x,
                                            base::source_points_->points[pointSearchInd[j]].y,
                                            base::source_points_->points[pointSearchInd[j]].z);
                        center = center + tmp;
                        nearCorners.push_back(tmp);
                    }
                    center = center / 5.0;
                    // 计算协方差矩阵
                    Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                    }
                    // 特征分解 
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                    Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                    // 计算线性度  
                    // 要足够像一条线   
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    { 
                        Eigen::Vector3d point_on_line = center;
                        Eigen::Vector3d point_a, point_b;
                        point_a = 0.1 * unit_direction + point_on_line;
                        point_b = -0.1 * unit_direction + point_on_line;
                        Eigen::Vector3d curr_point(point.x, point.y, point.z);
                        // 计算匹配的直线方程  
                        Eigen::Vector3d nu = (curr_point - point_a).cross(curr_point - point_b);   // 平行四边形的面积 
                        Eigen::Vector3d de = point_a - point_b;  
                        double de_norm = de.norm();
                        res.residuals_ = nu.norm() / de_norm;    // 点到线的距离    
                        res.norm_ = de.cross(nu).normalized();  // 残差的法向量 
                        res.points_set_.push_back(point_a);
                        res.points_set_.push_back(point_b);  
                        return true;
                    }                                                                           
                }
                return false;          
            }
    };
}
