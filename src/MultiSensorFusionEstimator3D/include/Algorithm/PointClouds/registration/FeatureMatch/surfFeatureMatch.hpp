/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-04 15:09:18
 * @Description: 
 * @Others: 
 */
#pragma once 

#include "FeatureMatchBase.hpp"

namespace Algorithm
{

    template<typename _PointT>
    class SurfFeatureMatch : public FeatureMatch<_PointT>
    {
        public:
            // 特征匹配后的信息    
            struct SurfCostFactorInfo
            {
                Eigen::Vector3d norm_ = {0, 0, 0};
                double D_;  
                double residuals_ = 0;
                double s_ = 0;     // 权重
            }; 
        private:
            using base = FeatureMatch<_PointT>;  
        public:
            bool Match(_PointT const& point, SurfCostFactorInfo &res)
            {
                if (base::source_points_ == nullptr) return false;  
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                base::search_tree_->nearestKSearch(point, 5, pointSearchInd, pointSearchSqDis); 

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

                if (pointSearchSqDis[4] < base::search_thresh_)
                {
                    // 首先拟合平面   
                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = base::source_points_->points[pointSearchInd[j]].x;
                        matA0(j, 1) = base::source_points_->points[pointSearchInd[j]].y;
                        matA0(j, 2) = base::source_points_->points[pointSearchInd[j]].z;
                    }
                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);    // 法向量 n ( 未归一化)
                    double D = 1 / norm.norm();   // D  
                    norm.normalize(); // 归一化法向量 
                    // 判断该平面质量   
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * base::source_points_->points[pointSearchInd[j]].x +
                                norm(1) * base::source_points_->points[pointSearchInd[j]].y +
                                norm(2) * base::source_points_->points[pointSearchInd[j]].z + D) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    // 质量好  则匹配成功 
                    if (planeValid)
                    {
                        Eigen::Vector3d curr_point(point.x, point.y, point.z);
                        float distance = norm.dot(curr_point) + D;
                         // 残差  
                        res.residuals_ = std::fabs(distance);
                        
                        if (distance >= 0)
                        {
                            res.norm_ = norm;
                            res.D_ = D;  
                        } else {
                            res.norm_ = -norm;
                            res.D_ = -D;  
                        }
                        return true;
                    }
                }
                return false;          
            }
    };
}
