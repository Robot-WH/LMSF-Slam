/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-11 17:16:48
 * @Description: 
 * @Others: 
 */
#pragma once 

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "Math.hpp"

namespace Algorithm {

    /**
     * @brief: 基于se3 计算的point - edge factor 
     * @details: 内部计算残差 ，梯度方向等
     */    
    class se3PointEdgeFactor : public ceres::SizedCostFunction<1, 7> 
    {
        public:
            se3PointEdgeFactor(Eigen::Vector3d curr_point, 
                                                    Eigen::Vector3d last_point_a, 
                                                    Eigen::Vector3d last_point_b)
            : curr_point_(curr_point), last_point_a_(last_point_a), last_point_b_(last_point_b)
            {}

            virtual ~se3PointEdgeFactor() {}

            virtual bool Evaluate(double const *const *parameters, 
                double *residuals, double **jacobians) const
            {
                Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
                Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
                Eigen::Vector3d lp;
                lp = q_last_curr * curr_point_ + t_last_curr; 

                Eigen::Vector3d nu = (lp - last_point_a_).cross(lp - last_point_b_);   // 平行四边形的面积 
                Eigen::Vector3d de = last_point_a_ - last_point_b_;  
                double de_norm = de.norm();
                residuals[0] = nu.norm()/de_norm;    // 点到线的距离    
                
                if(jacobians != NULL)
                {
                    if(jacobians[0] != NULL)
                    {
                        Eigen::Matrix<double, 3, 6> dp_by_se3;
                        dp_by_se3.block<3,3>(0,0) = -Math::GetSkewMatrix(lp);
                        (dp_by_se3.block<3,3>(0, 3)).setIdentity();
                        
                        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
                        J_se3.setZero();
                        Eigen::Matrix3d skew_de = Math::GetSkewMatrix(de);
                        J_se3.block<1,6>(0,0) = - nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
                    }
                }  
                return true;
            }
        private:
            Eigen::Vector3d curr_point_;
            Eigen::Vector3d last_point_a_;
            Eigen::Vector3d last_point_b_;
    };
}
