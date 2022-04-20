/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-11 17:16:34
 * @Description: 
 * @Others: 
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "Math.hpp"

namespace Algorithm {

    class se3PointSurfFactor : public ceres::SizedCostFunction<1, 7>     // 1: 残差维度    7 ： global size   
    {
        public:
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            se3PointSurfFactor(const Eigen::Vector3d &curr_point, 
                                                        const Eigen::Vector3d &plane_unit_norm, 
                                                        const double &plane_coeff_D) 
            : curr_point_(curr_point),  plane_unit_norm_(plane_unit_norm), plane_coeff_D_(plane_coeff_D)
            {
            }
            ~se3PointSurfFactor()
            {
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            virtual bool Evaluate(double const * const * parameters,  double *residuals, double **jacobians) const
            {
                Eigen::Map<const Eigen::Quaterniond> q_source_curr(parameters[0]);
                Eigen::Map<const Eigen::Vector3d> t_source_curr(parameters[0] + 4);

                Eigen::Vector3d target_point_after_trans = q_source_curr * curr_point_ + t_source_curr;  
                // 残差  
                residuals[0] = plane_unit_norm_.dot(target_point_after_trans) + plane_coeff_D_;

                if( jacobians != nullptr )
                {   
                    // 注意这里的jacobian 在维度上是  error 关于 李群的， 但是其实 是求关于李代数的jacobian
                    if( jacobians[0] != nullptr )
                    {
                        // 残差关于李群的jacobian
                        Eigen::Matrix<double, 3, 6> dp_by_se3;
                        dp_by_se3.block<3,3>(0,0) = -Math::GetSkewMatrix(target_point_after_trans);     // 关于旋转
                        (dp_by_se3.block<3,3>(0, 3)).setIdentity();
                        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
                        J_se3.setZero();
                        J_se3.block<1,6>(0,0) = plane_unit_norm_.transpose() * dp_by_se3;
                    }
                }
                return true;
            }
        private:
            Eigen::Vector3d curr_point_;
            Eigen::Vector3d plane_unit_norm_;   
            double plane_coeff_D_;    // 平面参数D 
    };
}
