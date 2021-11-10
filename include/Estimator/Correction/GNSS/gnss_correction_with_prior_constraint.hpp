

#ifndef _GNSS_CORRECTION_WITH_PRIOR_CONSTRAINT_HPP_
#define _GNSS_CORRECTION_WITH_PRIOR_CONSTRAINT_HPP_

#include "utility.hpp"
#include "Estimator/Correction/eskf_corrector.hpp"
#include "Estimator/Correction/GNSS/gnss_correction.hpp"
#include "Sensor/Gnss_data.h"

namespace Estimator{
    /**
     * @brief 考虑先验运动约束的GNSS校正 
     */
    template<typename StatesType, int _dim_states>
    class PositionCorrectionWithPriorConstraint : public PositionCorrection<StatesType, _dim_states>
    {
        protected:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算GNSS+先验运动约束 残差   5维 
             * @param lla GNSS经纬高测量值 
             * @param P 位置状态预测值  
             */
            virtual Eigen::VectorXd computeResidual(Eigen::Vector3d const& position, 
                                                                    StatesType const& states) override
            {
                Eigen::Vector3d P = states.common_states_.P_;
                Eigen::Matrix<double, 5, 1> residual;
                // 残差 = 测量值 - 估计值
                residual.head(3) = position - P;
                // 计算速度观测的残差
                Eigen::Matrix3d R_b_w = states.common_states_.Q_.toRotationMatrix().transpose();
                Eigen::Matrix<double, 2, 3> K;
                K.row(0) = R_b_w.row(0);
                K.row(1) = R_b_w.row(2);
                residual.tail(2) = Eigen::Vector2d::Zero() - K*states.common_states_.V_; 
                // // 协方差
                // V = Eigen::Matrix<double, 5, 5>::Zero();
                // V.block<3,3>(0,0) = gps_data_ptr->cov;   // GNSS观测噪声协方差矩阵  
                // V.block<2,2>(3,3) = Eigen::Matrix2d::Identity() * 1; 
                return residual; 
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算jacobian
             */
            virtual void computeJacobian(StatesType const& states, Eigen::MatrixXd &jacobian) override 
            {
               jacobian = Eigen::Matrix<double, 5, _dim_states>::Zero(); 
               // Compute jacobian.
               Eigen::Matrix3d R_b_w = states.common_states_.Q_.toRotationMatrix().transpose();
               Eigen::Matrix<double, 2, 3> K;
               K.row(0) = R_b_w.row(0);
               K.row(1) = R_b_w.row(2);
               jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
               jacobian.block<2, 3>(3, 3) = K;
               jacobian.block<2, 3>(3, 6) = -K*GetSkewMatrix(states.common_states_.V_);
            }
    }; // class PositionCorrectionWithPriorConstraint
} // namespace Estimator 
#endif