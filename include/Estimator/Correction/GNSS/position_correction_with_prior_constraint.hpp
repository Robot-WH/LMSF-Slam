/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */

#ifndef _POSITION_CORRECTION_WITH_PRIOR_CONSTRAINT_HPP_
#define _POSITION_CORRECTION_WITH_PRIOR_CONSTRAINT_HPP_

#include "utility.hpp"
#include "Estimator/Correction/eskf_corrector.hpp"
#include "Estimator/Correction/GNSS/position_correction.hpp"
#include "Sensor/Gnss_data.h"

namespace Estimator{
    /**
     * @brief 考虑先验运动约束的GNSS校正 
     */
    template<typename __StatesType, int __DimStates>
    class PositionCorrectionWithPriorConstraint : public  EskfCorrector<__StatesType, Eigen::Vector3d, __DimStates> {
        protected:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算XYZ+先验运动约束 残差   5维 
             */
            virtual Eigen::VectorXd computeResidual(Eigen::Vector3d const& position, 
                                                                    __StatesType const& states) override {
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
            virtual void computeJacobian(__StatesType const& states, Eigen::MatrixXd &jacobian) override {
                jacobian = Eigen::Matrix<double, 5, __DimStates>::Zero(); 
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