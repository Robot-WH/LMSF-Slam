

#ifndef _ESKF_CORRECTOR_HPP_
#define _ESKF_CORRECTOR_HPP_

#include "utility.hpp"

namespace Estimator{

/**
 * @brief ESKF滤波器的校正主流程  
 */
class EskfCorrector
{
    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 校正过程
         * @param states_covariance 预测状态的协方差矩阵
         * @param H 观测矩阵 
         * @param V 观测噪声
         * @param residual 观测残差  
         * @param states 估计状态
         */
        template<typename _StateType, int _dim_state>
        void Correct( Eigen::MatrixXd const& H, Eigen::MatrixXd const& V, 
                                    Eigen::MatrixXd const& residual, _StateType &states )
        {
            // 首先根据观测计算误差状态 
            const Eigen::MatrixXd& P = states.cov_;    
            const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
            const Eigen::VectorXd delta_x = K * residual;
            // 用误差状态校正
            injectErrorStateToNominalState(delta_x, states);
            // Covarance.
            const Eigen::MatrixXd I_KH = Eigen::Matrix<double, _dim_state, _dim_state>::Identity() - K * H;
            states.cov_ = I_KH * P * I_KH.transpose() + K * V * K.transpose();
        }

    protected:                                                          
     
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 状态校正
         * @details 根据状态的不同进行重载  
         * @param delta_x 误差量
         * @param states 校正的状态 
         */
        void injectErrorStateToNominalState(Eigen::Matrix<double, 15, 1> const& delta_x, StatesWithImu &states) 
        {
            states.common_states_.P_     += delta_x.block<3, 1>(0, 0);
            states.common_states_.V_     += delta_x.block<3, 1>(3, 0);
            states.acc_bias_  += delta_x.block<3, 1>(9, 0);
            states.gyro_bias_ += delta_x.block<3, 1>(12, 0);
            Eigen::Vector3d axis_angle = delta_x.block<3, 1>(6, 0);
            // 旋转更新    
            if (axis_angle.norm() > 1e-12) 
            {
                states.common_states_.Q_ *= Eigen::Quaterniond(
                        1, axis_angle[0] / 2, axis_angle[1] / 2, axis_angle[2] / 2);
            }
        }
};   // class EskfCorrector

} // namespace Estimator 


#endif  