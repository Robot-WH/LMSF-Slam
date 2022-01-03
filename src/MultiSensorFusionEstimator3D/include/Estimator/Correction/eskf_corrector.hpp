/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */

#ifndef _ESKF_CORRECTOR_HPP_
#define _ESKF_CORRECTOR_HPP_

#include "utility.hpp"
#include "states_update.hpp"
#include "corrector_interface.hpp"

namespace Estimator {

/**
 * @brief ESKF滤波器的校正主流程  
 * @param __StatesType 状态的类型
 * @param _ErrorStateDim 状态的维度
 * @param __ObsType 观测量类型，认为是一个观测向量
 */
template<typename _StateType, typename  _ObsType, int _ErrorStateDim>
class EskfCorrector : public CorrectorInterface<_StateType, _ObsType, _ErrorStateDim> {
    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
        * @brief ESKF校正流程  
        * @details
        * @param[in] observation 观测量 
        * @param[in] timestamp 校正的时间戳 
        * @param[in] V 观测残差协方差 
        * @param[out] states 当前估计的状态 
        * @param[out] cov 后验协方差矩阵 
        */
        void Correct( _ObsType const& observation, 
                                    double const& timestamp, 
                                    Eigen::MatrixXd const& V,
                                    _StateType &states, 
                                    Eigen::Matrix<double, _ErrorStateDim, _ErrorStateDim> &cov) override {   
            // 更新时间
            states.common_states_.timestamp_ = timestamp;  
            // 计算残差以及观测H矩阵  
            Eigen::MatrixXd H;
            Eigen::VectorXd residual;
            residual = computeResidual(observation, states);       // 计算残差
            computeJacobian(states, H);                                                // 计算jacobian矩阵
            // 校正
            correct(H, V, residual, states, cov); 
        }
        
    protected:                       
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 校正过程
         * @param states_covariance 预测状态的协方差矩阵
         * @param H 观测矩阵 
         * @param V 观测噪声
         * @param residual 观测残差  
         * @param[out] states 校正后的状态
         * @param[out] cov 后验协方差矩阵
         */
        void correct( Eigen::MatrixXd const& H, Eigen::MatrixXd const& V, 
                                    Eigen::MatrixXd const& residual, _StateType &states, 
                                    Eigen::Matrix<double, _ErrorStateDim, _ErrorStateDim> &cov ) {
            // 首先根据观测计算误差状态 
            const Eigen::MatrixXd& P = cov;    
            const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
            const Eigen::Matrix<double, _ErrorStateDim, 1> delta_x = K * residual;
            // 用误差状态校正
            updateNominalStateByErrorState(delta_x, states);
            // 更新   Covarance.
            const Eigen::MatrixXd I_KH = Eigen::Matrix<double, _ErrorStateDim, _ErrorStateDim>::Identity() - K * H;
            cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 计算残差 
         * @param observation 观测量
         * @param  states 当前状态 
         */
        virtual Eigen::VectorXd computeResidual( _ObsType const& observation, 
                                                                _StateType const& states) = 0;  

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 计算jacobian
         * @param states 当前的状态
         * @param[out] jacobian jacobian的求解结果   
         */
        virtual void computeJacobian(_StateType const& states, Eigen::MatrixXd &jacobian) = 0;
};   // class EskfCorrector
} // namespace Estimator 
#endif  