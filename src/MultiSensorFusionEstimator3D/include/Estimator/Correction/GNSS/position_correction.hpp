/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */

#ifndef _POSITION_CORRECTION_HPP_
#define _POSITION_CORRECTION_HPP_

#include "utility.hpp"
#include "Estimator/Correction/eskf_corrector.hpp"
#include "Estimator/Math/jacobian_state_with_error.hpp"

namespace Slam3D{
    
    /**
     * @brief:  ESKF 3DOF - XYZ 观测校正
     * @details: 一般用于如GNSS, UWB等 观测   
     * @param _StateType 使用状态
     * @param _StateDim 估计的状态维度  比如是否优化重力在这个维度上会有区别  
     * @param _ErrorStateDim 对应误差状态的维度  
     */    
    template<typename _StateType, int _StateDim, int _ErrorStateDim>
    class PositionCorrection : public  EskfCorrector<_StateType, Eigen::Vector3d, _ErrorStateDim> {
        public:
            PositionCorrection() {}
            virtual ~PositionCorrection() {}  
        protected:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算位置残差 
             */
            virtual Eigen::VectorXd computeResidual( Eigen::Vector3d const& position, 
                                                                  _StateType const& states) override {
                Eigen::Vector3d P = states.common_states_.P_;    // 当前状态的位置 
                Eigen::Vector3d residual;                                                  // 观测残差  
                // 残差 = 测量值 - 估计值
                residual = position - P;
                return residual; 
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算jacobian
             */
            virtual void computeJacobian(_StateType const& states, Eigen::MatrixXd &jacobian) override {   
                Eigen::MatrixXd jacobian_G_with_X = Eigen::Matrix<double, 3, _StateDim>::Zero();  
                // Compute jacobian.
                jacobian_G_with_X.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                jacobian = jacobian_G_with_X * ComputerJacobianStateWithErrorState<_StateType, _StateDim>(states);  
            }
    }; // class PositionCorrection
} // namespace Estimator 
#endif