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
#include "Sensor/Gnss_data.h"

namespace Estimator{
    
    /**
     * @brief:  ESKF 3DOF - XYZ 观测校正
     * @details: 
     * @param 
     */    
    template<typename __StatesType, int __dim_states>
    class PositionCorrection : public  EskfCorrector<__StatesType, Eigen::Vector3d, __dim_states> {
        public:
            PositionCorrection() {}
            virtual ~PositionCorrection() {}  
        protected:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算位置残差 
             */
            virtual Eigen::VectorXd computeResidual( Eigen::Vector3d const& position, 
                                                                  __StatesType const& states) override {
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
            virtual void computeJacobian(__StatesType const& states, Eigen::MatrixXd &jacobian) override {   
                jacobian = Eigen::Matrix<double, 3, __dim_states>::Zero(); 
                // Compute jacobian.
                jacobian.block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
            }
    }; // class PositionCorrection
} // namespace Estimator 
#endif