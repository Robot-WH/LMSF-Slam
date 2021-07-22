
#ifndef _GNSS_CORRECTION_HPP_
#define _GNSS_CORRECTION_HPP_

#include "utility.hpp"
#include "Estimator/Correction/eskf_corrector.hpp"
#include "Sensor/Gnss_data.h"

namespace Estimator{
    
    template<typename StatesType, int _dim_states>
    class PositionCorrection
    {
        public:
            PositionCorrection()
            {
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
            * @brief ESKF校正 
            * @details
            * @param position 观测位置
            * @param states 当前估计的状态 
            * @param timestamp 校正的时间戳 
            * @param V 残差协方差 
            */
            void Correct(Eigen::Vector3d const& position, StatesType &states, double const& timestamp, Eigen::MatrixXd const& V)
            {   
                // 更新时间
                states.common_states_.timestamp_ = timestamp;  
                // 计算残差以及观测H矩阵  
                Eigen::MatrixXd H;
                Eigen::VectorXd residual;
                residual = computeResidual(position, states);       // 计算残差
                computeJacobian(states, H);                         // 计算jacobian矩阵
                // 校正
                eskf_corrector_.Correct<StatesType, _dim_states>(H, V, residual, states); 
            }


        protected:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算GNSS残差 
             * @param lla GNSS经纬高测量值 
             * @param P 位置状态预测值  
             * @param V 残差的协方差矩阵  
             */
            virtual Eigen::VectorXd computeResidual( Eigen::Vector3d const& position, 
                                                                  StatesType const& states)
            {
                Eigen::Vector3d P = states.common_states_.P_;    // 当前状态的位置 
                Eigen::Vector3d residual;                        // 观测残差  
                // 残差 = 测量值 - 估计值
                residual = position - P;
                return residual; 
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 计算jacobian
             */
            virtual void computeJacobian(StatesType const& states, Eigen::MatrixXd &jacobian) 
            {   
                jacobian = Eigen::Matrix<double, 3, _dim_states>::Zero(); 
                // Compute jacobian.
                jacobian.block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
            }

        private:
            // ESKF估计器   
            EskfCorrector eskf_corrector_;  

    };
} // namespace Estimator 
#endif