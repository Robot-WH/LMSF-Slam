
#ifndef _STATES_HPP_
#define _STATES_HPP_

/**
 * @brief 估计器状态的库
 * @author lwh 
 * @date 
 */

#include "utility.hpp"


namespace Estimator{

    static constexpr unsigned int Index_pos = 0;
    static constexpr unsigned int Index_vel = 3;
    static constexpr unsigned int Index_rot = 6;
    static constexpr unsigned int Index_ba = 9;     // 加速度bias
    static constexpr unsigned int Index_bg = 12;    // 角速度bias
    static constexpr unsigned int Dim_pos = 3;
    static constexpr unsigned int Dim_vel = 3;
    static constexpr unsigned int Dim_rot = 3;
    static constexpr unsigned int Dim_ba = 3;
    static constexpr unsigned int Dim_bg = 3;

    class CommonStates 
    {
        public:
            // 当前状态对应时间戳 
            double timestamp_;

            Eigen::Vector3d P_;      
            Eigen::Vector3d V_;      
            Eigen::Quaterniond Q_;     
            Eigen::Vector3d g_;          // gravity  

            void Reset()
            {
                P_.setZero();  
                V_.setZero();   
                Q_.setIdentity();
                g_ = {0., 0., -9.81007};
            }
    };

    /**
     * @brief 融合IMU的 状态量
     */ 
    class StatesWithImu
    {
        public:
            Eigen::Vector3d acc_bias_;   // The bias of the acceleration sensor.
            Eigen::Vector3d gyro_bias_;  // The bias of the gyroscope sensor.
            // 公共状态 
            CommonStates common_states_;
            // Covariance.
            Eigen::Matrix<double, 15, 15> cov_;

            void Reset()
            {
                common_states_.Reset();
                acc_bias_.setZero();
                gyro_bias_.setZero();
                cov_.setZero();
            }
    };


    /**
     * @brief 融合轮速的 状态量
     */ 
    class StatesWithWheels
    {

    };


    /**
     * @brief 融合轮速与IMU的 状态量
     */ 
    class StatesWithWheelImu
    {
    
    };


}

#endif
