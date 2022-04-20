/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */

#ifndef _STATES_HPP_
#define _STATES_HPP_

#include <eigen3/Eigen/Dense>

namespace Slam3D{

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

    /**
     * @brief: 通用状态 位置、速度、姿态、重力 
     * @details: 仅仅利用运动学模型进行预测时采用  
     */    
    struct CommonStates {
            virtual void Reset() {
                P_.setZero();  
                V_.setZero();   
                Q_.setIdentity();
                G_ = {0., 0., -9.81007};
            }
            // 当前状态对应时间戳 
            double timestamp_;
            Eigen::Vector3d P_;      
            Eigen::Vector3d V_;      
            Eigen::Quaterniond Q_;     
            Eigen::Vector3d G_;          // gravity  
    }; // class StatesBase

    /**
     * @brief 融合IMU的 状态量
     */ 
    struct StatesWithImu {
            Eigen::Vector3d acc_bias_;   // The bias of the acceleration sensor.
            Eigen::Vector3d gyro_bias_;  // The bias of the gyroscope sensor.
            // 公共状态 
            CommonStates common_states_;
            
            void Reset() {
                common_states_.Reset();
                acc_bias_.setZero();
                gyro_bias_.setZero();
            }
    };

    /**
     * @brief:  融合轮速使用的状态
     * @details: 
     */    
    struct StatesWithWheels {};

    /**
     * @brief:  融合轮速和IMU使用的状态  
     * @details: 
     */    
    struct StatesWithWheelsIMU {};


}

#endif
