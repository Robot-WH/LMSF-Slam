

#ifndef _IMU_MOTION_MODEL_INTERFACE_HPP_
#define _IMU_MOTION_MODEL_INTERFACE_HPP_

#include "utility.hpp"
#include "Sensor/sensor.hpp"

using namespace Sensor; 

namespace Model
{

    class ImuMotionModelInterface
    {
        public:
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 初始化
             */
            virtual void ImuMotionModelInitialize(ImuData const& curr_imu) = 0;  
        
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 预测PVQ
             */
            virtual void ImuPredictPvq(CommonStates & states, ImuDataConstPtr const& curr_imu, Eigen::Vector3d const& acc_bias, 
                                Eigen::Vector3d const& gyro_bias) = 0; 

    };
}

#endif