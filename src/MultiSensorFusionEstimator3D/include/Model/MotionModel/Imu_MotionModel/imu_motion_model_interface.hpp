/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */


#ifndef _IMU_MOTION_MODEL_INTERFACE_HPP_
#define _IMU_MOTION_MODEL_INTERFACE_HPP_

#include "utility.hpp"
#include "Sensor/sensor.hpp"
#include "Estimator/states.hpp"

using namespace Sensor; 

namespace Model {
    class ImuMotionModelInterface {
        public:
            ImuMotionModelInterface() {}
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 初始化
             * @details 有的需要专门的初始化  有的不需要  所以这里不设置为纯虚函数     
             */
            virtual void ImuMotionModelInitialize(ImuData const& curr_imu) {};  
        
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 预测PVQ
             */
            virtual void ImuPredictPVQ(Estimator::CommonStates & states, 
                                                                     ImuDataConstPtr const& curr_imu, 
                                                                     Eigen::Vector3d const& acc_bias, 
                                                                     Eigen::Vector3d const& gyro_bias) = 0; 
    }; // class ImuMotionModelInterface
} // namespace Model 
#endif