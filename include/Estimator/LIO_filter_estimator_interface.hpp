#ifndef _LIDARIMUGNSS_FILTER_ESTIMATOR_INTERFACE_HPP_
#define _LIDARIMUGNSS_FILTER_ESTIMATOR_INTERFACE_HPP_

#include "utility.hpp"
#include "Sensor/sensor.hpp"
#include "states.hpp"

namespace Estimator{

    /**
     * @brief imu-lidar 滤波估计器接口类   
     * @details 主要的变化是  1. 采用的滤波器的种类  如eskf ， ieskf ，区别主要在观测更新上  
     *                      2. 估计的状态不同      (重载？， 继承？)                          
     *                      3. 估计的方式 - 机体坐标系为中心 、 世界坐标为中心   (子类继承)
     */
    class LidarImuFilterEstimatorInterFace
    {  
        public: 
            LidarImuFilterEstimatorInterFace(){}
            virtual ~LidarImuFilterEstimatorInterFace(){}
            
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief Gnss是否已经初始化
             */
            bool IsGnssInitialized() const
            {
                return gnss_initialize_;  
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 激光雷达是否已经初始化
             */
            bool IsLidarInitialized() const
            {
                return lidar_initialize_;  
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 获取估计器上一时刻IMU数据
             */
            virtual Sensor::ImuDataConstPtr const& GetLastImuData() const = 0;

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 获取通用状态 即PVQ
             */
            virtual CommonStates const& GetCommonStates() const = 0;

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 处理IMU数据  
             */
            virtual void ProcessSensorData(Sensor::ImuDataConstPtr &imuData) = 0;    
    
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 处理GNSS数据  
             */
            virtual void ProcessSensorData(Sensor::GnssDataConstPtr const& gnssData) = 0;  

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 处理Lidar数据  
             * @details 处理Lidar观测的位姿
             */
            virtual void ProcessSensorData() = 0;  
        
        protected:
            // gnss初始化标志
            bool gnss_initialize_ = false;     
            // lidar初始化标志
            bool lidar_initialize_ = false;
    };  
}

#endif 