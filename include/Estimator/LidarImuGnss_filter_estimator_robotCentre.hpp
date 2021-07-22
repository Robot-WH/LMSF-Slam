
/**
 * @brief Lidar-IMU-GNSS 滤波器状态估计器  
 * @author wh.l
 * @date 2021/4/20
 */

#ifndef _LIDARIMUGNSS_FILTER_ESTIMATOR_ROBOTCENTRE_HPP_
#define _LIDARIMUGNSS_FILTER_ESTIMATOR_ROBOTCENTRE_HPP_

#include "utility.hpp"
#include "Estimator/LidarImuGnss_filter_estimator_interface.hpp"
#include "Estimator/Predictor/imu_predictor.hpp"
#include "Estimator/Correction/GNSS/gnss_correction.hpp"
#include "Estimator/Correction/GNSS/gnss_correction_with_prior_constraint.hpp"
#include "initialize.hpp"
#include "Sensor/Gnss_data.h"

namespace Estimator{

  /**
   * @brief 使用滤波器融合IMU-GPS-LIDAR的里程计      
   * @details 估计的方式是以局部机体坐标系为中心
   *          变化主要有： 1. 滤波器  eskf/ieskf(主要是观测更新的区别，采取多态)    2. 估计的状态变量设置 (可以复用 ，采取模板)
   *            
   */
  template<typename EstimatedStates, int _dim_states >
  class LidarImuGnssFilterEstimatorRobotCentre : public LidarImuGnssFilterEstimatorInterFace
  {
    private:

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
        /**
         * @brief 初始化
         */
        bool estimatorInitialize(double const& timestamp)
        {  
          
        }  

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 估计器状态初始化 
         */
        void estimatorStatesInitialize(StatesWithImu &states, Eigen::Matrix3d const& R_b_w, 
                                        Eigen::Vector3d const& gyro_bias)
        {
          states.Reset();
          // 重力转换到局部坐标系 
          states.common_states_.g_ = R_b_w * global_states_.g_;  
          states.gyro_bias_ = gyro_bias;
          // 状态初始协方差设置   
          states.cov_.block<3, 3>(0, 0) = 0.01 * Eigen::Matrix3d::Identity(); // position std: 0.1 m
          states.cov_.block<3, 3>(3, 3) = 0.01 * Eigen::Matrix3d::Identity(); // velocity std: 0.1 m/s
          // roll pitch std 10 degree.
          states.cov_.block<2, 2>(6, 6) = 10. * CoefDegreeToRadian * 10. * CoefDegreeToRadian * Eigen::Matrix2d::Identity();
          states.cov_(8, 8)             = 100. * CoefDegreeToRadian * 100. * CoefDegreeToRadian;     // yaw std: 100 degree.
          // Acc bias.
          states.cov_.block<3, 3>(9, 9) = 0.01 * 0.01 * Eigen::Matrix3d::Identity();   // 0.02
          // Gyro bias.
          states.cov_.block<3, 3>(12, 12) = 0.0001 * 0.0001 * Eigen::Matrix3d::Identity();
        }

    public: 
        LidarImuGnssFilterEstimatorRobotCentre(){}
        ~LidarImuGnssFilterEstimatorRobotCentre(){}
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        LidarImuGnssFilterEstimatorRobotCentre(string const& fusion_filter_name)
        {
        }
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 处理IMU数据  
         */
        void ProcessSensorData(Sensor::ImuDataConstPtr &imuData) override
        { 
          // 只要gnss与lidar有一个初始化了  那么就进行滤波器的预测环节
          if(gnss_initialize_||lidar_initialize_)
          {
            imu_predict_.ImuPredict(local_states_, imuData);
          }
          else
          { 
            prepared_imus_.emplace_back(std::move(imuData));       // 缓存数据用于初始化  
          }
        } 
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 处理GNSS数据  
         */
        void ProcessSensorData(Sensor::GnssDataConstPtr const& gnssData) override
        {
          // 如果gnss没有初始化 
          if(!gnss_initialize_)
          {  
            // 如果LIO没有初始化   那么先初始化INS 运行 gps与imu的组合导航 
            if(!lidar_initialize_)
            {    
              // 初始化GNSS 
              // 估计器初始化 
              if(estimatorInitialize())
              { 
                gnss_initialize_ = true;  
                // GNSS初始化 
                Sensor::GnssDataProcess* gnss_data_process = Sensor::GnssDataProcess::GetInstance(); 
                gnss_data_process->InitOriginPosition(gnssData->lla);  
              }  
            }
            else  // lidar初始化后的GNSS初始化   
            {
                
            }
          }
          else   // 如果初始化了 执行滤波器校正  
          {

          }

        } 
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 处理Lidar数据  
         * @details 处理Lidar观测的位姿
         */
        void ProcessSensorData() override
        {
          if(!lidar_initialize_)    // 没初始化那么进行初始化 
          { 
            // 如果gnss没有初始化  则先初始化lidar&imu   运行lio
            if(!gnss_initialize_)
            {
                
            }
            else   // 进行gnss已经运行后的 LIO初始化    
            {
                
            } 
          }
        }  
      
    private:
        // 初始化器
        Initializer initializer_;  
        // 全局状态   body->odom
        CommonStates global_states_;   
        // 局部估计的状态
        EstimatedStates local_states_;  
        // IMU抽象接口
        ImuPredictor imu_predict_;  
        // Lidar抽象接口 
        
        // GNSS抽象接口  
        PositionCorrection<EstimatedStates, _dim_states> gnss_correction_; 
        
        // 用于初始化的imu数据 
        std::vector<Sensor::ImuDataConstPtr> prepared_imus_;  
          
  };  

} // namespace Estimator  
#endif