
#ifndef _FUSIONODOMETRY_BRIDGE_FACTOR_BRIDGE_HPP_
#define _FUSIONODOMETRY_BRIDGE_FACTOR_BRIDGE_HPP_

#include "utility.hpp"
#include "ros_utils.hpp"
#include "Sensor/sensor.hpp"
#include "ros_bridge/FusionOdometry_bridge_interface.h"
#include "ros_bridge/FilterFusionLIO_bridge.h"
#include "Estimator/LIO_filter_estimator_interface.hpp"
#include "Estimator/LIO_filter_estimator_robotCentre.hpp"
#include "Estimator/LIO_filter_estimator_worldCentre.hpp"
#include "Model/MotionModel/Imu_MotionModel/imu_motion_model_interface.hpp"
#include "Model/MotionModel/Imu_MotionModel/imu_midIntegral_model.hpp"

using namespace Sensor; 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 对接器工厂基类  
 */
class FusionOdometryBridgeFactory
{        
    protected:
        // 参数服务
        ParamServer param_server_;  
    public:    
        FusionOdometryBridgeFactory(){}
        virtual ~FusionOdometryBridgeFactory(){}
        virtual std::unique_ptr<FusionOdometryBridgeInterface> CreateFusionOdometryBridgeObject() = 0;  
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief lidar-imu-gnss 对接器 工厂
 */
class LidarImuGnssFusionOdometryBridgeFactory : public FusionOdometryBridgeFactory
{
    public:
        std::unique_ptr<FusionOdometryBridgeInterface> CreateFusionOdometryBridgeObject()
        {
            // IMU 运动模型       中值积分  
            std::shared_ptr<Model::ImuMotionModelInterface> imu_motion_model_ptr;
            imu_motion_model_ptr = std::make_shared<Model::ImuMidIntegralModel>(); 
            // 构建 估计器 
            std::unique_ptr<LidarImuFilterEstimatorInterFace> estimator_ptr{
                new LidarImuFilterEstimatorWorldCentre<StatesWithImu, 15>{
                    param_server_.imuAccNoise, param_server_.imuGyroNoise, 
                    param_server_.imuAccBiasN, param_server_.imuGyroBiasN, 
                    imu_motion_model_ptr, param_server_.Rlb, param_server_.tlb, 
                    param_server_.Ril, param_server_.til,
                    param_server_.Rig, param_server_.tig
                }};  
            // 创建ROS bridge对象   传入  估计器对象 
            std::unique_ptr<FusionOdometryBridgeInterface> fusionOdometry_ptr{
                new FilterFusionLIOBridge{estimator_ptr}};
            return fusionOdometry_ptr;  
        }
};


#endif