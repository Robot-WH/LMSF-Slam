#ifndef _FILTERFUSIONLIDARODOMETRY_BRIDGE_H_
#define _FILTERFUSIONLIDARODOMETRY_BRIDGE_H_

#include "utility.hpp"
#include "ros_utils.hpp"
#include "Estimator/LidarImuGnss_filter_estimator_interface.hpp"
#include "ros_bridge/FusionOdometry_bridge_interface.h"
#include "Estimator/initialize.hpp"
#include "Sensor/sensor.hpp"

using namespace Sensor; 
using namespace Estimator;  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Lidar-IMU-GNSS 滤波器融合的ros入口类 
 * @details 负责数据的传送
 */
class LidarImuGnssFilterFusionOdometryBridge : public FusionOdometryBridgeInterface    
{
    private:
        // 订阅激光里程计模块发送的数据
        ros::Subscriber subLidarPose;
        // IMU
        ros::Subscriber subImu;
        // gnss
        ros::Subscriber subGnss;

        ros::Publisher pubGnssPath;                   // 发布GNSS轨迹
        ros::Publisher pubFusionPath;                 // 发布GNSS轨迹
        ros::Publisher pubImuPredictPath;                 // 发布GNSS轨迹
        nav_msgs::Path GnssPath;                    // 记录gnss轨迹  
        nav_msgs::Path FusionPath;                  // 记录融合后轨迹  
        nav_msgs::Path ImuPredictPath;              // IMU预测的轨迹  

        // 维护IMU系相对于world系的姿态    
        Eigen::Matrix3d Rwi = Eigen::Matrix3d::Identity();
        Eigen::Vector3d twi = {0, 0, 0};
        // 通过滤波器估计前后两帧间的运动 
        // 核心！！   主要实现采用eskf/ieskf  融合imu,gps,lidar,轮速
        std::unique_ptr<LidarImuGnssFilterEstimatorInterFace> estimator_ptr_;  
        // 激光scan-map里程计对象
        
        // 数据缓存队列
        queue<Sensor::ImuDataPtr> imu_buf;  
        queue<nav_msgs::OdometryConstPtr> lidarOdom_buf;  
        queue<Sensor::GnssDataPtr> gnss_buf;

        // gnss 融合 开关 
        bool gnss_fusion_switch; 
        // gnss 轨迹显示 开关 
        bool gnss_path_switch; 
        // IMU 融合 开关 
        bool IMU_fusion_switch; 
        // IMU 轨迹显示 开关 
        bool IMU_path_switch; 
        // 轮速计 融合 开关
        bool wheels_fusion_switch; 

        // 线程同步相关
        std::mutex m_buf;

        // 坐标系
        std::string map_frame_id="map";
    
    public:    
        ~LidarImuGnssFilterFusionOdometryBridge(){} 
        LidarImuGnssFilterFusionOdometryBridge(std::unique_ptr<LidarImuGnssFilterEstimatorInterFace> &estimator_ptr);

        // 处理线程 
        void Process() override; 

    protected:
        // 内存分配 
        virtual void allocateMemory(); 
        // IMU回调函数 
        virtual void imuHandler(sensor_msgs::ImuConstPtr const& imu_msg);
        // 激光回调   几种重载实现 
        virtual void lidarPointCloudHandler(nav_msgs::OdometryConstPtr const& LidarOdom_msg); 
        // 接收不同类型的激光数据  
        // gnss回调
        virtual void gnssHandler(sensor_msgs::NavSatFixConstPtr const& navsat_msg); 
    
    private:
        void predict(const sensor_msgs::ImuConstPtr &imu_msg); 
        void update();
};



#endif