/*********************************************************************************************
 * @brief 基于滤波器的多传感器融合激光里程计
 * @details 3D激光雷达为主  融合IMU，GNSS  
 *          坐标系：车体坐标系(轮速计)： b    imu系： i    激光系： l   
 * @author wh.l
 * @date 2020.10.20
 *********************************************************************************************/
#include "ros_bridge/LidarImuGnssFilterFusionOdometry_bridge.h"
#include "ros_bridge/FusionOdometry_bridge_interface.h"
#include "ros_bridge/FusionOdometry_bridge_factor.hpp"
#include "Estimator/LidarImuGnss_filter_estimator_robotCentre.hpp"


using namespace std;  
using namespace Sensor; 

// Estimator estimator;

// std::condition_variable con;
// double current_time = -1;
// queue<sensor_msgs::ImuConstPtr> imu_buf;
// queue<sensor_msgs::PointCloudConstPtr> feature_buf;
// queue<sensor_msgs::PointCloudConstPtr> relo_buf;
// int sum_of_wait = 0;

// std::mutex m_buf;
// std::mutex m_state;
// std::mutex i_buf;
// std::mutex m_estimator;

// double latest_time;
// Eigen::Vector3d tmp_P;
// Eigen::Quaterniond tmp_Q;
// Eigen::Vector3d tmp_V;
// Eigen::Vector3d tmp_Ba;
// Eigen::Vector3d tmp_Bg;
// Eigen::Vector3d acc_0;
// Eigen::Vector3d gyr_0;
// bool init_feature = 0;
// bool init_imu = 1;
// double last_imu_t = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "FilterFusionOdometry_node");
    ROS_INFO("Started FilterFusionLidarOdometryBridge node");  
    // 构建工厂对象  
    std::unique_ptr<FusionOdometryBridgeFactor> fusion_odometry_bridge_factor;  
    fusion_odometry_bridge_factor.reset(new LidarImuGnssFusionOdometryBridgeFactor{});     // 变化 ！！！！！！！！！！！！！！      
    
    std::unique_ptr<FusionOdometryBridgeInterface> fusion_odometry_bridge = std::move(
        fusion_odometry_bridge_factor->CreateFusionOdometryBridgeObject() );  
    // 融合线程  
    std::thread measurement_process{&FusionOdometryBridgeInterface::Process, &(*fusion_odometry_bridge)};
    ros::spin();

    return 0;
}
