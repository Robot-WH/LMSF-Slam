#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP


#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>

#include "utility.hpp"

// 参数服务器  
class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // 外参   
    Eigen::Matrix3d Rlb;     // body->lidar的旋转   
    Eigen::Vector3d tlb;     // body->lidar的平移
    Eigen::Matrix3d Ril;     // lidar->imu的旋转   
    Eigen::Vector3d til;     // lidar->imu的平移  
    Eigen::Matrix3d Rig;     // gnss->imu的旋转   
    Eigen::Vector3d tig;     // gnss->imu的平移 

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyroNoise;
    float imuAccBiasN;
    float imuGyroBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("liv_slam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("liv_slam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("liv_slam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("liv_slam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("liv_slam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("liv_slam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("liv_slam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("liv_slam/mapFrame", mapFrame, "map");

        nh.param<bool>("liv_slam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("liv_slam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("liv_slam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("liv_slam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("liv_slam/savePCD", savePCD, false);
        nh.param<std::string>("liv_slam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        nh.param<std::string>("liv_slam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("liv_slam/N_SCAN", N_SCAN, 16);
        nh.param<int>("liv_slam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("liv_slam/downsampleRate", downsampleRate, 1);
        nh.param<float>("liv_slam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("liv_slam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("liv_slam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("liv_slam/imuGyroNoise", imuGyroNoise, 0.001);
        nh.param<float>("liv_slam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("liv_slam/imuGyroBiasN", imuGyroBiasN, 0.00003);
        nh.param<float>("liv_slam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("liv_slam/imuRPYWeight", imuRPYWeight, 0.01);

        // 读取外参 
        // 先用vector接收
        vector<double> RotV;
        vector<double> TransV;
        // lidar->imu
        nh.param<vector<double>>("liv_slam/ExtrinsicLidarToImuRot", RotV, vector<double>());
        nh.param<vector<double>>("liv_slam/ExtrinsicLidarToImuTrans", TransV, vector<double>());
        Ril = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(RotV.data(), 3, 3);
        til = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(TransV.data(), 3, 1);  
        // lidar->body
        nh.param<vector<double>>("liv_slam/ExtrinsicBodyToLidarRot", RotV, vector<double>());
        nh.param<vector<double>>("liv_slam/ExtrinsicBodyToLidarTrans", TransV, vector<double>());
        Rlb = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(RotV.data(), 3, 3);
        tlb = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(TransV.data(), 3, 1);
        // gnss->imu
        nh.param<vector<double>>("liv_slam/ExtrinsicGnssToImuRot", RotV, vector<double>());
        nh.param<vector<double>>("liv_slam/ExtrinsicGnssToImuTrans", TransV, vector<double>());
        Rig = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(RotV.data(), 3, 3);
        tig = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(TransV.data(), 3, 1);


        cout<<" lidar->imu R: "<< endl << Ril << endl << " t: " << til.transpose() << endl;
        cout<<" body->lidar R: "<< endl << Rlb << endl << " t: " << tlb.transpose() << endl;  
        
        // 如果 body到lidar的外参没有给出   那么认为body系设置在imu上  
        if( Rlb ==  Eigen::Matrix3d::Zero() && tlb == Eigen::Vector3d::Zero())
        {
            Rlb = Ril.transpose();
            tlb = -til;
        }
        else    
        {

        }

        nh.param<float>("liv_slam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("liv_slam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("liv_slam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("liv_slam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("liv_slam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("liv_slam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("liv_slam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("liv_slam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("liv_slam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("liv_slam/numberOfCores", numberOfCores, 2);
        nh.param<double>("liv_slam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("liv_slam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("liv_slam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("liv_slam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("liv_slam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("liv_slam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("liv_slam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("liv_slam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("liv_slam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("liv_slam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("liv_slam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("liv_slam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("liv_slam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("liv_slam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("liv_slam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};


static sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, 
                                             ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
static double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
static void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
static void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
static void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped  Eigen转换为tf msg
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, 
                                                        const std::string& frame_id, const std::string& child_frame_id) 
{
  // 旋转矩阵 -> 四元数
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  // 四元数单位化
  quat.normalize();
  // 构造四元数   ROS信息
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();
  
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  // 该tf关系表示 从 frame_id-> child_frame_id
  odom_trans.header.frame_id = frame_id;       
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}


// 输入: 位姿的ROS Msg
// 输出: Eigen变换矩阵
static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) 
{
  const auto& orientation = odom_msg->pose.pose.orientation;  
  const auto& position = odom_msg->pose.pose.position;
  // ROS   四元数转Eigen
  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  // linear是获取旋转矩阵
  isometry.linear() = quat.toRotationMatrix();
  // 赋值平移
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}    

#endif // ROS_UTILS_HPP
