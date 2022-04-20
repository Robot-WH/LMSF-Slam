/*
 * @Copyright(C):
 * @FileName: 文件名
 * @Author: lwh
 * @Version: v1.0
 * @Date: 2022-01-05 
 * @Description:  LIO -  激光、IMU、轮速、GNSS 滤波器融合里程计， lidar x 1 + imu + wheel + gnss 
 *                                 功能:  1、Lidar 与 GNSS&IMU 在线外参标定
 *                                              2、IMU去除Lidar畸变
 *                                              3、零速检测优化以及外参在线优化 
 *                                              4、纯激光融合运动学模型  
 *                                              5、支持 eskf, ieskf 
 */
#include "ros_utils.hpp"
#include "utility.hpp"
#include "Estimator/states.hpp"
#include "Estimator/filter_lio_estimator.hpp" 
#include "boost/scoped_ptr.hpp"
#include "Estimator/Correction/GNSS/position_correction.hpp"
#include "factory/registration/registration_factory.hpp"

using namespace Sensor;
using namespace Estimator;
using namespace std;  

ros::Subscriber subImu;                             // IMU
ros::Subscriber subGnss;                           // gnss    提供真值用于比较  
ros::Subscriber subLidar;                           

ros::Publisher pubGnssPath;                    // 发布GNSS轨迹
ros::Publisher pubFusionPath;                // 发布GNSS轨迹
ros::Publisher pubImuPredictPath;                 // 发布GNSS轨迹

nav_msgs::Path GnssPath;                           // 记录gnss轨迹  
nav_msgs::Path FusionPath;                       // 记录融合后轨迹  
nav_msgs::Path ImuPredictPath;              // IMU预测的轨迹  
// 维护IMU系相对于world系的姿态    
Eigen::Matrix3d Rwi = Eigen::Matrix3d::Identity();
Eigen::Vector3d twi = {0, 0, 0};
// LIO估计器
std::unique_ptr<Estimator::LioEstimator<Estimator::StatesWithImu, 15>> estimator;  
// 数据缓存队列
queue<Sensor::ImuDataPtr> imu_buf;  
queue<Sensor::GnssDataPtr> gnss_buf;    
queue<Sensor::LidarDataPtr> lidar_buf; 
// 线程同步相关
std::mutex m_buf;   
// 坐标系
std::string map_frame_id="map";
// IMU
float imuAccNoise = 0;
float imuGyroNoise = 0;
float imuAccBiasN = 0;
float imuGyroBiasN = 0;
// NDT
float ndt_resolution = 0; 
float transformation_epsilon = 0; 
int maximum_iterations = 0; 
int num_threads = 0;  
string nn_search_method;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void allocateMemory() {
    // // 匹配算法设置
    // std::unique_ptr<pcl::Registration<PointType, PointType>> registration_ptr
    //     =  Factory::make_ndtOmp(ndt_resolution, transformation_epsilon,  maximum_iterations,  num_threads,
    //                                             nn_search_method);  
    // // 估计器初始化 
    // estimator.reset( new Estimator::LioEstimator<Estimator::StatesWithImu, 15>(
    //                                             imuAccNoise, imuGyroNoise, imuAccBiasN, imuGyroBiasN, 
    //                                             // 设置纯激光里程计-用于初始化的外参标定 
    //                                             std::make_unique<LidarTrackerDirectMethodLocalMap>(
    //                                                 // 设置匹配算法 
    //                                                 std::move(registration_ptr)
    //                                             )
    //                                 ));  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imuHandler( sensor_msgs::ImuConstPtr const& imu_msg) {
    // 保证队列中 数据的顺序正确 
    m_buf.lock();
    static double last_imu_t = -1; 
    if (imu_msg->header.stamp.toSec() <= last_imu_t) {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    // 解析IMU数据 
    Sensor::ImuDataPtr imu_data_ptr = std::make_shared<Sensor::ImuData>();
    // 保存时间戳 
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg->linear_acceleration.x, 
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg->angular_velocity.x,
                        imu_msg->angular_velocity.y,
                        imu_msg->angular_velocity.z;
    imu_data_ptr->rot.w() = imu_msg->orientation.w;
    imu_data_ptr->rot.x() = imu_msg->orientation.x;
    imu_data_ptr->rot.y() = imu_msg->orientation.y;
    imu_data_ptr->rot.z() = imu_msg->orientation.z;
    imu_buf.push(imu_data_ptr);
    m_buf.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gnssHandler(sensor_msgs::NavSatFixConstPtr const& navsat_msg) {
    // 保证队列中 数据的顺序正确 
    m_buf.lock();
    static double last_gnss_t = -1; 
    if (navsat_msg->header.stamp.toSec() <= last_gnss_t) {
        ROS_WARN("gnss message in disorder!");
        return;
    }
    last_gnss_t = navsat_msg->header.stamp.toSec();
    // 解析Gnss数据 
    Sensor::GnssDataPtr gnss_data_ptr = std::make_shared<Sensor::GnssData>();
    // 保存时间戳 
    gnss_data_ptr->timestamp = navsat_msg->header.stamp.toSec();
    gnss_data_ptr->lla << navsat_msg->latitude,
                        navsat_msg->longitude,
                        navsat_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(navsat_msg->position_covariance.data());
    gnss_buf.push(gnss_data_ptr);
    m_buf.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lidarHandler(sensor_msgs::PointCloud2ConstPtr const& lidar_msg) {
    m_buf.lock();
    Sensor::LidarDataPtr lidar_data_ptr = std::make_shared<Sensor::LidarData>();  
    pcl::fromROSMsg(*lidar_msg, lidar_data_ptr->point_clouds_);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(lidar_data_ptr->point_clouds_, lidar_data_ptr->point_clouds_, indices);
    lidar_data_ptr->timestamp = lidar_msg->header.stamp.toSec();     // 起始点的时间戳  
    lidar_buf.push(lidar_data_ptr);
    m_buf.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Process() {
    while (true) {   
        std::unique_lock<std::mutex> m_g(m_buf);
        // 如果有数据容器非空  那么进行处理 
        if (!imu_buf.empty()||!gnss_buf.empty()||!lidar_buf.empty()) {
            uint8_t sensor_id = 0;
            double earliest_time = numeric_limits<double>::max();  
            // 找到最早的数据  
            if (!imu_buf.empty()) {   
                earliest_time = imu_buf.front()->timestamp;  
                sensor_id = imu;
            }
            if (!gnss_buf.empty()) {   
                // 如果gnss更早 
                if(earliest_time > gnss_buf.front()->timestamp) {
                    earliest_time = gnss_buf.front()->timestamp;
                    sensor_id = gnss;
                }
            }
            if (!lidar_buf.empty()) {   
                // 如果lidar更早 
                if(earliest_time > lidar_buf.front()->timestamp) {
                    earliest_time = lidar_buf.front()->timestamp;
                    sensor_id = lidar;
                }
            }
            switch (sensor_id) {
                // 如果是lidar数据  执行观测
                case lidar: {
                    // 取出头数据 
                    Sensor::LidarDataConstPtr lidar_ptr = lidar_buf.front();  
                    lidar_buf.pop();  
                    m_g.unlock();    
                    estimator->Process(lidar_ptr);      // 将数据传入到估计器中进行
                }
                break;  
                // 如果是imu数据执行 预测  
                case imu: {
                    // 取出头数据 
                    Sensor::ImuDataConstPtr imu_ptr = imu_buf.front();  
                    imu_buf.pop();  
                    m_g.unlock();    
                    estimator->Process(imu_ptr);      // 将数据传入到估计器中进行
                    // 显示当前IMU的预测轨迹
                    if(estimator->IsInitialized()) {   
                        CommonStates const& common_states = estimator->GetState().common_states_;     // 获取估计后的通用状态 
                        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
                        geometry_msgs::PoseStamped imu_predict_pose; 
                        imu_predict_pose.header.stamp = ros::Time{imu_ptr->timestamp};
                        imu_predict_pose.pose.position.x = common_states.P_.x();
                        imu_predict_pose.pose.position.y = common_states.P_.y();
                        imu_predict_pose.pose.position.z = common_states.P_.z();
                        imu_predict_pose.pose.orientation.w = rot.w();
                        imu_predict_pose.pose.orientation.x = rot.x();
                        imu_predict_pose.pose.orientation.y = rot.y();
                        imu_predict_pose.pose.orientation.z = rot.z();
                        ImuPredictPath.header.stamp = imu_predict_pose.header.stamp;
                        ImuPredictPath.poses.push_back(imu_predict_pose);
                        ImuPredictPath.header.frame_id = map_frame_id; // odom坐标
                        pubImuPredictPath.publish(ImuPredictPath);
                        std::cout<<"pubImuPredictPath! "<<" num: "<< ImuPredictPath.poses.size() <<std::endl;
                    }
                }
                break;
                case gnss: {   
                    // 至少要保证容器中有2个以上数据才进行处理 
                    if(gnss_buf.size()<2) {
                        break;
                    }
                    // 取出头数据 
                    Sensor::GnssDataConstPtr gnss_ptr = gnss_buf.front();  
                    gnss_buf.pop();  
                    m_g.unlock();  
                    // 如果初始化了  那么开始校正之前 检查是否需要增加预测 
                    if(estimator->IsInitialized()) {
                        // 由于校正时，上一次IMU的预测状态的时间戳可能与当前校正的时间戳有较大的偏差
                        // 如果存在IMU数据    那么在校正之前需要插值出GNSS时间戳处的IMU数据  然后再次进行预测 
                        if(!imu_buf.empty()) {
                            // 插值 
                            Sensor::ImuDataConstPtr curr_imu = imu_buf.front();  
                            imu_buf.pop();   
                            Sensor::ImuDataConstPtr const& last_imu = estimator->GetLastIMU();     // 获取上一个IMU数据 
                            Sensor::ImuDataConstPtr interpolation_imu = ImuDataLinearInterpolation(last_imu, curr_imu, 
                                                                                                        gnss_ptr->timestamp); // 插值出 GNSS 位置处的 IMU数据                                                                       
                            // 用插值后的数剧进行预测
                            estimator->Process(interpolation_imu);  
                        } else {    // 如果之后没有IMU数据 即IMU挂了
                                        // 如果IMU没有    那么可以采用匀速运动学模型进行预测   
                                        // 首先判断上一个IMU的时间间隔是否足够大，若足够大那么需要通过运动学模型
                                        // 进行预测，若比较小，则不需要预测 
                        }
                    }
                    // 校正 
                    estimator->Process(gnss_ptr);  
                    // 保存GNSS的轨迹 
                    if (estimator->IsInitialized()) {   
                        // 获取更新后的GNSS轨迹 
                        GnssDataProcess& gnss_processor = GnssDataProcess::GetInstance();  
                        Eigen::Vector3d xyz = gnss_processor.GetEnuPosition();
                        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
                        // 发布odom
                        geometry_msgs::PoseStamped Pose; 
                        Pose.header.stamp = ros::Time{gnss_ptr->timestamp};
                        Pose.pose.position.x = xyz.x();
                        Pose.pose.position.y = xyz.y();
                        Pose.pose.position.z = xyz.z();
                        Pose.pose.orientation.w = rot.w();
                        Pose.pose.orientation.x = rot.x();
                        Pose.pose.orientation.y = rot.y();
                        Pose.pose.orientation.z = rot.z();
                        GnssPath.header.stamp = Pose.header.stamp;
                        GnssPath.poses.push_back(Pose);
                        GnssPath.header.frame_id = map_frame_id; // odom坐标
                        pubGnssPath.publish(GnssPath);
                        std::cout<<"pubGnssPath! "<<" num: "<< GnssPath.poses.size() 
                        << " xyz: "<< xyz.transpose() <<std::endl;
                        // 发布滤波估计后的轨迹 
                        CommonStates const& common_states = estimator->GetState().common_states_;    // 获取估计后的通用状态 
                        Pose.pose.position.x = common_states.P_.x();
                        Pose.pose.position.y = common_states.P_.y();
                        Pose.pose.position.z = common_states.P_.z();
                        FusionPath.header.stamp = Pose.header.stamp;
                        FusionPath.poses.push_back(Pose);
                        FusionPath.header.frame_id = map_frame_id; // odom坐标
                        pubFusionPath.publish(FusionPath);
                        // IMU预测的轨迹清空
                        ImuPredictPath.poses.clear();
                    }
                }
                break;  
            }
        }
        // 10us的延时  
        std::chrono::microseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init(ros::NodeHandle &private_nh, ros::NodeHandle &nh) {   
    private_nh = ros::NodeHandle("~");
    string imu_topic, gps_topic, lidar_topic;
    imu_topic = private_nh.param<std::string>("ImuTopic", "/");
    gps_topic = private_nh.param<std::string>("GnssTopic", "/");
    lidar_topic = private_nh.param<std::string>("LidarTopic", "/");
    ROS_WARN("imu_topic:");
    std::cout<<imu_topic<<std::endl;
    ROS_WARN("gps_topic:");
    std::cout<<gps_topic<<std::endl;
    ROS_WARN("lidar_topic:");
    std::cout<<lidar_topic<<std::endl;
    subImu = private_nh.subscribe(imu_topic, 1000, &imuHandler,
                                                    ros::TransportHints().tcpNoDelay());
    subGnss = private_nh.subscribe(gps_topic, 1000, &gnssHandler, 
                                                    ros::TransportHints().tcpNoDelay());  
    subLidar = private_nh.subscribe(lidar_topic, 1000, &lidarHandler, 
                                                    ros::TransportHints().tcpNoDelay());  
    pubGnssPath = private_nh.advertise<nav_msgs::Path>("/Gnsspath", 100);
    pubFusionPath = private_nh.advertise<nav_msgs::Path>("/Fusionpath", 100);
    pubImuPredictPath = private_nh.advertise<nav_msgs::Path>("/ImuPredictpath", 100);
    // 读取IMU的内参  
    nh.param<float>("liv_slam/imuAccNoise", imuAccNoise, 0.01);
    nh.param<float>("liv_slam/imuGyroNoise", imuGyroNoise, 0.001);
    nh.param<float>("liv_slam/imuAccBiasN", imuAccBiasN, 0.0002);
    nh.param<float>("liv_slam/imuGyroBiasN", imuGyroBiasN, 0.01);
    std::cout<<"imuAccNoise: "<<imuAccNoise<<std::endl;
    std::cout<<"imuGyroNoise: "<<imuGyroNoise<<std::endl;
    std::cout<<"imuAccBiasN: "<<imuAccBiasN<<std::endl;
    std::cout<<"imuGyroBiasN: "<<imuGyroBiasN<<std::endl;
    // 匹配算法参数
    nh.param<float>("liv_slam/Registration/NDT/ndt_resolution", ndt_resolution, 0);
    nh.param<float>("liv_slam/Registration/NDT/transformation_epsilon", transformation_epsilon, 0);
    nh.param<int>("liv_slam/Registration/NDT/maximum_iterations", maximum_iterations, 0);
    nh.param<int>("liv_slam/Registration/NDT/num_threads", num_threads, 0);
    nh.param<string>("liv_slam/Registration/NDT/nn_search_method", nn_search_method, "");
    std::cout<<"NDT/ndt_resolution: "<<ndt_resolution<<std::endl;
    std::cout<<"NDT/transformation_epsilon: "<<transformation_epsilon<<std::endl;
    std::cout<<"NDT/maximum_iterations: "<<maximum_iterations<<std::endl;
    std::cout<<"NDT/num_threads: "<<num_threads<<std::endl;
    std::cout<<"NDT/nn_search_method: "<<nn_search_method<<std::endl;
    allocateMemory();  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    ros::init(argc, argv, "filter_node");
    ROS_INFO("Started filter lio node");  
    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    init(private_nh, nh);  
    // 融合线程  
    std::thread measurement_process{&Process};
    ros::spin();
    return 0;
}