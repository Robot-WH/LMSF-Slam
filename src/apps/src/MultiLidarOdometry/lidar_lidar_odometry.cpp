/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: lwh
 * @Version: 1.0
 * @Date: 2022-01-27 16:32:54
 * @Description:  参考MLOAM实现的多激光标定与里程计  
 * @Others: 
 */
#include "ros_utils.hpp"
#include "Sensor/sensor.hpp"
#include "Sensor/lidar_data_type.h"
#include "Common/color.hpp"
#include "Common/parameters.h"
#include "Algorithm/PointClouds/processing/common_processing.hpp"
#include "Algorithm/PointClouds/processing/deskew/deskew_base.hpp"
#include "Algorithm/PointClouds/processing/Preprocess/RotaryLidar_preprocessing.hpp"
#include "Estimator/estimator/MultiLidar/MultiLidar_estimator_base.hpp"
// #include "Estimator/estimator/MultiLidar/Direct_MultiLidar_estimator.hpp"
#include "Estimator/estimator/MultiLidar/MultiLidar_estimator.hpp"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include "factory/estimator/multiLidar_estimator/multiLidar_loamFeatureEstimator_factory.hpp"

using namespace std;
using namespace Slam3D;
using namespace Algorithm;  


#define USE_SINGLE_LIDAR 0     // 使用单激光时  该激光的id 

using PointT = pcl::PointXYZI;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;  
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;  
using PointCloud = pcl::PointCloud<PointT>;  

string config_path;     // 算法参数文件path 
int NUM_OF_LIDAR;   
std::vector<std::string> lidar_topic_names = {"multiSensorFusion_slam/lidar/topic/index_0",
                                                                                              "multiSensorFusion_slam/lidar/topic/index_1" };  
std::vector<std::string> lidar_frame_names = {"lidar_0", "lidar_1" };                                                                                                
std::vector<std::string> lidar_topic_container;  
string public_topic = "undistortion_pointcloud";
std::string odom_frame = "odom";

std::queue<MultiLidarData<PointT>> all_cloud_buf;    // 原始点云缓存 
std::queue<MultiLidarData<PointT>> undistorted_cloud_buf;    // 去除畸变后的点云缓存 
std::mutex m_preprocess, m_estimate;

std::unique_ptr<RotaryLidarPreProcess<PointT>> pre_processor;  
std::unique_ptr<MultiLidarEstimatorBase<PointT>> estimator;             // 估计器
std::unique_ptr<ExternalSensorDeskewBase> deskew;  

ros::Publisher pubUndistortPoints;  
std::vector<ros::Publisher> pubLidarFiltered;    // 发布每个激光滤波后的点   直接法时使用 
std::vector<ros::Publisher> pubLidarEdge;    // 发布每个激光提取的边缘特征
std::vector<ros::Publisher> pubLidarSurf;    // 发布每个激光提取的平面特征

std::vector<std::string> pubLidarFiltered_topic = { "filtered_lidar_0", "filtered_lidar_1"};
std::vector<std::string> pubLidarEdge_topic = { "lidar_edge_0", "lidar_edge_1"};
std::vector<std::string> pubLidarSurf_topic = { "lidar_surf_0", "lidar_surf_1"};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitParam(ros::NodeHandle &n) 
{
    config_path = RosReadParam<string>(n, "multiSensorFusion_slam/config_file");  
    NUM_OF_LIDAR = RosReadParam<int>(n, "multiSensorFusion_slam/lidar_num");  
    // 读取激光话题 
    for (uint16_t i = 0; i < lidar_topic_names.size(); i++)
    {
        lidar_topic_container.push_back(RosReadParam<string>(n, lidar_topic_names[i]));  
    }
    readParameters(config_path);   // 读取参数 
    LoamFeatureMultiLidarEstimatorFactory<PointT, PointT> MLE_factory; 
    estimator = MLE_factory.Create(config_path);   // 使用工厂函数构造估计器  
    //deskew = std::unique_ptr<DeskewBase>(new DeskewBase(SCAN_PERIOD_));  
    // 建立预处理器  
    float SCAN_PERIOD = RosReadParam<float>(n, "multiSensorFusion_slam/lidar/scan_period"); 
    int SCAN = RosReadParam<int>(n, "multiSensorFusion_slam/lidar/scan"); 
    pre_processor = std::unique_ptr<RotaryLidarPreProcess<PointT>>(
            new RotaryLidarPreProcess<PointT>(SCAN_PERIOD));  
    if (estimator == nullptr)
    {
        std::cout<<common::RED<<"Estimator construct failure!"<<std::endl;
    }
    else 
    {
        std::cout<<common::GREEN<<"Estimator construct success!"<<std::endl;
    }
}

void initRosPub(ros::NodeHandle &private_nh)
{
    for (uint16_t i = 0; i < NUM_OF_LIDAR; i++)
    {
        ros::Publisher pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarFiltered_topic[i], 10); 
        pubLidarFiltered.push_back(pub);  
        pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarSurf_topic[i], 10); 
        pubLidarSurf.push_back(pub);  
        pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarEdge_topic[i], 10); 
        pubLidarEdge.push_back(pub);  
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<PointT> getCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<PointT> laser_cloud;
    pcl::fromROSMsg(*cloud_msg, laser_cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laser_cloud, laser_cloud, indices);
    return laser_cloud;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief:  接受同一时刻的两个激光数据  已经实现硬件同步
 * @param cloud0_msg lidar_topic_container[0] 话题对应的点云
 * @param cloud1_msg lidar_topic_container[1] 话题对应的点云
 */
void dataProcessCallback_2(const sensor_msgs::PointCloud2ConstPtr &cloud0_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud1_msg)
{
    MultiLidarData<PointT> data;   
    data.timestamp = cloud0_msg->header.stamp.toSec();
    LidarData<PointT> one_lidar_data;
    one_lidar_data.point_cloud = getCloudFromMsg(cloud0_msg); 
    data.all_lidar_data.emplace_back(0, std::move(one_lidar_data));  
    one_lidar_data.point_cloud = getCloudFromMsg(cloud1_msg); 
    data.all_lidar_data.emplace_back(1, std::move(one_lidar_data));  
    m_preprocess.lock();
    all_cloud_buf.push(std::move(data));  
    m_preprocess.unlock();
}

// 单激光的回调  
void dataProcessCallback_1(const sensor_msgs::PointCloud2ConstPtr &cloud0_msg)
{
    MultiLidarData<PointT> data;   
    data.timestamp = cloud0_msg->header.stamp.toSec();
    LidarData<PointT> one_lidar_data;
    one_lidar_data.point_cloud = getCloudFromMsg(cloud0_msg); 
    data.all_lidar_data.emplace_back(0, std::move(one_lidar_data));  
    m_preprocess.lock();
    all_cloud_buf.push(std::move(data));  
    m_preprocess.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 预处理
 * @details:  主要是进行畸变去去除  
 */
void preprocess()
{
    while (1)
    {
        m_preprocess.lock();
        // 激光容器非空 
        if (!all_cloud_buf.empty())
        {
            MultiLidarData<PointT> data = all_cloud_buf.front();
            all_cloud_buf.pop();  
            // 如果有数据    则进行数据处理
            // 1、给出时间戳   2、去除畸变  
            //  openmp加速  
            #pragma omp parallel for num_threads(NUM_OF_LIDAR)
            for (int i = 0; i < data.all_lidar_data.size(); i++)
            {   
                // 先判断是否有时间戳信息           
                pre_processor->Process(data.all_lidar_data[i].second);       
                // 去除畸变  
            }
            m_estimate.lock();
            undistorted_cloud_buf.push(std::move(data));
            m_estimate.unlock();     
            // cout<<"process MultiLidarData, lidar num: "<<data.lidar_data_container_.size()<<std::endl;
        }
        m_preprocess.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}   

/**
 * @brief:  去除畸变后 对点云数据计算里程计    
 */
void Estimate() 
{
    while(1)
    {
        m_estimate.lock(); 
        if (!undistorted_cloud_buf.empty()) 
        {
            MultiLidarData<PointT> data = std::move(undistorted_cloud_buf.front());
            undistorted_cloud_buf.pop();             
            estimator->Process(data);              
        }
        m_estimate.unlock();     
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// 读取估计结果的线程 
void processResult()
{
    while(1)
    {
        MultiLidarResultInfo<PointT> result;     // 本次读取的结果 
        // 获取里程计的结果以及处理完成的点云进行发布
        if (DataManager::GetInstance().GetData<MultiLidarResultInfo<PointT>>("frontend_info", result))
        {  
            // 遍历全部激光雷达  
            for(auto curr_lidar = result.begin(); curr_lidar != result.end(); curr_lidar++)
            {
                uint8_t id = curr_lidar->first;  
                LidarResultInfo<PointT>& curr_lidar_res_info = curr_lidar->second;     
                ros::Time stamp = ros::Time(curr_lidar_res_info.time_stamps_);  
                // 发布点云
                // 特征法  发布特征
                // 遍历全部特征  
                for(auto feature = curr_lidar_res_info.feature_point_.begin(); 
                        feature !=  curr_lidar_res_info.feature_point_.end(); feature++)
                {
                    if (feature->first == "loam_edge")   
                    {
                        publishCloud( &pubLidarEdge[id],    // 发布该点云的话题 
                                                        feature->second,   // 边缘特征   
                                                        stamp, lidar_frame_names[id]);     
                    }
                    else if (feature->first == "loam_surf")
                    {
                        publishCloud( &pubLidarSurf[id],    // 发布该点云的话题 
                                                        feature->second,   // 点云数据   
                                                        stamp, lidar_frame_names[id]);     
                    }
                }
            
                // std::cout<<"pose: "<<std::endl<<result.pose_[i]<<std::endl;
                Eigen::Vector3f p(curr_lidar_res_info.pose_.block<3, 1>(0, 3));  
                Eigen::Quaternionf quat(curr_lidar_res_info.pose_.block<3, 3>(0, 0));
                quat.normalize();
                // 发布tf 
                PublishTF(p, quat, stamp, odom_frame, lidar_frame_names[id]);  
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_lidar_odometry_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    InitParam(nh);  
    initRosPub(private_nh);  
    // ******************************************2个激光雷达的ROS同步设置 
    typedef sensor_msgs::PointCloud2 LidarMsgType;
    typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType> LidarSyncPolicy;
    typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

    std::vector<LidarSubType *> sub_lidar(2);
    NUM_OF_LIDAR = NUM_OF_LIDAR < 2 ? NUM_OF_LIDAR : 2;     // 目前最多支持2个雷达 
    for (size_t i = 0; i < NUM_OF_LIDAR; i++) sub_lidar[i] = new LidarSubType(private_nh, lidar_topic_container[i], 1);
    // for (size_t i = NUM_OF_LIDAR; i < 2; i++) sub_lidar[i] = new LidarSubType(private_nh, CLOUD_TOPIC[0], 1);
    message_filters::Synchronizer<LidarSyncPolicy> *lidar_synchronizer; 
    ros::Subscriber lidar_single;
    if (NUM_OF_LIDAR == 2)
    {
        lidar_synchronizer = new message_filters::Synchronizer<LidarSyncPolicy>(
            LidarSyncPolicy(10), *sub_lidar[0], *sub_lidar[1]);
        lidar_synchronizer->registerCallback(boost::bind(&dataProcessCallback_2, _1, _2));
        pubUndistortPoints = private_nh.advertise<nav_msgs::Odometry>(public_topic, 10); 
    }
    if (NUM_OF_LIDAR == 1)  // 单激光  
    {
        std::cout<<"use single lidar, topic is: "<<lidar_topic_container[USE_SINGLE_LIDAR]<<std::endl;
        lidar_single = private_nh.subscribe(lidar_topic_container[USE_SINGLE_LIDAR], 100, dataProcessCallback_1);
    }
    // 启动预处理线程  
    std::thread preprocess_thread(preprocess);
    // 启动估计线程
    std::thread estimate_thread(Estimate);
    std::thread processResult_thread(processResult);
    ros::spin();
    return 0;
}


