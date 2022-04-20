/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-28 13:27:19
 * @Description:  激光雷达点云预处理模块  ， 主要负责 1、点云数据的畸变去除  2、点云的处理如滤波、提取特征等   
 * @Others:  支持多激光雷达， livox、velodyne等型号雷达  
 */

#include "ros_utils.hpp"
#include "Sensor/sensor.hpp"
#include "Common/color.hpp"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

using namespace std;
using namespace Sensor;  

uint8_t NUM_OF_LASER = 2;   
std::vector<std::string> CLOUD_TOPIC = {"/left/velodyne_points", "/right/velodyne_points"};  
// string lidar_topic_1 = "/left/velodyne_points";
// string lidar_topic_2 = "/right/velodyne_points";
string public_topic = "undistortion_pointcloud";
ros::Publisher pubUndistortPoints;  
// message buffer
// std::vector<std::queue<sensor_msgs::PointCloud2ConstPtr>> all_cloud_buf(2);
std::queue<MultiLidarData> all_cloud_buf; 
std::mutex m_buf;

pcl::PointCloud<PointType> getCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<PointType> laser_cloud;
    pcl::fromROSMsg(*cloud_msg, laser_cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laser_cloud, laser_cloud, indices);
    return laser_cloud;
}

// 接受同一时刻的两个激光数据  已经实现硬件同步
void dataProcessCallback(const sensor_msgs::PointCloud2ConstPtr &cloud0_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud1_msg)
{
    m_buf.lock();
    MultiLidarData data;   
    data.timestamp = cloud0_msg->header.stamp.toSec();
    data.point_clouds_.push_back(getCloudFromMsg(cloud0_msg));   
    data.point_clouds_.push_back(getCloudFromMsg(cloud1_msg));   
    all_cloud_buf.push(std::move(data));  
    m_buf.unlock();
}

void sync_process()
{
    while(1)
    {
        m_buf.lock();
        // 激光容器非空 
        if (!all_cloud_buf.empty())
        {
            MultiLidarData data = all_cloud_buf.front();
            all_cloud_buf.pop();  
            // 如果有数据    则进行数据处理
            // 1、给出时间戳   2、去除畸变    

            // cout<<"process MultiLidarData, lidar num: "<<data.point_clouds_.size()<<std::endl;

        }
        // 剔除陈旧的数据 保证及时性   
        if (all_cloud_buf.size() > 10)
        {
            while (!all_cloud_buf.empty())
            {
                all_cloud_buf.pop();
            }   
            std::cout << common::GREEN << "drop lidar frame in odometry for real time performance"
                      << common::RESET << std::endl;
        }
        m_buf.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_preprocessing_node");
    ros::NodeHandle nh("~");

    // ******************************************2个激光雷达的ROS同步设置 
    typedef sensor_msgs::PointCloud2 LidarMsgType;
    typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType> LidarSyncPolicy;
    typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

    std::vector<LidarSubType *> sub_lidar(2);
    NUM_OF_LASER = NUM_OF_LASER < 2 ? NUM_OF_LASER : 2;     // 目前最多支持2个雷达 
    for (size_t i = 0; i < NUM_OF_LASER; i++) sub_lidar[i] = new LidarSubType(nh, CLOUD_TOPIC[i], 1);
    for (size_t i = NUM_OF_LASER; i < 2; i++) sub_lidar[i] = new LidarSubType(nh, CLOUD_TOPIC[0], 1);
    message_filters::Synchronizer<LidarSyncPolicy> *lidar_synchronizer =
        new message_filters::Synchronizer<LidarSyncPolicy>(
            LidarSyncPolicy(10), *sub_lidar[0], *sub_lidar[1]);
    lidar_synchronizer->registerCallback(boost::bind(&dataProcessCallback, _1, _2));
    pubUndistortPoints = nh.advertise<nav_msgs::Odometry>(public_topic, 10); 
    // 启动处理线程  
    std::thread sync_thread(sync_process);
    ros::spin();
    return 0;
}

