
/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: lwh
 * @Version: 1.0
 * @Description:  GNSS-IMU-LIDAR 滤波器融合的SLAM系统
 * @Others: 
 */

#include "ros_utils.hpp"
#include "Sensor/sensor.hpp"
#include "Sensor/lidar_data_type.h"
#include "Common/color.hpp"
#include "Common/parameters.h"
#include "Common/keyframe.hpp"
#include "Common/data_manager.hpp"
#include "factory/System/LIG_SystemFactory.hpp"

using namespace std;
using namespace Algorithm;  
using namespace Slam3D;  
using namespace common; 

using PointT = pcl::PointXYZI;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;  
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;  
using PointCloud = pcl::PointCloud<PointT>;  

string config_path;     // 算法参数文件path 
int NUM_OF_LIDAR;   
// 多激光话题在参数文件中的索引名称  
std::vector<std::string> lidar_topic_names = {"lidar_topic_0",
                                                                                              "lidar_topic_1" };  
std::vector<std::string> lidar_frame_names = {"lidar_0", "lidar_1" };            // 多激光坐标                                                                                          
std::vector<std::string> lidar_topic_container;   // 多激光话题名称容器  
string imu_topic, gnss_topic;
string public_topic = "undistortion_pointcloud";
std::string odom_frame = "odom";

std::unique_ptr<LidarImuGnssFusionSystem<PointT, PointT>> System;             // 估计器

ros::Publisher pubUndistortPoints;  
std::vector<ros::Publisher> pubLidarFiltered;    // 发布每个激光滤波后的点   直接法时使用 
std::vector<ros::Publisher> pubLidarEdge;    // 发布每个激光提取的边缘特征
std::vector<ros::Publisher> pubLidarSurf;    // 发布每个激光提取的平面特征
std::vector<ros::Publisher> pubLocalMapFiltered;    // 发布每个激光滤波后的点   直接法时使用 
std::vector<ros::Publisher> pubLocalMapEdge;    // 发布每个激光提取的边缘特征
std::vector<ros::Publisher> pubLocalMapSurf;    // 发布每个激光提取的平面特征
ros::Publisher markers_pub; // 可视化
ros::ServiceServer save_data_server;   // 数据保存服务
ros::ServiceServer save_map_server;  // 地图保存服务 
// 发布话题名称  
std::vector<std::string> pubLidarFiltered_topic = { "filtered_lidar_0", "filtered_lidar_1"};
std::vector<std::string> pubLidarEdge_topic = { "lidar_edge_0", "lidar_edge_1"};
std::vector<std::string> pubLidarSurf_topic = { "lidar_surf_0", "lidar_surf_1"};
std::vector<std::string> pubLocalMapFiltered_topic = { "LocalMap_filtered_0", "LocalMap_filtered_1"};
std::vector<std::string> pubLocalMapEdge_topic = { "LocalMap_edge_0", "LocalMap_edge_1"};
std::vector<std::string> pubLocalMapSurf_topic = { "LocalMap_surf_0", "LocalMap_surf_1"};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitParam(ros::NodeHandle &n) 
{
    // 算法配置文件路径
    config_path = RosReadParam<string>(n, "config_path");  
    NUM_OF_LIDAR = RosReadParam<int>(n, "lidar_num");  
    // 读取激光话题 
    for (uint16_t i = 0; i < NUM_OF_LIDAR; i++) {
        lidar_topic_container.push_back(RosReadParam<string>(n, lidar_topic_names[i]));  
    }
    // 读取 IMU、GNSS话题 
    imu_topic = RosReadParam<string>(n, "imu_topic");
    gnss_topic = RosReadParam<string>(n, "gnss_topic");
    ROS_WARN("imu_topic:");
    std::cout<<imu_topic<<std::endl;
    ROS_WARN("gnss_topic:");
    std::cout<<gnss_topic<<std::endl;
    LidarImuGnssSystemFactory<PointT, PointT> System_factory; 
    System = System_factory.Create(config_path);   // 使用工厂函数构造估计器  
    if (System == nullptr) {
        std::cout<<common::RED<<"System construct failure!"<<std::endl;
    } else {
        std::cout<<common::GREEN<<"System construct success!"<<std::endl;
    }
}

bool SaveDataService(liv_slam::SaveDataRequest& req, liv_slam::SaveDataResponse& res);
bool SaveMapService(liv_slam::SaveMapRequest& req, liv_slam::SaveMapResponse& res);

void RosInit(ros::NodeHandle &private_nh)
{
    for (uint16_t i = 0; i < NUM_OF_LIDAR; i++)
    {
        ros::Publisher pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarFiltered_topic[i], 10); 
        pubLidarFiltered.push_back(pub);  
        pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarSurf_topic[i], 10); 
        pubLidarSurf.push_back(pub);  
        pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarEdge_topic[i], 10); 
        pubLidarEdge.push_back(pub);  

        pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLocalMapFiltered_topic[i], 10); 
        pubLocalMapFiltered.push_back(pub);  
        pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLocalMapEdge_topic[i], 10); 
        pubLocalMapEdge.push_back(pub);  
        pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLocalMapSurf_topic[i], 10); 
        pubLocalMapSurf.push_back(pub);  
    }
    markers_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/graph_markers", 10);        // 可视化
    save_data_server = private_nh.advertiseService("/SaveGraph", &SaveDataService);
    save_map_server = private_nh.advertiseService("/liv_slam/save_map", &SaveMapService);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SaveDataService(liv_slam::SaveDataRequest& req, liv_slam::SaveDataResponse& res) 
{
    std::string directory = req.destination;
    System->SavePoseGraph();  
    std::cout<<"Save Graph Data success! "<<std::endl;
    res.success = true; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SaveMapService(liv_slam::SaveMapRequest& req, liv_slam::SaveMapResponse& res) 
{
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
void dataProcessCallbackTwo(const sensor_msgs::PointCloud2ConstPtr &cloud0_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud1_msg)
{
    MultiLidarData<PointT> data;   
    data.timestamp = cloud0_msg->header.stamp.toSec();
    LidarData<PointT> one_lidar_data;
    one_lidar_data.point_cloud = getCloudFromMsg(cloud0_msg); 
    data.all_lidar_data.emplace_back(0, std::move(one_lidar_data));  
    one_lidar_data.point_cloud = getCloudFromMsg(cloud1_msg); 
    data.all_lidar_data.emplace_back(1, std::move(one_lidar_data));  
    System->Process(data);     // 传入 
}

// 单激光的回调  
void dataProcessCallbackOne(const sensor_msgs::PointCloud2ConstPtr &cloud0_msg)
{
    MultiLidarData<PointT> data;   
    data.timestamp = cloud0_msg->header.stamp.toSec();
    LidarData<PointT> one_lidar_data;
    one_lidar_data.point_cloud = getCloudFromMsg(cloud0_msg); 
    data.all_lidar_data.emplace_back(0, std::move(one_lidar_data));  
    System->Process(data);     // 传入 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imuHandler(sensor_msgs::ImuConstPtr const& imu_msg) 
{
    // 保证队列中 数据的顺序正确 
    static double last_imu_t = -1; 
    if (imu_msg->header.stamp.toSec() <= last_imu_t) {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    // 解析IMU数据 
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
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

    // m_buf.lock();
    // imu_buf.push(imu_data_ptr);
    // m_buf.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gnssHandler(sensor_msgs::NavSatFixConstPtr const& navsat_msg) 
{
    // 保证队列中 数据的顺序正确 
    static double last_gnss_t = -1; 
    if (navsat_msg->header.stamp.toSec() <= last_gnss_t) {
        ROS_WARN("gnss message in disorder!");
        return;
    }
    last_gnss_t = navsat_msg->header.stamp.toSec();
    // 解析Gnss数据 
    GnssDataPtr gnss_data_ptr = std::make_shared<GnssData>();
    // 保存时间戳 
    gnss_data_ptr->timestamp = navsat_msg->header.stamp.toSec();
    gnss_data_ptr->lla << navsat_msg->latitude,
                        navsat_msg->longitude,
                        navsat_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(navsat_msg->position_covariance.data());

    // m_buf.lock();
    // gnss_buf.push(gnss_data_ptr);
    // m_buf.unlock();
}

Eigen::Isometry3d trans_odom2map = Eigen::Isometry3d::Identity();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 根据图数据 创建可视化的图结构 
 */
template<typename _PointT>
visualization_msgs::MarkerArray createMarkerArray(KeyFrameInfo<PointT> const& info) 
{
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(4);
    ros::Time stamp(info.time_stamps_);  
    // node markers    位姿节点
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    // std::cout<<"info.vertex_database_.size(): "<<info.vertex_database_.size()<<std::endl;
    // std::cout<<"info.edge_database_.size(): "<<info.edge_database_.size()<<std::endl;
    // std::cout<<"info.new_keyframes_.size(): "<<info.new_keyframes_.size()<<std::endl;
    // 数量
    traj_marker.points.resize(info.vertex_database_.size() + 
                                                            info.new_keyframes_.size());
    // 颜色
    traj_marker.colors.resize(info.vertex_database_.size() + 
                                                            info.new_keyframes_.size());
    // 新增位姿节点
    for(int i=0; i<info.new_keyframes_.size(); i++) 
    {
        // 设置位置
        Eigen::Vector3d pos = (trans_odom2map * info.new_keyframes_[i].odom_).translation();
        traj_marker.points[i].x = pos.x();
        traj_marker.points[i].y = pos.y();
        traj_marker.points[i].z = pos.z();
        // 颜色
        traj_marker.colors[i].r = 1.0;
        traj_marker.colors[i].g = 0;
        traj_marker.colors[i].b = 0.0;
        traj_marker.colors[i].a = 1.0;
    }   
    // 关键帧数据库的位姿节点 
    for (int i = 0; i < info.vertex_database_.size(); i++) 
    {
        // 设置位置
        Eigen::Vector3d pos = info.vertex_database_[i].pose_.translation();
        traj_marker.points[info.new_keyframes_.size() + i].x = pos.x();
        traj_marker.points[info.new_keyframes_.size() + i].y = pos.y();
        traj_marker.points[info.new_keyframes_.size() + i].z = pos.z();
        // 颜色
        traj_marker.colors[info.new_keyframes_.size()+i].r = 0;
        traj_marker.colors[info.new_keyframes_.size()+i].g = 0;
        traj_marker.colors[info.new_keyframes_.size()+i].b = 1.0;
        traj_marker.colors[info.new_keyframes_.size()+i].a = 1.0;
    }   
    // edge markers  边
    visualization_msgs::Marker& edge_marker = markers.markers[2];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.1;
    // 这里要注意 ！！！！
    edge_marker.points.resize(info.edge_database_.size() * 2); 
    edge_marker.colors.resize(info.edge_database_.size() * 2); 
    int i=0;

    for (int num = 0; num < info.edge_database_.size(); num++) 
    {
        // 里程计边    Pc
        // Eigen::Vector3d pt1 = info.keyframe_database_[num].correct_pose_.translation();
        // // Twc*Tlc^-1 = Twl
        // Eigen::Vector3d pt2 = (info.keyframe_database_[num].correct_pose_ 
        //     * info.keyframe_database_[num].between_constraint_.inverse()).translation();
        Eigen::Vector3d pt1 = info.vertex_database_[info.edge_database_[num].link_id_.first].pose_.translation();
        // Twc*Tlc^-1 = Twl
        Eigen::Vector3d pt2 = (info.vertex_database_[info.edge_database_[num].link_id_.first].pose_
            * info.edge_database_[num].constraint_).translation();
        // 设置位置关系     每个frame 2个点 
        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].g = 2.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].g = 2.0;
        edge_marker.colors[i*2 + 1].a = 1.0;
        i++;
        
        // if(enable_GNSS_optimize)
        // {
        //     // GNSS 先验边     2个点  
        //     if(keyframes[num]->GNSS_Valid) {
        //         Eigen::Vector3d pt1 = keyframes[num]->Pose.translation();  
        //         Eigen::Vector3d pt2 = keyframes[num]->utm_coord;

        //         edge_marker.points[i*2].x = pt1.x();
        //         edge_marker.points[i*2].y = pt1.y();
        //         edge_marker.points[i*2].z = pt1.z()+1;
        //         edge_marker.points[i*2 + 1].x = pt2.x();
        //         edge_marker.points[i*2 + 1].y = pt2.y();
        //         edge_marker.points[i*2 + 1].z = pt2.z();

        //         edge_marker.colors[i*2].r = 1.0;
        //         edge_marker.colors[i*2].a = 1.0;
        //         edge_marker.colors[i*2 + 1].r = 1.0;
        //         edge_marker.colors[i*2 + 1].a = 1.0;
        //         i++;
        //     }
        // }
        // 地面约束边  
        // if (enable_planeConstraint_optimize)
        // {
        //     if (keyframe_database[num]->planeConstraint_Valid) 
        //     {
        //         Eigen::Vector3d plane = {pt1[0], pt1[1], 0};

        //         edge_marker.points[i*2].x = pt1.x();
        //         edge_marker.points[i*2].y = pt1.y();
        //         edge_marker.points[i*2].z = pt1.z();
        //         edge_marker.points[i*2 + 1].x = plane.x();
        //         edge_marker.points[i*2 + 1].y = plane.y();
        //         edge_marker.points[i*2 + 1].z = plane.z()-1;

        //         edge_marker.colors[i*2].r = 1.0;
        //         edge_marker.colors[i*2].a = 2.0;
        //         edge_marker.colors[i*2 + 1].r = 1.0;
        //         edge_marker.colors[i*2 + 1].a = 2.0;
        //         i++;
        //     }
        // }
    }
    // // 回环检测的边   2个点 
    // for (auto const& loop:info.loops_)
    // { 
    //     Eigen::Vector3d pt1 = info.keyframe_database_[loop.id_1_].correct_pose_.translation(); // 新帧  
    //     Eigen::Vector3d pt2 = (info.keyframe_database_[loop.id_1_].correct_pose_
    //                                                     * loop.relative_pose_).translation();     // 与新帧闭环的老帧   Twc * Tlc^-1

    //     edge_marker.points[i*2].x = pt1.x();
    //     edge_marker.points[i*2].y = pt1.y();
    //     edge_marker.points[i*2].z = pt1.z();
    //     edge_marker.points[i*2 + 1].x = pt2.x();
    //     edge_marker.points[i*2 + 1].y = pt2.y();
    //     edge_marker.points[i*2 + 1].z = pt2.z();

    //     edge_marker.colors[i*2].r = 2.0;
    //     edge_marker.colors[i*2].a = 2.0;
    //     edge_marker.colors[i*2 + 1].r = 2.0;
    //     edge_marker.colors[i*2 + 1].a = 2.0;
    //     i++;
    // }
    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "Odom Error";
    sphere_marker.id = 3;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    if (!info.new_keyframes_.empty()) 
    {
        Eigen::Vector3d pos = (trans_odom2map * info.new_keyframes_.back().odom_).translation();
        sphere_marker.pose.position.x = pos.x();
        sphere_marker.pose.position.y = pos.y();
        sphere_marker.pose.position.z = pos.z();

        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 100;

        sphere_marker.color.r = 1.0;
        sphere_marker.color.a = 0.3;
    }
    return markers;
}
// 读取估计结果的线程 
void processResult()
{
    while(1)
    {
        MultiLidarResultInfo<PointT> result;     // 本次读取的结果 
        // odom->map的变换
        DataManager::GetInstance().GetData("odom_to_map", trans_odom2map);
        // 获取里程计的结果以及处理完成的点云进行发布
        if (DataManager::GetInstance().GetData<MultiLidarResultInfo<PointT>>("frontend_info", result))
        {  
            // 遍历全部激光雷达  
            for (auto curr_lidar = result.begin(); curr_lidar != result.end(); curr_lidar++)
            {
                uint8_t id = curr_lidar->first;  
                LidarResultInfo<PointT>& curr_lidar_res_info = curr_lidar->second;     
                ros::Time stamp = ros::Time(curr_lidar_res_info.time_stamps_);  
                // 发布点云
                for(auto feature = curr_lidar_res_info.feature_point_.begin(); 
                        feature !=  curr_lidar_res_info.feature_point_.end(); feature++)
                {
                    // std::cout<<"name: "<<feature->first<<std::endl;
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
                    else if (feature->first == POINTS_PROCESSED_NAME)    
                    {
                        if (pubLidarFiltered[id].getNumSubscribers() != 0) 
                        {
                            publishCloud(&pubLidarFiltered[id],    // 发布该点云的话题 
                                feature->second,   // 点云数据   
                                stamp, lidar_frame_names[id]);     
                        }
                    }
                }
                // // 发布local map 
                // for(auto iter = curr_lidar_res_info.local_map_.begin(); 
                //         iter !=  curr_lidar_res_info.local_map_.end(); iter++)
                // {
                //     // std::cout<<"name: "<<iter->first<<std::endl;
                //     if (iter->first == "loam_edge")   
                //     {
                //         publishCloud( &pubLocalMapEdge[id],    // 发布该点云的话题 
                //                                         iter->second,   // 边缘特征   
                //                                         stamp, odom_frame);     
                //     }
                //     else if (iter->first == "loam_surf")
                //     {
                //         publishCloud( &pubLocalMapSurf[id],    // 发布该点云的话题 
                //                                         iter->second,   // 点云数据   
                //                                         stamp, odom_frame);     
                //     }
                //     else if (iter->first == "filtered")     // 滤波后的
                //     {
                //         publishCloud( &pubLocalMapFiltered[id],    // 发布该点云的话题 
                //                                         iter->second,   // 点云数据   
                //                                         stamp, odom_frame);     
                //     }
                // }
                // std::cout<<"pose: "<<std::endl<<result.pose_[i]<<std::endl;
                // 发过来的pose是相对于odom系的   将他转到map系下 
                curr_lidar_res_info.pose_ = trans_odom2map.matrix().cast<float>() 
                                                                            * curr_lidar_res_info.pose_; 
                Eigen::Vector3f p(curr_lidar_res_info.pose_.block<3, 1>(0, 3));  
                Eigen::Quaternionf quat(curr_lidar_res_info.pose_.block<3, 3>(0, 0));
                quat.normalize();
                // 发布tf 
                PublishTF(p, quat, stamp, "map", lidar_frame_names[id]); 
                //PublishTF(p, quat, stamp, odom_frame, lidar_frame_names[id]); 
            }
        }
        // 图优化可视化
        if (markers_pub.getNumSubscribers()) 
        {
            // 读取关键帧信息 
            KeyFrameInfo<PointT> info;
            if (DataManager::GetInstance().GetData<KeyFrameInfo<PointT>>("keyframes_info", info)) 
            {
                // 发布 markers
                auto markers = createMarkerArray<PointT>(info);
                markers_pub.publish(markers);
                
                // Eigen::Vector3f p(trans_odom2map.inverse().cast<float>().translation());  
                // Eigen::Quaternionf quat(trans_odom2map.inverse().cast<float>().rotation());
                // quat.normalize();
                // // 发布tf 
                // PublishTF(p, quat, ros::Time(info.time_stamps_), odom_frame, "map"); 
            }
            else
            {
                //std::cout<<common::RED<<"get KeyFrameInfo error"<<common::RESET<<std::endl;
            }
        }  
        
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
    }; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MultiSensorFilterSLAM_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    InitParam(nh);  
    RosInit(private_nh);  
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
    if (NUM_OF_LIDAR == 2)   // 多激光 
    {
        lidar_synchronizer = new message_filters::Synchronizer<LidarSyncPolicy>(
            LidarSyncPolicy(10), *sub_lidar[0], *sub_lidar[1]);
        lidar_synchronizer->registerCallback(boost::bind(&dataProcessCallbackTwo, _1, _2));
        pubUndistortPoints = private_nh.advertise<nav_msgs::Odometry>(public_topic, 10); 
    }

    if (NUM_OF_LIDAR == 1)  // 单激光  
    {
        std::cout<<"use single lidar, topic is: "<<lidar_topic_container[0]<<std::endl;
        lidar_single = private_nh.subscribe(lidar_topic_container[0], 100, dataProcessCallbackOne);
    }
    ros::Subscriber subImu;                             // IMU
    ros::Subscriber subGnss;                           // gnss    提供真值用于比较      
    subImu = private_nh.subscribe(imu_topic, 1000, &imuHandler,
                                                    ros::TransportHints().tcpNoDelay());
    subGnss = private_nh.subscribe(gnss_topic, 1000, &gnssHandler, 
                                                    ros::TransportHints().tcpNoDelay());  
    std::thread processResult_thread(processResult);
    ros::spin();
    return 0;
}



