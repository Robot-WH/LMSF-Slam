
#include <ros/ros.h>
#include <iostream>

// ros消息  
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// tf
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "registration.hpp"
#include "ros_utils.hpp"
#include "tic_toc.h"
#include "utility.hpp"

using namespace std;

typedef pcl::PointXYZI PointT;
const int match_dis = 80;  

class lidarOdometry_node
{
private:
    /* data */
    ros::NodeHandle nh; 
    ros::NodeHandle private_nh;  

    ros::Subscriber points_sub;
    ros::Publisher odom_pub;         // 发布/odom话题
    ros::Publisher pubLaserPath;     // 发布轨迹
    // tf发布
    tf::TransformBroadcaster odom_broadcaster;       // 发布 odom -> Lidar tf变换 
    tf::TransformBroadcaster keyframe_broadcaster;   // odom->KF

    // keyframe parameters 关键帧条件
    double keyframe_delta_trans;  // minimum distance between keyframes
    double keyframe_delta_angle;   
    double keyframe_delta_time;   

    // registration validation by thresholding   非法匹配阈值
    bool transform_thresholding;  
    double max_acceptable_trans;  
    double max_acceptable_angle;

    // odometry calculation
    Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe  上一帧与参考关键帧的相对位姿
    Eigen::Matrix4f predict_trans;               // 预测位姿
    Eigen::Matrix4f delta_motion;                // 运动增量 
    // ????????????????????????????????????
    Eigen::Matrix4f keyframe_pose;               // keyframe pose      参考关键帧的位姿
    Eigen::Matrix4f odom;   
    nav_msgs::Path laserPath;                    // 记录轨迹  
    bool frist = true;
    ros::Time keyframe_stamp;                    // keyframe time

    pcl::PointCloud<PointT>::Ptr submap = nullptr;  // keyframe point cloud  用来匹配的参考子图 
    // 子图滑窗
    std::deque<pcl::PointCloud<PointT>::ConstPtr> FramesWin;
    int windows_size = 1;    // 滑窗size  

    boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> registration;      // 常用多线程NDT  
    std::string odom_frame_id;  
    std::string lidar_frame_id;                  

public:
    lidarOdometry_node(/* args */)
    {
        private_nh = ros::NodeHandle("~");
        string pointcloud_topic;  
        pointcloud_topic = private_nh.param<string>("pointcloud_topic", "/processed_points");
        std::cout<<" lidar odom subscribe pointcloud: "<<pointcloud_topic<<std::endl;
        // ROS topics    去除了畸变以及噪声的点云  
        points_sub = private_nh.subscribe(pointcloud_topic, 100, &lidarOdometry_node::cloud_callback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 32); 
        // 发布轨迹   
        pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
        initialize_params();
    }

    ~lidarOdometry_node(){};

    void initialize_params();
    Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);
    void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};


void lidarOdometry_node::initialize_params() {
    // 设置原点坐标系名字
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "/odom");
    lidar_frame_id = private_nh.param<std::string>("lidar_frame_id", "/lidar_odom");
    std::cout<<" lidar odom odom_frame_id: "<<odom_frame_id<<std::endl;
    std::cout<<" lidar odom lidar_frame_id: "<<lidar_frame_id<<std::endl;
    // 关键帧选取 
    keyframe_delta_trans = private_nh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = private_nh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = private_nh.param<double>("keyframe_delta_time", 1.0);
    // 运动求解失败   
    transform_thresholding = private_nh.param<bool>("transform_thresholding", false);
    max_acceptable_trans = private_nh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = private_nh.param<double>("max_acceptable_angle", 1.0);

    windows_size = private_nh.param<int>("windows_size", 10);
    std::cout<<" lidar odom windows_size: "<<windows_size<<std::endl;
    // 设置匹配方法   默认为ndt    
    registration = Set_NDTOMP_param(private_nh);
}


/**
 * @brief estimate the relative pose between an input cloud and a keyframe cloud   这个里程计需要重点优化预测值的给定
 * @param stamp  the timestamp of the input cloud
 * @param cloud  the input cloud
 * @return 当前帧在全局坐标系下的pose
 */
Eigen::Matrix4f lidarOdometry_node::matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) 
{    
    TicToc tt;  
    // 第一帧时
    if(submap == nullptr) 
    {
        submap.reset(new pcl::PointCloud<PointT>());  
        prev_trans.setIdentity();                  // 上一帧变换设置为单位阵
        predict_trans.setIdentity();               // 预测位姿
        delta_motion.setIdentity();  
        keyframe_stamp = stamp;
        // 当前帧加入滑窗 
        *submap = *cloud;
        FramesWin.emplace_back(std::move(cloud));  
        registration->setInputTarget(submap);      // 将关键帧设置为匹配对象
        return Eigen::Matrix4f::Identity();
    }

    tt.tic();  
    // 每次匹配都与参考关键帧进行匹配
    registration->setInputSource(cloud);
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned, predict_trans);                              // 进行配准     predict_trans = setInputSource -> setInputTarget
    float score = registration->getFitnessScore();                             // 获得得分
    vector<double> datas;
    datas.emplace_back(tt.toc("match time:"));  
    datas.push_back(score);  
    // 时间保存  
    SaveDataCsv("/slam_data/time", "/slam_data/time/times_scan_scan.csv", datas, {"time_scan_scan", "score"});

    // 如果迭代没收敛   则该帧忽略    匹配的协方差应该设置很大 
    if(!registration->hasConverged()) 
    {
        ROS_INFO("scan matching has not converged!!");
        cout<<"ignore this frame(" << stamp << ")"<<endl;
        predict_trans = prev_trans*delta_motion*delta_motion;
        return prev_trans*delta_motion;        // 返回预测位姿
    }
    else
    {
        ROS_INFO_STREAM ( "front score: " << score);
        // 认为匹配不好 
        if(score>2)
        {
            // 调整协方差矩阵 

            // ROS_INFO("scan matching bad!!");
            // cout<<"ignore this frame(" << stamp << ")"<<endl;
            // predict_trans = prev_trans*delta_motion*delta_motion;
            // return prev_trans*delta_motion;        // 返回预测位姿 
        }
    }

    // 收敛的话   注意PCL中的T 是B -> W
    Eigen::Matrix4f trans = registration->getFinalTransformation();          // trans为当前帧->参考关键帧的变换矩阵
    // std::cout<<" odom : "<<std::endl<<trans<<std::endl;
    // Tb1w*Twb2 = Tb1b2
    delta_motion = prev_trans.inverse()*trans;                               // 前两帧位姿的增量   用于预测下一帧位姿

    // 判断是否重新设置关键帧
    double delta_trans = delta_motion.block<3, 1>(0, 3).norm();         
    // 旋转矩阵对应 u*theta  对应四元数  e^u*theta/2  = [cos(theta/2), usin(theta/2)]
    Eigen::Quaternionf q_trans(delta_motion.block<3, 3>(0, 0));
    q_trans.normalize();   
    double delta_angle = std::acos(q_trans.w())*2;     // 获得弧度    45度 约等于 0.8  
    double delta_time = (stamp - keyframe_stamp).toSec();

    // 满足关键帧条件
    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) 
    {
        // 变换点云到odom坐标 
        // 执行变换，并将结果保存在新创建的‎‎ transformed_cloud ‎‎中
        pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>());
        pcl::transformPointCloud (*cloud, *transformed_cloud, trans);

        // 更新滑动窗口  
        if(FramesWin.size()>=windows_size)
        {  
           tt.tic();  
           FramesWin.pop_front();
           FramesWin.emplace_back(std::move(transformed_cloud));
           submap->clear();   
           // 更新submap  
           for(deque<pcl::PointCloud<PointT>::ConstPtr>::const_iterator it = FramesWin.begin(); it != FramesWin.end(); it++)
           {
             *submap += **it;   
           }
           //tt.toc("map slidingwindow: ");  
        }
        else  //滑窗没有满
        {
           *submap += *transformed_cloud;      
           FramesWin.emplace_back(std::move(transformed_cloud));   // std::move后变成右值后会调用移动构造函数， transformed_cloud的内容会被转移  
        }
        
        // 降采样 
        if(FramesWin.size() >= 5)
        {
        }

        tt.tic();  
        registration->setInputTarget(submap);   // 设定为匹配目标
        //tt.toc("map construct: ");  
        keyframe_stamp = stamp;
        // 记录
        prev_trans = trans;   
    }

    // 根据匀速运动假设进行预测    
    predict_trans = trans*delta_motion;         // Twb1*Tb1b2 = Twb2
    
    // 返回当前帧的世界坐标 
    return trans;
}    


/**
 * @brief publish odometry
 * @param stamp  timestamp
 * @param pose   odometry pose to be published
 */
void lidarOdometry_node::publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) 
{
    // publish the transform                发布当前帧里程计到/odom话题                 
    nav_msgs::Odometry odom;                            
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;       // odom坐标    /odom
    odom.child_frame_id = lidar_frame_id;        // /lidar_odom
    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    
    // 旋转矩阵 -> 四元数
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    // 构造四元数   ROS信息
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();
    odom.pose.pose.orientation = odom_quat;  
    odom_pub.publish(odom);
    // 发布轨迹  
    geometry_msgs::PoseStamped laserPose;    
    laserPose.header = odom.header;
    laserPose.pose = odom.pose.pose;                // 和laserOdometry的pose相同  
    laserPath.header.stamp = odom.header.stamp;
    laserPath.poses.push_back(laserPose);
    laserPath.header.frame_id = odom_frame_id;      // odom坐标     /odom
    pubLaserPath.publish(laserPath);

    // 发布TF
	static tf::TransformBroadcaster br;
	tf::Transform transform;
    tf::Quaternion q;
	transform.setOrigin(tf::Vector3(pose(0, 3),
									pose(1, 3),
									pose(2, 3)));
    q.setW(quat.w());                               
	q.setX(quat.x());
	q.setY(quat.y());
	q.setZ(quat.z());    

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, stamp, odom_frame_id, lidar_frame_id));
}


void lidarOdometry_node::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
    // 转成PCL格式
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // 匹配
    TicToc t_pub;
    Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);
    printf("odometry time %f ms \n", t_pub.toc());
    // 发布话题     lidar_frame_id =   /lidar_odom
    publish_odometry(cloud_msg->header.stamp, pose);
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "lidarOdometry_node");   
    ROS_INFO("Started lidarOdometry_node node");  
    lidarOdometry_node lidar_odometry;
    ros::spin(); 
    return 0;
}



