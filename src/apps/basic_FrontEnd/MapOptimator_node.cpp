
#include <ros/ros.h>
#include <iostream>
#include <queue>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <mutex>
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


/**目标：建立一个局部地图 对odometry的结果进行粗略的校正 
 * 方法： 求取原odom坐标系->校正odom坐标系 的转换矩阵 
 * **********************/

using namespace std;

typedef pcl::PointXYZI PointT;
const int windows_size = 20;              // 滑动窗口尺寸  

int Optimize_freq = 4;                // 每4帧优化一次  
bool full = false;
int freq_Count = 1;

// 当前点云 
pcl::PointCloud<PointT>::Ptr lidarCloud(new pcl::PointCloud<PointT>());
// 当前里程计发送过来的数据   
Eigen::Quaternionf q_odom_curr(1, 0, 0, 0);
Eigen::Vector3f t_odom_curr(0, 0, 0);
Eigen::Matrix4f Pose_odom_curr = Eigen::Matrix4f::Identity();

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// 原始odom到优化后的odom的变换 
Eigen::Quaternionf q_opt_odom(1, 0, 0, 0);
Eigen::Vector3f t_opt_odom(0, 0, 0);
Eigen::Matrix4f T_opt_odom = Eigen::Matrix4f::Identity();

// 转换到opt-odom的Pose 
Eigen::Quaternionf q_opt_curr(1, 0, 0, 0);
Eigen::Vector3f t_opt_curr(0, 0, 0);
Eigen::Matrix4f Pose_opt_curr = Eigen::Matrix4f::Identity();
 
// 滑动窗口    windows_size 必须要是 const   
// PointT Frames[windows_size];
int Frame_count = 0;
pcl::PointCloud<PointT>::Ptr FramesWin[windows_size];

Eigen::Matrix4f T_win_last = Eigen::Matrix4f::Identity();      // 滑窗最后一帧的位姿

// 点云接收队列
queue<sensor_msgs::PointCloud2ConstPtr> PointsBuf;
// 里程计接收队列 
queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
mutex mBuf;
//  用于匹配的局部地图  
pcl::PointCloud<PointT>::Ptr CloudLocalMap(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr testMap(new pcl::PointCloud<PointT>());
// 匹配算法
boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> registration;      // 常用多线程NDT 

double keyframe_delta_trans;  // minimum distance between keyframes
double keyframe_delta_angle; 
string odom_frame_id;
string optimized_lidar_frame_id;

nav_msgs::Path laserAfterMappedPath;


// odometry 数据订阅   
ros::Subscriber subLaserOdometry ;
// Todo: 接收去畸变后的点云
ros::Subscriber points_sub;
// 高频的优化后odom位姿  这个主要用于状态估计
ros::Publisher pubOdomAftMappedHighFrec;
// 优化后输出的低频位姿    主要用于建图 
ros::Publisher pubOdomAftMapped;
// 路径
ros::Publisher pubLaserAfterMappedPath;
// 局部地图
ros::Publisher pubLaserCloudSurround;  
// 当前点云
ros::Publisher pubLaserCloud;


// set initial guess  设置优化的初始值    将odom的位姿转换为优化odom的位姿      To'o*Tob = To'b   
// 即使用odometry的结果作为map匹配的初始值   
void transformAssociateToMap()
{
    // 先求出四元数表示 
    q_opt_curr = q_opt_odom * q_odom_curr;
	  t_opt_curr = q_opt_odom * t_odom_curr + t_opt_odom;
    // odom下的位姿转换成矩阵描述 
    Pose_opt_curr.block<3,3>(0,0) = q_opt_curr.toRotationMatrix();
    Pose_opt_curr.block<3,1>(0,3) = t_opt_curr;
}

// q_w_curr   t_w_curr是指优化的结果  
// q_wodom_curr   t_wodom_curr 是指odometry结果  
void Update()
{
    // 将优化后的坐标Pose_opt_curr 转换为四元数  
    Eigen::Quaternionf q(Pose_opt_curr.block<3,3>(0,0));
    q.normalize();
    q_opt_curr = q;
    t_opt_curr = Pose_opt_curr.block<3,1>(0,3);
    // 更新转换矩阵   To'o = To'b*Tob^-1 
    q_opt_odom = q_opt_curr * q_odom_curr.inverse();         // 求出Rwo 
	  t_opt_odom = t_opt_curr - q_opt_odom * t_odom_curr;      // 求出Two
}

// 点云转移到Map坐标下
void TransformClouds(pcl::PointCloud<PointT>::Ptr Clouds)
{
    for(PointT point:Clouds->points)   
    {
        Eigen::Vector3f point_curr(point.x, point.y, point.z);
        // Twi = Twb*Tbi
        Eigen::Vector3f point_w = q_opt_curr * point_curr + t_opt_curr;
        point.x = point_w.x();
        point.y = point_w.y();
        point.z = point_w.z();
    }
}

// 点云转移到Map坐标下
void pointAssociateToMap(PointT const *const pi, PointT *const po)
{
  Eigen::Vector3f point_curr(pi->x, pi->y, pi->z);
	// Twi = Twb*Tbi
	Eigen::Vector3f point_w = q_opt_curr * point_curr + t_opt_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}

//receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(laserOdometry);
	mBuf.unlock();
    
	// high frequence publish   将接收到的odom数据 根据map优化后的结果进行修正后发出   
	Eigen::Quaternionf q_odom_curr;
	Eigen::Vector3f t_odom_curr;
	// 接收odom发送过来的odom信息
	q_odom_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_odom_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_odom_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_odom_curr.w() = laserOdometry->pose.pose.orientation.w;
	t_odom_curr.x() = laserOdometry->pose.pose.position.x;
	t_odom_curr.y() = laserOdometry->pose.pose.position.y;
	t_odom_curr.z() = laserOdometry->pose.pose.position.z;
  // 直接将laser_odometry的结果乘以优化结果   Two*Tob
	Eigen::Quaternionf q_opt_curr = q_opt_odom * q_odom_curr;
	Eigen::Vector3f t_opt_curr = q_opt_odom * t_odom_curr + t_opt_odom; 
  // 将校正后的结果输出   
	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = odom_frame_id;    
	odomAftMapped.child_frame_id = optimized_lidar_frame_id;    // 重新设置优化后的坐标系  
	odomAftMapped.header.stamp = laserOdometry->header.stamp;
	odomAftMapped.pose.pose.orientation.x = q_opt_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_opt_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_opt_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_opt_curr.w();
	odomAftMapped.pose.pose.position.x = t_opt_curr.x();
	odomAftMapped.pose.pose.position.y = t_opt_curr.y();
	odomAftMapped.pose.pose.position.z = t_opt_curr.z();
	pubOdomAftMappedHighFrec.publish(odomAftMapped);
	// 发布TF
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	transform.setOrigin(tf::Vector3(t_opt_curr(0),
									t_opt_curr(1),
									t_opt_curr(2)));
	q.setW(q_opt_curr.w());                               
	q.setX(q_opt_curr.x());
	q.setY(q_opt_curr.y());
	q.setZ(q_opt_curr.z());
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, odom_frame_id, optimized_lidar_frame_id));
}

// 这里获得的激光点云  要是去除了畸变后的 !!!!
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
  // 当滑动窗口填满后   每2Hz添加添加一次  这个是为了限制最高频率 
  if(full)
  {
    if(freq_Count<Optimize_freq)  
    {
        freq_Count++;
        return;
    }
    else
    {
      freq_Count=1;  
    }      
  }

  mBuf.lock();
	PointsBuf.push(cloud_msg);   
	mBuf.unlock();    
}

void MapOptimize()
{
  while(1)
  { 
    // 获取点云数据以及里程计数据  同时进行对齐 
    while(!PointsBuf.empty() && !odometryBuf.empty())  
    {  
       mBuf.lock();      // 上锁  此时 回调函数阻塞   
       // 如果 点云数据队首早于odometry数据队首   则点云数据队首丢弃 
       while(!PointsBuf.empty()&&odometryBuf.front()->header.stamp.toSec()>PointsBuf.front()->header.stamp.toSec())
         PointsBuf.pop();  

       if(PointsBuf.empty()) 
       {
          mBuf.unlock();    
          break;
       }

       while(!odometryBuf.empty()&&odometryBuf.front()->header.stamp.toSec()<PointsBuf.front()->header.stamp.toSec())
         odometryBuf.pop();

       if(odometryBuf.empty())  
       {  
         mBuf.unlock();
         break;
       }

       // 最后再检查一下对弃没有 
       if(odometryBuf.front()->header.stamp.toSec()!=PointsBuf.front()->header.stamp.toSec())
       {
        ROS_INFO_STREAM("Map Optimize error! data unsync");
        mBuf.unlock();
        break;
       }

       // 获取点云数据     这个点云数据 是由数据处理节点发送过来的  去除畸变 + 滤波的点云   用于与地图匹配
       lidarCloud->clear();
       pcl::fromROSMsg(*PointsBuf.front(), *lidarCloud);
       PointsBuf.pop();
        double timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
        // 获取里程计的数据放置与 q_wodom_curr t_wodom_curr
        q_odom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
        q_odom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
        q_odom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
        q_odom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
        t_odom_curr.x() = odometryBuf.front()->pose.pose.position.x;
        t_odom_curr.y() = odometryBuf.front()->pose.pose.position.y;
        t_odom_curr.z() = odometryBuf.front()->pose.pose.position.z;
        odometryBuf.pop();

       // 将 PointsBuf 清空   保证下一次处理时  是最新的数据 
       while(!PointsBuf.empty())
       {
         PointsBuf.pop();    
       }

       mBuf.unlock();
       // 将odom转到Map系中
       transformAssociateToMap();   

       // 如果滑窗填满则判断运动  
       if(full)
       {
        // 判断与滑动窗口上一帧的位移是否足够大    足够大则需要匹配优化        Tb0o'*To'b1 = Tb0b1
        Eigen::Matrix4f delta_move = T_win_last.inverse()*Pose_opt_curr;     
        // 检查
        double dx = delta_move.block<3, 1>(0, 3).norm();                                  // 平移量

        // 旋转矩阵对应 u*theta  对应四元数  e^u*theta/2  = [cos(theta/2), usin(theta/2)]
        Eigen::Quaternionf q_a(delta_move.block<3, 3>(0, 0));
        q_a.normalize();   
        double da = std::acos(q_a.w())*2;     // 获得弧度    45度 约等于 0.8  
        
        if(dx<keyframe_delta_trans&&da<keyframe_delta_angle)
        {
            break;
        }    
       }

       /************************************************************ 地图匹配校正 ***********************************************/
       // 如果滑动窗口存在点云  则进行匹配 
       if(Frame_count)
       {  
          TicToc t_pub;
          // 构造匹配地图
          CloudLocalMap.reset(new pcl::PointCloud<PointT>());

          for(int i=0;i<Frame_count;i++)
          {
            *CloudLocalMap += *FramesWin[i];
          }
          t_pub.tic();  
          registration->setInputTarget(CloudLocalMap);   
          registration->setInputSource(lidarCloud);    
          pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());  
          registration->align(*aligned, Pose_opt_curr);         // 将之前直接转到Map系的位姿Pose_opt_curr作为预测  

          // 如果迭代没收敛   则该帧忽略  
          if(!registration->hasConverged()) 
          {
            ROS_INFO("Map matching has not converged!!!!!");
            break;
          }
          else
          {
            float ndt_score = registration->getFitnessScore();   // 获得得分
            ROS_INFO_STREAM ( "Map matching converged "<<" front score: " << ndt_score);  
            if(ndt_score>1)
            {
              // 匹配不佳  
            }  
          }

          Pose_opt_curr = registration->getFinalTransformation();       // 优化结果  
          Update();                 // 将Pose_opt_curr转换成四元数    同时  更新转换矩阵   
          vector<double> datas;
          datas.emplace_back(t_pub.toc("submap match time:"));  
          // 时间保存  
          SaveDataCsv("/slam_data/time", "/slam_data/time/times_scan_map.csv", datas, {"time_scan_map"});
       }  

       /*************************************** 更新全局地图     只有经过优化 才能添加到全局地图中 **********************************************/
       // 将当前帧转到优化后odom中
       PointT point;
       int laserCloudNum = lidarCloud->points.size();
   
       for (int i = 0; i < laserCloudNum; i++)
       {
          pointAssociateToMap(&lidarCloud->points[i], &point);
          FramesWin[Frame_count]->push_back(point);
       }

       // 如果滑窗满了   则窗口滑动
       if(full)
       {  
         for(int i=0; i<windows_size-1; i++)
         {
           FramesWin[i] = FramesWin[i+1];  
         }

         FramesWin[Frame_count].reset(new pcl::PointCloud<PointT>());
       }
       else
       { 
         if(Frame_count>=windows_size-2)   
         {         
            full = true;        // 填满    
         }

         Frame_count++;          
       }

       //保存窗口最后一帧的位姿 
        T_win_last = Pose_opt_curr;
        /************************************************************* 发布信息 ***************************************************************/
        // 发布局部地图
        sensor_msgs::PointCloud2 allWindowsClouds;
        pcl::toROSMsg(*CloudLocalMap, allWindowsClouds);
        allWindowsClouds.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        allWindowsClouds.header.frame_id = odom_frame_id;
        pubLaserCloudSurround.publish(allWindowsClouds);
        // 发布当前帧点云
        sensor_msgs::PointCloud2 laserCloud;
        pcl::toROSMsg(*lidarCloud, laserCloud);
        laserCloud.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloud.header.frame_id = optimized_lidar_frame_id;
        pubLaserCloud.publish(laserCloud);
        // 发布odom信息   低频   给后端优化的
        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = odom_frame_id;
        odomAftMapped.child_frame_id = optimized_lidar_frame_id;
        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = q_opt_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_opt_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_opt_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_opt_curr.w();
        odomAftMapped.pose.pose.position.x = t_opt_curr.x();
        odomAftMapped.pose.pose.position.y = t_opt_curr.y();
        odomAftMapped.pose.pose.position.z = t_opt_curr.z();
        pubOdomAftMapped.publish(odomAftMapped);
        // 发布路径
        geometry_msgs::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = odom_frame_id;
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        pubLaserAfterMappedPath.publish(laserAfterMappedPath);
       
    }
    // 2ms的延时  
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void initialize_params(ros::NodeHandle nh) 
{
    
  // 设置原点坐标系名字
  // TODO: 从yaml 文件中读取该参数  
  odom_frame_id = nh.param<std::string>("odom_frame_id", "/odom");
  optimized_lidar_frame_id = nh.param<std::string>("optimized_lidar_frame_id", "/lidar_opt_odom");

  // 关键帧选取 
  keyframe_delta_trans = nh.param<double>("keyframe_delta_trans", 0.5);
  keyframe_delta_angle = nh.param<double>("keyframe_delta_angle", 0.8);
  // 设置匹配方法   默认为ndt    
  registration = Set_NDTOMP_param(nh);
  // 滑动窗口每一帧都初始化
	for (int i = 0; i < windows_size; i++)
	{
		FramesWin[i].reset(new pcl::PointCloud<PointT>());
	}
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "MapOptimator_node");   
    ROS_INFO("Started MapOptimator_node node");  
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    initialize_params(private_nh);
    // odometry 数据订阅   
    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, laserOdometryHandler);
    // Todo: 接收去畸变后的点云
    points_sub = nh.subscribe("/processed_points", 100, cloudHandler);
    // 高频的优化后odom位姿  这个主要用于状态估计
	  pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/odom_opt_high", 100);
    // 优化后输出的低频位姿    主要用于建图 
	  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/odom_opt_low", 100);
    // 路径
    pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);
    // 局部地图
	  pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/key_cloud", 100);
    std::thread process{MapOptimize}; 
    ros::spin(); 
    return 0;
}







