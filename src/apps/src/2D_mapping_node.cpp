
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include "ros_utils.hpp"
#include "grid_map/2d_grid_mapping.h"
#include "tic_toc.h"

/* Global */
GridMap* g_map;
GridMapping* grid_mapping;
tf::StampedTransform g_transform;

ros::Subscriber g_odom_suber;
ros::Subscriber laser_suber;
ros::Subscriber odom2map_suber;  

ros::Publisher g_map_puber;
Pose2d g_robot_pose;

// laser接收队列
std::queue<sensor_msgs::LaserScanConstPtr> laserBuf;
// 里程计接收队列 
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
std::mutex mBuf;
// odom坐标系到map系的转换  
Eigen::Matrix4f trans_odom2map;

// 里程计的回调  
void odometryCallback ( const nav_msgs::OdometryConstPtr& odom );
// laser 的回调  
void laserCallback ( const sensor_msgs::LaserScanConstPtr& scan );


void odometryCallback ( const nav_msgs::OdometryConstPtr& odom )
{   
    mBuf.lock();
    odometryBuf.push(odom);
    mBuf.unlock();   
}

// 激光回调
void laserCallback ( const sensor_msgs::LaserScanConstPtr& scan )
{   
    mBuf.lock();
    laserBuf.push(scan);
    mBuf.unlock();   
}

/**
 * @brief 发送odom转换到map的变换关系 
 * @details 其实就是将栅格地图和点云图对齐  
 **/
void odom2mapCallback(geometry_msgs::TransformStamped odom2map_T)
{
   trans_odom2map(0,3) = odom2map_T.transform.translation.x;
   trans_odom2map(1,3) = odom2map_T.transform.translation.y;
   trans_odom2map(2,3) = odom2map_T.transform.translation.z;

   Eigen::Quaternionf quat;  
   quat.w() = odom2map_T.transform.rotation.w;
   quat.x() = odom2map_T.transform.rotation.x;
   quat.y() = odom2map_T.transform.rotation.y;
   quat.z() = odom2map_T.transform.rotation.z;

   trans_odom2map.block<3,3>(0,0) = quat.toRotationMatrix();      
}

// 2D栅格建图线程 
void grid_map_create()
{
   while(1)
  { 
    // 获取点云数据以及里程计数据  同时进行对齐 
    while(!odometryBuf.empty() && !laserBuf.empty())  
    {  
      mBuf.lock();      // 上锁  此时 回调函数阻塞   
      // 如果 点云数据队首早于odometry数据队首   则点云数据队首丢弃 
      while(!odometryBuf.empty()&&laserBuf.front()->header.stamp.toSec()>odometryBuf.front()->header.stamp.toSec())
        odometryBuf.pop();  
      if(odometryBuf.empty()) 
      {
        mBuf.unlock();    
        break;
      }
      while(!laserBuf.empty()&&laserBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec())
        laserBuf.pop();
      if(laserBuf.empty())  
      {  
        mBuf.unlock();
        break;
      }
      // 最后再检查一下对齐没有 
      if(odometryBuf.front()->header.stamp.toSec()!=laserBuf.front()->header.stamp.toSec())
      {
        ROS_INFO_STREAM("2d Map error!! ,   data unsync");
        mBuf.unlock();
        break;
      } 
      mBuf.unlock();   

      TicToc t_pub;  
      /* 获取机器人2D姿态    这个姿态是相对与odom系的 */
      nav_msgs::OdometryConstPtr odom = odometryBuf.front();
      odometryBuf.pop();    
      
      // 将其转换到map系  
      Eigen::Isometry3d odom_T = odom2isometry(odom);
      Eigen::Matrix4f pose_in_map = trans_odom2map * odom_T.cast<float>().matrix(); 
      //ROS_INFO_STREAM("2d_mapping_node - trans_odom2map: "<<std::endl<<trans_odom2map);    

      double x = pose_in_map(0,3);  
      double y = pose_in_map(1,3); 
      Eigen::Quaterniond eigen_quat(pose_in_map.block<3,3>(0,0).cast<double>());
      geometry_msgs::Quaternion ros_quat;
      ros_quat.w = eigen_quat.w();
      ros_quat.x = eigen_quat.x();
      ros_quat.y = eigen_quat.y();
      ros_quat.z = eigen_quat.z();
      double theta = tf::getYaw ( ros_quat );

      //ROS_INFO_STREAM("pose_in_map x: "<<x<<" y: "<<y<<" theta: "<<theta);   
      /* 获取机器人姿态 */
    
      g_robot_pose = Pose2d ( x, y, theta );     // 创建2d 机器人位姿  
      // 获取激光 
      sensor_msgs::LaserScanConstPtr laser;
      laser = laserBuf.front();
      laserBuf.pop();
      
      /* 输入激光与机器人pose(map系下)    更新地图 */          
      grid_mapping->updateMap ( laser, g_robot_pose ); 
      printf("update 2D Map time %f ms \n", t_pub.toc());
      /* 用opencv图像显示地图 */
      //cv::Mat map = g_map->toCvMat();
      //cv::imshow ( "map", map );
      //cv::waitKey ( 1 );
    }
    // 2ms的延时  
    std::chrono::milliseconds dura(1);
    std::this_thread::sleep_for(dura);

  }
}

// 发布地图的线程  
void send_map()
{
  while(1)
  { 
    /* 发布地图 */
    nav_msgs::OccupancyGrid occ_map;
    g_map->toRosOccGridMap ( "map", occ_map );
    g_map_puber.publish ( occ_map );
    // 100ms的延时  
    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }
}

int main ( int argc, char **argv )
{
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "GridMapping_node" );
    ros::NodeHandle nh;
    
    // /***** 加载参数 *****/
    int map_sizex, map_sizey, map_initx, map_inity;
    double map_cell_size;
    /* TODO 错误处理 */
    // 地图信息
    nh.getParam ( "/map/sizex", map_sizex );      // 1000
    nh.getParam ( "/map/sizey", map_sizey );      // 1000
    // 地图中点   
    nh.getParam ( "/map/initx", map_initx );      // 500
    nh.getParam ( "/map/inity", map_inity );      // 500
    nh.getParam ( "/map/cell_size", map_cell_size );   // 0.05
    // 激光 - body 转换  
    Pose2d T_r_l;
    double x, y, theta;
    double P_occ, P_free, P_prior;
    /* 读取激光-body的参数          TODO 错误处理 */
    nh.getParam ( "/robot_laser/x", x );
    nh.getParam ( "/robot_laser/y", y );
    nh.getParam ( "/robot_laser/theta", theta );
    x = 0;
    y = 0;
    theta = 0; 
    T_r_l = Pose2d ( x, y, theta );

    // 概率设置  
    nh.getParam ( "/sensor_model/P_occ", P_occ );       // 0.6
    nh.getParam ( "/sensor_model/P_free", P_free );     // 0.4
    nh.getParam ( "/sensor_model/P_prior", P_prior );   // 0.5  
    
    /* 地图保存地址 */
    //std::string map_image_save_dir, map_config_save_dir;
    //nh.getParam ( "/map_image_save_dir", map_image_save_dir );
    //nh.getParam ( "/map_config_save_dir", map_config_save_dir );
    
    map_sizex = 6000;
    map_sizey = 6000;
    map_initx = 3000;
    map_inity = 3000;
    map_cell_size = 0.3;  
    
    P_occ = 0.6;
    P_free = 0.4;
    P_prior = 0.5;

    ROS_INFO_STREAM( "map_sizex: "<<map_sizex<<" map_sizey: "<<map_sizey<<" map_initx: "<<map_initx<<" map_inity: "
                     << map_inity<< " map_cell_size: "<<map_cell_size );

    trans_odom2map = Eigen::Matrix4f::Identity(); 

    /***** 初始化地图和构图器 *****/
    g_map = new GridMap ( map_sizex, map_sizey,  map_initx, map_inity, map_cell_size );      // 地图初始化
    grid_mapping = new GridMapping ( g_map, T_r_l, P_occ, P_free, P_prior );              // 构图器初始化 

    /***** 初始Topic *****/
    g_odom_suber = nh.subscribe ( "/odom_opt_high", 1000, odometryCallback );          // odom    
    laser_suber = nh.subscribe ( "/laser_scan", 1000, laserCallback );                 // laser
    g_map_puber = nh.advertise<nav_msgs::OccupancyGrid> ( "mapping/grid_map", 1 );     // 发布占据栅格地图  
    odom2map_suber = nh.subscribe ( "/odom2pub", 10, odom2mapCallback );               // odom - map  
    
    std::thread process{grid_map_create}; 
    std::thread sendMap{send_map}; 
    ros::spin();

    /* TODO 保存地图 */
    // g_map->saveMap(map_image_save_dir, map_config_save_dir);
    
    // std::cout << "\nMap saved\n";
}




