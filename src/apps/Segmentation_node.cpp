
#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "omp.h"
#include "Segmentation.hpp" 

ros::Subscriber points_sub; 
ros::Publisher seg_pub; 
std::string lidar_frame_id;
Segmentation seg;

// 点云的回调函数  
void points_cb(const sensor_msgs::PointCloud2 cloud_msg)
{
   // 转成PCL格式
   pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
   pcl::fromROSMsg(cloud_msg, *cloud);
   // 地面检测
   Eigen::Vector4f n = seg.floor_remove(cloud);    
   // 发布
   sensor_msgs::PointCloud2 output;
   pcl::toROSMsg(*cloud, output);
   // output.header.stamp=ros::Time::now(); 
   output.header.stamp=cloud_msg.header.stamp; 
   output.header.frame_id = lidar_frame_id;
   // std::cout<<"frame_id: "<<output.header.frame_id<<std::endl;
   seg_pub.publish(output);
}

void init(ros::NodeHandle& nh)
{
   lidar_frame_id = nh.param<std::string>("lidar_frame_id", "lidar_opt_odom");  
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "Segmentation_node");   
    ROS_INFO("Started Segmentation_node");   
    ros::NodeHandle nh("~");                         // 初始化私有句柄   
    init(nh);
    // 数据订阅   
    points_sub = nh.subscribe<sensor_msgs::PointCloud2> ("/processed_points", 100, points_cb);  
    seg_pub = nh.advertise<sensor_msgs::PointCloud2> ("/seg_points", 10);      
    ros::spin(); 
    return 0;
}