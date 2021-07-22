
#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tic_toc.h"

ros::Publisher laserscan_pub;
ros::Subscriber pointcloud_sub;

// ROS Parameters
std::string target_frame_;
double tolerance_;
// 高度滤波范围
double min_height_;   
double max_height_; 
// 角度范围  
double angle_min_;   // -pi
double angle_max_;   // pi
double angle_increment_;  // 角分辨率
double scan_time_;        // 扫描时间 
double range_min_;        // 激光最小范围
double range_max_;        // 激光最大范围
bool use_inf_;            // 
double inf_epsilon_;

// 16线的回调函数  
void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  TicToc t_pub;  
  // build laserscan output
  sensor_msgs::LaserScan output;        // 单线激光   
  // 所在坐标系名
  output.header = cloud_msg->header;
  // 如果给定了坐标系   
  if (!target_frame_.empty()) 
  {
    output.header.frame_id = target_frame_;
  }

  output.angle_min = angle_min_;                // -pi
  output.angle_max = angle_max_;                // pi
  output.angle_increment = angle_increment_;    // 角度增量  0.17度  
  output.time_increment = 0.0; 
  output.scan_time = scan_time_;    // 扫描时间 
  output.range_min = range_min_;    // 0.2
  output.range_max = range_max_;    // 100 

  // determine amount of rays to create    等效单线激光束的个数      ceil 向上取整  
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  // 初始化每个点的值  初始化为 infinity()
  if (use_inf_)                                  // true  
  { // ranges  保存每个激光束的测量值  
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }
  
  sensor_msgs::PointCloud2ConstPtr cloud_out;
  cloud_out = cloud_msg;
  // Iterate through pointcloud  用迭代器遍历每一个点  
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // 无效点  
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }
    // 不在高度范围内
    if (*iter_z > max_height_ || *iter_z < min_height_)
    {
      continue;
    }
    // 求激光测量值  
    double range = hypot(*iter_x, *iter_y);     // 两个数平方和的平方根
    if (range < range_min_)
    {
      continue;
    }
    if (range > range_max_)
    {
      continue;
    }
    // 角度  范围: [-pi, pi]     
    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    // 在高度范围内  取打在该角度上的最近激光束  
    int index = (angle - output.angle_min) / output.angle_increment;       // 求当前激光束的index    
    if (range < output.ranges[index])       // 只有比原有值小才取  
    {
      output.ranges[index] = range;
    }
  }
  
  laserscan_pub.publish(output);
  printf("pointcloud to laser time %f ms \n", t_pub.toc());
}


void laserInit(ros::NodeHandle& n)
{
  n.param<std::string>("target_frame", target_frame_, "/lidar_opt_odom");
  n.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  n.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  n.param<double>("angle_min", angle_min_, -M_PI);
  n.param<double>("angle_max", angle_max_, M_PI);
  n.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  n.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  n.param<double>("range_min", range_min_, 0.0);
  n.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  n.param<double>("inf_epsilon", inf_epsilon_, 1.0);

  n.param<bool>("use_inf", use_inf_, true);

  pointcloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 1000, cloudCb);       // /kitti/velo/pointcloud    /rslidar_points
  laserscan_pub = n.advertise<sensor_msgs::LaserScan>("/laser_scan", 10);
}



int main(int argc, char **argv)
{
    ros::init (argc, argv, "Pointcloud2laserscan_node");   
    ROS_INFO("Started Pointcloud2laserscan_node run");   
    ros::NodeHandle nh("~");                         // 初始化私有句柄   
    laserInit(nh);  
    ros::spin(); 
    return 0;
}