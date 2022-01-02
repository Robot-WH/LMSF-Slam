
#include <ros/ros.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

//#include <ceres/ceres.h>
 
/**
 * 主要完成
 * 1 如果采用特征点的里程计    
 **/
using namespace std;

typedef pcl::PointXYZI PointT;
const float scanPeriod = 0.1;   

class LidarPreprocessing
{
public:  
  LidarPreprocessing()  
  {  
    private_nh = ros::NodeHandle("~");       // 初始化私有句柄  
    points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/processed_points", 10);                                       // 初始化发布器      
    // 注意： scan_cb()要定义  不然编译不过
    //points_sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, &LidarPreprocessing::scan_cb, this);  
    string pointcloud_topic;
    nh.param<std::string>("liv_slam/pointCloudTopic", pointcloud_topic, "points_raw");  
    points_sub = nh.subscribe<sensor_msgs::PointCloud2> (pointcloud_topic, 100, &LidarPreprocessing::scan_cb, this);  
    std::cout<<" pointcloud_topic: "<< pointcloud_topic <<std::endl;
    // points_sub = nh.subscribe<sensor_msgs::PointCloud2> ("/rslidar_points", 100, &LidarPreprocessing::scan_cb, this);   
    basedFrame = private_nh.param<std::string>("based_frame_id", "/lidar_odom");
    std::cout<<" laser based frame: "<< basedFrame <<std::endl;
    process_init();                 // 初始化      
  }  
  virtual ~LidarPreprocessing() {}
  //回调函数  接收发送过来的点云msg  用于定位
  void scan_cb(const sensor_msgs::PointCloud2 cloud_msg);
  void process_init();             // 初始化  

  // 处理方法
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
  pcl::PointCloud<PointT>::ConstPtr process(const pcl::PointCloud<PointT>::ConstPtr& cloud, const float& startOri, const float& endOri);

private:  
  ros::NodeHandle nh;   
  ros::NodeHandle private_nh;      // 私有句柄

  ros::Publisher points_pub;       // 创建一个点云处理的
  ros::Subscriber points_sub;      // 创建一个订阅者  订阅话题laser_scan

  pcl::Filter<PointT>::Ptr downsample_filter;         // 降采样滤波器对象
  pcl::Filter<PointT>::Ptr outlier_removal_filter;    // 离群点滤波对象
  // 距离滤波阈值
  float distance_near_thresh;   
  float distance_far_thresh;   

  string basedFrame;  

  bool use_downsample_filter = false; 

};

// 初始化  
void LidarPreprocessing::process_init()
{ 
  /***************** 降采样滤波器初始化 *******************/
  float downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
  use_downsample_filter = private_nh.param<bool>("use_downsample_filter", true);
  if(use_downsample_filter){
    cout<<"use_downsample_filter:true"<<endl;
    cout<<"downsample: VOXELGRID,resolution: "<<downsample_resolution<<endl;
    // 创建指向体素滤波器VoxelGrid的智能指针    
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;
  }
  else{
    cout<<"disable downsample filter!"<<endl;
    downsample_filter = NULL;   
  }
  /***************** 离群点滤波器初始化 ***************************/
  double radius = private_nh.param<double>("radius_r", 0.5);                  
  int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);
  std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;
  // 几何法去除离群点
  pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
  rad->setRadiusSearch(radius);                      
  rad->setMinNeighborsInRadius(min_neighbors);
  outlier_removal_filter = rad;
  // 距离滤波初始化
  distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100);
  distance_near_thresh = private_nh.param<double>("distance_near_thresh",  2);
  cout<<"distance filter threshold: "<< distance_near_thresh << " ,"<<distance_far_thresh<<endl;
}

// 点云的回调函数  
void LidarPreprocessing::scan_cb(const sensor_msgs::PointCloud2 cloud_msg)
{
  // 转成PCL格式
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(cloud_msg, *cloud);
  if(cloud->empty())
      return;

  /***********************************************数据模型化********************************************/
  int cloudSize = cloud->points.size();
  //lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转,atan2逆时针为正角
  float startOri = -atan2(cloud->points[0].y, cloud->points[0].x);
  //lidar scan结束点的旋转角，加2*pi使点云旋转周期为2*pi   ？？？？？？？？？？？？？？？？？？？？？？
  float endOri = -atan2(cloud->points[cloudSize - 1].y,
                        cloud->points[cloudSize - 1].x) +
                  2 * M_PI;
  // 处理  保证  M_PI < endOri - startOri < 3 * M_PI
  if (endOri - startOri > 3 * M_PI)   // startOri<0
  {
      endOri -= 2 * M_PI;  
  }
  else if (endOri - startOri < M_PI)
  {
      endOri += 2 * M_PI;
  }    
    
  pcl::PointCloud<PointT>::ConstPtr filtered = downsample(cloud);    // 降采样
  filtered = process(filtered, startOri, endOri);                    // 点云处理 
  filtered = outlier_removal(filtered);                              // 外点去除

  // 发布
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*filtered, output);
  // output.header.stamp=ros::Time::now(); 
  output.header.stamp=cloud_msg.header.stamp; 
  output.header.frame_id = basedFrame;
//   cout<<"frame_id: "<<output.header.frame_id<<endl;
  points_pub.publish(output);
    
}

// 降采样滤波
pcl::PointCloud<PointT>::ConstPtr LidarPreprocessing::downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
  if(use_downsample_filter == false)  return cloud;
  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  downsample_filter->setInputCloud(cloud);
  downsample_filter->filter(*filtered);
  filtered->header = cloud->header;
  return filtered;
}

// 离群点去除
pcl::PointCloud<PointT>::ConstPtr LidarPreprocessing::outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  outlier_removal_filter->setInputCloud(cloud);
  outlier_removal_filter->filter(*filtered);
  filtered->header = cloud->header;

  return filtered;
}

// 1. 获取深度 获取深度的目的是匹配的时候要对深度进行筛选  2. 深度滤波   3. 求点的角度  用于去除畸变  
pcl::PointCloud<PointT>::ConstPtr LidarPreprocessing::process(const pcl::PointCloud<PointT>::ConstPtr& cloud, const float& startOri, const float& endOri) {
  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  filtered->reserve(cloud->size());
  /*
  std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
    [&](const PointT& p) {           // 外部变量按引用传递   
      double d = p.getVector3fMap().norm();
      if(d > distance_near_thresh && d < distance_far_thresh){   
         p.intensity = d;
         return true;
      }
      else return false;
    }
  );   */
  PointT point;  
  // lidar扫描线是否旋转过半
  bool halfPassed = false;
  for(auto& p:cloud->points)
  {
     int d = (int) p.getVector3fMap().norm();
     if(d > distance_near_thresh && d < distance_far_thresh){   
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        /************************求出该点的旋转角*************************/
        float ori = -atan2(point.y, point.x);
        //std::cout<<"ori: "<<ori<<std::endl;
        if (!halfPassed)    // false
        {
            // 确保-pi/2 < ori - startOri < 3*pi/2
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            { 
                ori -= 2 * M_PI;     // 结果为   ori - startOri > - M_PI / 2
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else    // true
        {
            ori += 2 * M_PI;
            //确保-3*pi/2 < ori - endOri < pi/2
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        // -0.5 < relTime < 1.5（点旋转的角度与整个周期旋转角度的比率, 即点云中点的相对时间）
        float relTime = (ori - startOri) / (endOri - startOri);
        //点强度=线号+点相对时间（即一个整数+一个小数，整数部分是距离 ，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
        point.intensity = d + scanPeriod * relTime;         // scanPeriod每一帧的时间   
        filtered->push_back(point);
     }
  }

  filtered->width = filtered->size();    // 点云的数量
  filtered->height = 1;
  filtered->is_dense = false;

  filtered->header = cloud->header;

  return filtered;
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "LidarPreprocessing_node");   
    ROS_INFO("Started LidarPreprocessing node");   
    LidarPreprocessing processing;
    ros::spin(); 
    return 0;
}


