#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

namespace g2o {
  class VertexSE3;
  class HyperGraph;
  class SparseOptimizer;
}


/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<KeyFrame>;
  
  KeyFrame(const ros::Time& _stamp, const Eigen::Isometry3d& _odom, int _id, const pcl::PointCloud<PointT>::ConstPtr& _cloud);
  KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  void save(const std::string& directory);
  bool load(const std::string& directory, g2o::HyperGraph* graph);

  int get_id() const;
  Eigen::Isometry3d estimate() const;

public:
  // 关键帧的数据结构
  ros::Time stamp;                                // timestamp
  int id;
  Eigen::Isometry3d odom;                         // odometry (estimated by scan_matching_odometry)   
  Eigen::Isometry3d Pose;                         // Map系下被校正后的坐标  
  Eigen::Isometry3d deltaOdom;                    // 激光里程计 计算出来的位移  
  pcl::PointCloud<PointT>::ConstPtr cloud;         // point cloud   点云
  boost::optional<Eigen::Vector4d> floor_coeffs;   // detected floor's coefficients    地面参数
  Eigen::Vector3d utm_coord;                       // UTM coord obtained by GPS    UTM坐标   
  Eigen::Quaterniond orientation;                  // imu测得的激光姿态
  bool gnss_matched = false;  
  bool GNSS_Valid = false;
  bool IMU_Valid = false;
  bool planeConstraint_Valid = false;  
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key);
  KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  ~KeyFrameSnapshot();

public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};



#endif // KEYFRAME_HPP
