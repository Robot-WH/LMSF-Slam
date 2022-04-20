/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-18 18:21:53
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>
#include <eigen3/Eigen/Dense>
#include "Sensor/lidar_data_type.h"

namespace Slam3D
{
  /**
   * @brief KeyFrame (pose node)
   */
  template<typename PointT>
  class KeyFrame 
  {
    public:
      using Ptr = std::shared_ptr<KeyFrame<PointT>>;

      // 关键帧也可以不储存点云数据  
      KeyFrame(double const& stamp, Eigen::Isometry3d const& odom, int id)
      : time_stamp_(stamp), odom_(odom), id_(id)
      {}
      
      KeyFrame(double const& stamp, Eigen::Isometry3d const& odom, 
                              int id, FeatureInfo<PointT> const& feature_data)
      : time_stamp_(stamp), odom_(odom), id_(id), feature_data_(feature_data)
      {}

      virtual ~KeyFrame() {}

      void save(const std::string& directory)
      {}
      // bool load(const std::string& directory, g2o::HyperGraph* graph);

      int get_id() const
      {
        return id_;
      }

      // Eigen::Isometry3d estimate() const;

    public:
      // 关键帧的数据结构
      double time_stamp_;                                // timestamp
      int id_;   // 对应点云文件  在文件夹中的标识    以及   节点
      Eigen::Isometry3d odom_;     // odometry (estimated by scan_matching_odometry)   
      Eigen::Isometry3d correct_pose_;       // Map系下被校正后的坐标  
      Eigen::Isometry3d between_constraint_;  // 与上一个关键帧的相对约束   
      FeatureInfo<PointT> feature_data_;  // point cloud   特征点云
      boost::optional<Eigen::Vector4d> floor_coeffs_;   // detected floor's coefficients    地面信息 
      Eigen::Vector3d utm_coord_;                       // UTM coord obtained by GPS    UTM坐标   
      Eigen::Quaterniond orientation_;               // imu测得的激光姿态
      bool gnss_matched_ = false;  
      bool GNSS_valid_ = false;  // 是否使用GNSS 观测
      bool IMU_valid_ = false;     // 是否使用IMU姿态观测 
      bool planeConstraint_valid_ = false;  // 是否使用 平面约束观测 
  };

  // /**
  //  * @brief 
  //  */
  // struct KeyFrameSimplify
  // {
  //   public:
  //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //     using PointT = KeyFrame::PointT;
  //     using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  //     KeyFrameSimplify(const KeyFrame::Ptr& key);
  //     KeyFrameSimplify(const Eigen::Isometry3d& pose, FeatureInfo& cloud);

  //     ~KeyFrameSimplify();

  //   public:
  //     Eigen::Isometry3d pose;   // pose estimated by graph optimization
  //     FeatureInfo cloud;  // point cloud
  // };
}


