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
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Dense>
#include "Sensor/lidar_data_type.h"

namespace Slam3D
{
  /**
   * @brief KeyFrame  主要保存关键帧的各种观测信息
   */
  class KeyFrame 
  {
    public:
      using Ptr = std::shared_ptr<KeyFrame>;
      using ConstPtr = std::shared_ptr<const KeyFrame>; 
      KeyFrame() {}
      // 关键帧不储存点云数据  
      KeyFrame(double const& stamp, Eigen::Isometry3d const& odom, int64_t id = -1)
      : time_stamp_(stamp), odom_(odom), id_(id)
      {}

      virtual ~KeyFrame() {}

      void Save(const std::string& directory)
      {
        if (!boost::filesystem::is_directory(directory)) {
          boost::filesystem::create_directory(directory);
        }

        std::ofstream ofs(directory + "/KeyFrameData/data_" + std::to_string(id_));
        ofs << "stamp " << time_stamp_<< "\n";

        ofs << "id " << id_ << "\n";

        ofs << "odom\n";
        ofs << odom_.matrix() << "\n";
  
        if (utm_coord_)
        {
          ofs << "utm_coord\n";
          ofs << utm_coord_->transpose() << "\n";
        }
        if (floor_coeffs_)
        {
          ofs << "floor_coeffs\n";
          ofs << floor_coeffs_->transpose() << "\n";
        }
        if (acceleration_)
        {
          ofs << "acceleration\n";
          ofs << acceleration_->transpose() << "\n";
        }
        if (orientation_)
        {
          ofs << "orientation\n";
          ofs << orientation_->w() << " " << orientation_->x() << " " << orientation_->y() << " " << orientation_->z() << "\n";
        }
      }

      bool Load(const std::string& directory)
      {

      }
      /**
       * @brief: id 是和保存在硬盘中的点云数据标签一致的
       */      
      int get_id() const
      {
        return id_;
      }

    public:
      // 关键帧的数据结构
      double time_stamp_ = 0.0;                                // timestamp
      int64_t id_ = -1;   // 对应点云文件  在文件夹中的标识    以及   节点
      int64_t adjacent_id_ = -1; // 上一个连接的关键帧 id 
      Eigen::Isometry3d odom_ = Eigen::Isometry3d::Identity();     // odometry (estimated by scan_matching_odometry)   
      Eigen::Isometry3d between_constraint_ = Eigen::Isometry3d::Identity();  // 与上一个关键帧的相对约束   
      boost::optional<Eigen::Vector4d> floor_coeffs_;   // detected floor's coefficients    地面信息 
      boost::optional<Eigen::Vector3d> utm_coord_;                       // UTM coord obtained by GPS    UTM坐标   
      boost::optional<Eigen::Vector3d> acceleration_;                       // UTM coord obtained by GPS    UTM坐标   
      boost::optional<Eigen::Quaterniond> orientation_;               // imu测得的激光姿态
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
  //     KeyFrameSimplify(const Eigen::Isometry3d& pose, CloudContainer& cloud);

  //     ~KeyFrameSimplify();

  //   public:
  //     Eigen::Isometry3d pose;   // pose estimated by graph optimization
  //     CloudContainer cloud;  // point cloud
  // };
}


