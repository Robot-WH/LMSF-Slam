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
   * @brief KeyFrame (pose node)
   */
  class KeyFrame 
  {
    public:
      using Ptr = std::shared_ptr<KeyFrame>;
      using ConstPtr = std::shared_ptr<const KeyFrame>; 
      KeyFrame() {}
      // 关键帧不储存点云数据  
      KeyFrame(double const& stamp, Eigen::Isometry3d const& odom, int id)
      : time_stamp_(stamp), odom_(odom), id_(id)
      {}

      virtual ~KeyFrame() {}

      void save(const std::string& directory)
      {
        if (!boost::filesystem::is_directory(directory)) {
          boost::filesystem::create_directory(directory);
        }

        std::ofstream ofs(directory + "/KeyFrameData/data_" + std::to_string(id_));
        ofs << "stamp " << time_stamp_<< "\n";

        ofs << "pose\n";
        ofs << correct_pose_.matrix() << "\n";

        // if(floor_coeffs) {
        //   ofs << "floor_coeffs " << floor_coeffs->transpose() << "\n";
        // }

        // if(utm_coord) {
        //   ofs << "utm_coord " << utm_coord->transpose() << "\n";
        // }

        // if(acceleration) {
        //   ofs << "acceleration " << acceleration->transpose() << "\n";
        // }

        // if(orientation) {
        //   ofs << "orientation " << orientation->w() << " " << orientation->x() << " " << orientation->y() << " " << orientation->z() << "\n";
        // }

        ofs << "id " << id_ << "\n";
        
      }
      // bool load(const std::string& directory, g2o::HyperGraph* graph);
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
      uint32_t id_ = 0;   // 对应点云文件  在文件夹中的标识    以及   节点
      Eigen::Isometry3d odom_ = Eigen::Isometry3d::Identity();     // odometry (estimated by scan_matching_odometry)   
      Eigen::Isometry3d correct_pose_ = Eigen::Isometry3d::Identity();       // Map系下被校正后的坐标  
      Eigen::Isometry3d between_constraint_ = Eigen::Isometry3d::Identity();  // 与上一个关键帧的相对约束   
      boost::optional<Eigen::Vector4d> floor_coeffs_;   // detected floor's coefficients    地面信息 
      Eigen::Vector3d utm_coord_ = {0, 0, 0};                       // UTM coord obtained by GPS    UTM坐标   
      Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();               // imu测得的激光姿态
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


