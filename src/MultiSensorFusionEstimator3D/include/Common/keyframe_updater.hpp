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

#include <Eigen/Dense>

namespace Slam3D
{
  /**
   * @brief: 用于检测 是否需要更新关键帧
   */  
  class KeyframeUpdater 
  {
    public:
      enum UpdateType    // 更新类型
      {
        FALSE = 0,
        TIME,     // 时间间隔过久
        MOTION   // 运动过大  
      };

      /**
       * @brief constructor
       * @param 
       */
      KeyframeUpdater()
        : is_first_(true),
          prev_keypose_(Eigen::Isometry3d::Identity()),
          last_keyframe_time_(0),
          KEYFRAME_TRANS_INCREM_(2.0), // m
          KEYFRAME_ANGLE_INCREM_(0.5236),    // 30度
          KEYFRAME_TIME_INTERVAL_(60)  // s
      {
        // 关键帧的筛选条件
        accum_distance_ = 0.0;
        odom_increment_ = Eigen::Isometry3d::Identity();   // 初始化为单位阵  
      }

      /**
       * @brief 通过当前帧的位姿 以及 时间戳 判断是否是关键帧 
       * @param pose pose of the frame
       * @return  if true, the frame should be registered
       */
      UpdateType NeedUpdate(const Eigen::Isometry3d& pose, double const& curr_time) 
      {
        // first frame is always registered to the graph
        // 如果是第一次添加关键帧
        if (is_first_) 
        {
          is_first_ = false;
          prev_keypose_ = pose;
          last_keyframe_time_ = curr_time; 
          return TIME;
        }
         // 先检查时间
        if (curr_time - last_keyframe_time_ > KEYFRAME_TIME_INTERVAL_) 
        {
            //std::cout<<common::GREEN<<"NEED TIME_UPDATA!"<<common::RESET<<std::endl;
            last_keyframe_time_ = curr_time;  
            prev_keypose_ = pose;
            return TIME;
        } 
        // 然后检查运动  
        // 计算与上一帧的位移
        odom_increment_ = prev_keypose_.inverse() * pose;
        double dx = odom_increment_.translation().norm();
        Eigen::Quaterniond q_a(odom_increment_.linear());
        q_a.normalize();   
        double da = std::acos(q_a.w())*2;     // 获得弧度    90度 约等于 1.5  
        // too close to the previous frame
        if(dx < KEYFRAME_TRANS_INCREM_ && da < KEYFRAME_ANGLE_INCREM_) 
        {
          return FALSE;   //  不需要更新  
        }
        // 总里程距离  
        accum_distance_ += dx;   
        // 记录
        prev_keypose_ = pose;
        last_keyframe_time_ = curr_time;  
        return MOTION;
      }

      /**
       * @brief the last keyframe's accumulated distance from the first keyframe
       * @return accumulated distance
       */
      double get_accum_distance() const 
      {
        return accum_distance_;
      }

      /**
       * @brief 获取与上个关键帧之间的位姿变化
       */
      Eigen::Isometry3d const& GetOdomIncrement() const 
      {
          return odom_increment_;  
      }

    private:
      // parameters
      double KEYFRAME_TRANS_INCREM_;      
      double KEYFRAME_ANGLE_INCREM_;    
      double KEYFRAME_TIME_INTERVAL_;   
      bool is_first_;
      // 总里程数   
      double accum_distance_;
      Eigen::Isometry3d prev_keypose_;    // 上一帧关键帧的位姿
      double last_keyframe_time_; 
      // 两个关键帧之间的odom
      Eigen::Isometry3d odom_increment_;  
  }; // class 
} // namespace 

