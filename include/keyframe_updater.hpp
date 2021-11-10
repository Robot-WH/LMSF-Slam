#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>


/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param 
   */
  KeyframeUpdater(ros::NodeHandle& pnh)
    : is_first(true),
      prev_keypose(Eigen::Isometry3d::Identity())
  {
    // 关键帧的筛选条件
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 1.5);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 1.5);
    accum_distance = 0.0;
    deltaOdom = Eigen::Isometry3d::Identity();   // 初始化为单位阵  
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose pose of the frame
   * @return  if true, the frame should be registered
   */
  bool Update(const Eigen::Isometry3d& pose) 
  {
    // first frame is always registered to the graph
    // 如果是第一次添加关键帧
    if(is_first) 
    {
      is_first = false;
      prev_keypose = pose;
      return true;
    }
    // calculate the delta transformation from the previous keyframe  计算与上一帧的位移
    deltaOdom = prev_keypose.inverse() * pose;
    double dx = deltaOdom.translation().norm();
    //   delta.linear()是获取变换矩阵中的旋转矩阵      生成四元数   获取角度
    //  旋转矩阵对应 u*theta  对应四元数  e^u*theta/2  = [cos(theta/2), usin(theta/2)]
    Eigen::Quaterniond q_a(deltaOdom.linear());
    q_a.normalize();   
    double da = std::acos(q_a.w())*2;     // 获得弧度    90度 约等于 1.5  
    // too close to the previous frame
    if(dx < keyframe_delta_trans && da < keyframe_delta_angle) 
    {
      return false;
    }
    // 总里程距离  
    accum_distance += dx;
    // 记录
    prev_keypose = pose;
    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const 
  {
    return accum_distance;
  }

  /**
   * @brief 获取与上个关键帧之间的位姿变化
   */
  Eigen::Isometry3d const& GetdeltaOdom() const 
  {
      return deltaOdom;  
  }

private:
  // parameters
  double keyframe_delta_trans;      
  double keyframe_delta_angle;      

  bool is_first;
  // 总里程数   
  double accum_distance;
  Eigen::Isometry3d prev_keypose;    // 上一帧关键帧的位姿
  // 两个关键帧之间的odom
  Eigen::Isometry3d deltaOdom;  
};

#endif // KEYFRAME_UPDATOR_HPP
