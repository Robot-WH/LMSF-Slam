
#ifndef GRID_MAPPING_H
#define GRID_MAPPING_H

#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "2d_grid_map.hpp"
#include "Pose2d.h"

// 占据栅格地图建立方法 
class GridMapping{
public:
    GridMapping(GridMap* map,  Pose2d& T_r_l,  double& P_occ, double& P_free, double& P_prior);   
    void updateMap(const sensor_msgs::LaserScanConstPtr& scan, Pose2d& robot_pose);      //根据当前机器人的位姿和激光雷达数据跟新一次地图

    /** TODO 定义为 protected **/   
    void updateGrid(const Eigen::Vector2d& grid_pos, const double& log_pmzx);                    // 更新栅格  
    double laserInvModel(const double& r, const double& R, const double& cell_size,  const int& range_max);     // 反演观测模型     
    
private:
    GridMap* map_;                          // 地图对象
    Pose2d T_r_l_;                          // laser - robot的变换 
    double P_occ_, P_free_, P_prior_;       // 占据, 先验, 空闲概率  
    std::unordered_map<float, float> InvModel_Ps;                  // 反演观测模型概率值  
}; //class GridMapper

#endif

