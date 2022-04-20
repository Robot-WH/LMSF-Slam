/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <map>
#include <cassert>
#include <cstdio>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "color.hpp"

extern bool init_flag_;   // 初始化标志位
extern int removal_nan_;  // 是否去除NaN点 
extern std::string OPERATION_MODE_;  // pure_lidar: 纯雷达 , lidar_imu: 融合IMU, lidar_wheel: 融合轮速, lidar_wheel_imu: 融合imu与轮速 
extern std::string POINTCLOUD_MATCHING_MODE_;  // 匹配模式 
extern std::string LIDAR_TRACKING_MODE_;   // 激光跟踪类型 
extern std::string DOWNSAMPLING_TYPE_ ;  
extern std::string OUTLIERREMOVAL_TYPE_ ;
extern int lidar_num_;    
extern float SCAN_PERIOD_;  
extern int SCAN_NUM_;
// 激光预处理参数
extern float voxel_grid_resolution_;   
extern float radiusOutlierRemoval_radius_;  
extern int radiusOutlierRemoval_minNum_;  
extern float distanceFilter_max_;
extern float distanceFilter_min_;

extern int scan_map_windows_size_;  
// extern int MLOAM_RESULT_SAVE;
// extern std::string OUTPUT_FOLDER;
// extern std::string MLOAM_ODOM_PATH;
// extern std::string MLOAM_MAP_PATH;
// extern std::string MLOAM_GPS_PATH;
// extern std::string MLOAM_GT_PATH;
// extern std::string EX_CALIB_RESULT_PATH;
// extern std::string EX_CALIB_EIG_PATH;

// extern int MULTIPLE_THREAD;

// extern double SOLVER_TIME;
// extern int NUM_ITERATIONS;
// extern int ESTIMATE_EXTRINSIC;
// extern int ESTIMATE_TD;

// extern int SEGMENT_CLOUD;
// extern int HORIZON_SCAN;
// extern int MIN_CLUSTER_SIZE;
// extern int MIN_LINE_SIZE;
// extern int SEGMENT_VALID_POINT_NUM;
// extern int SEGMENT_VALID_LINE_NUM;
// extern float SEGMENT_THETA;

// // LiDAR
// extern size_t IDX_REF;
// extern size_t NUM_OF_LASER;
// extern size_t N_SCANS;

// extern int WINDOW_SIZE;
// extern int OPT_WINDOW_SIZE;

// extern int DISTORTION;
// extern float SCAN_PERIOD;
// extern float DISTANCE_SQ_THRESHOLD;
// extern float NEARBY_SCAN;

// extern std::string CLOUD0_TOPIC, CLOUD1_TOPIC;
// extern std::vector<std::string> CLOUD_TOPIC;

// extern float LASER_SYNC_THRESHOLD;
// extern double ROI_RANGE;

// extern std::vector<Eigen::Quaterniond> QBL;
// extern std::vector<Eigen::Vector3d> TBL;
// extern std::vector<double> TDBL;

// // odometry
// extern int PLANAR_MOVEMENT;

// extern float MIN_MATCH_SQ_DIS;
// extern float MIN_PLANE_DIS;

// extern int MARGINALIZATION_FACTOR;
// extern int POINT_PLANE_FACTOR;
// extern int POINT_EDGE_FACTOR;
// extern int PRIOR_FACTOR;
// extern double PRIOR_FACTOR_POS;
// extern double PRIOR_FACTOR_ROT;
// extern int CHECK_JACOBIAN;

// extern int PCL_VIEWER;
// extern int PCL_VIEWER_NORMAL_RATIO;

// extern int N_CUMU_FEATURE;
// extern double LAMBDA_INITIAL;
// extern double LAMBDA_THRE_CALIB;
// extern int N_CALIB;
// extern float ODOM_GF_RATIO;

// extern int SKIP_NUM_ODOM_PUB;
// extern int LM_OPT_ENABLE;

// // mapping
// extern float MAP_CORNER_RES;
// extern float MAP_SURF_RES;
// extern float MAP_OUTLIER_RES;
// extern float MAP_SUR_KF_RES;
// extern float MAP_EIG_THRE;
// extern float MAP_DEG_THRE;

// extern float DISTANCE_KEYFRAMES;
// extern float ORIENTATION_KEYFRAMES;
// extern float SURROUNDING_KF_RADIUS;

// extern float UCT_EXT_RATIO;
// extern std::vector<Eigen::Matrix<double, 6, 6> > COV_EXT;
// extern Eigen::Matrix<double, 3, 3> COV_MEASUREMENT;
// extern double TRACE_THRESHOLD_MAPPING;

void readParameters(std::string config_file);


