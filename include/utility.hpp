#pragma once
#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <eigen3/Eigen/Dense>
#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>


#include "tool/file_manager.hpp"

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER };

const std::string WORK_SPACE_PATH = "/home/gogo/lwh/lwh_ws/src/liv_slam-master"; 

constexpr double CoefDegreeToRadian = M_PI / 180.;
constexpr double CoefRadianToDegree = 180. / M_PI;

/**
 * @brief 保存轨迹数据  
 * 
 */
static bool SaveTrajectory(Eigen::Matrix4d const& gt_odom, Eigen::Matrix4d const& est_odom, string const& directory_path,
                           string const& file_1, string const& file_2) 
{
    static std::ofstream ground_truth, est_path;
    static bool is_file_created = false;
    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + directory_path))
            return false;
        if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + file_1))
            return false;
        if (!FileManager::CreateFile(est_path, WORK_SPACE_PATH + file_2))
            return false;
        is_file_created = true;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << gt_odom(i, j);
            est_path << est_odom(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                est_path << std::endl;
            } else {
                ground_truth << " ";
                est_path << " ";
            }
        }
    }

    return true;
}

/**
 * @brief 保存数据
 * 
 */
static bool SaveDataCsv(string const& Directory_path, string const& file_path, 
                        vector<double> const& datas, vector<string> const& labels) 
{
    static std::ofstream out;
    static bool is_file_created = false;

    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + Directory_path))
            return false;
        if (!FileManager::CreateFile(out, WORK_SPACE_PATH + file_path))
            return false;
        is_file_created = true;
        // 写标签
        for(auto const& label:labels)
        {
           out << label << ',';  
        }
        out << endl;
    }

    for(auto const& data:datas)
    {
       out << data << ',';
    }

    out << endl;
    return true;
}


static float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


static float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

// 获取反对称矩阵 
static Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}

#endif
