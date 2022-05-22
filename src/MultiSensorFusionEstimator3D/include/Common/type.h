/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-28 23:22:00
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * 6D位姿点云结构定义
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D     
    PCL_ADD_INTENSITY;  
    float roll;         
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                (float, x, x) (float, y, y)
                                (float, z, z) (float, intensity, intensity)
                                (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                (double, time, time))

typedef PointXYZIRPYT  PointTypePose;


