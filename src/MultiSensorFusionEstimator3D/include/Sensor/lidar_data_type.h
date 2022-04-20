/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-04 11:43:04
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Slam3D
{
    using type_id = uint8_t;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief:  一个雷达的数据
     */    
    template<typename _PointT>
    struct LidarData
    {
        pcl::PointCloud<_PointT> point_cloud;  

    };
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief  多激光点云数据结构 
     * @details 传入到激光估计器的数据  
     */
    template<class _PointT>
    struct MultiLidarData
    {
       double timestamp;    // In second.
       std::vector<std::pair<type_id, LidarData<_PointT>>> all_lidar_data;      // 点云
    };

    template<typename _PointType>
    using FeaturePointCloudContainer = std::unordered_map<std::string, 
            typename pcl::PointCloud<_PointType>::ConstPtr>;

    /**
     * @brief:  保存特征点各种信息数据 
     * @details: 
     * @param _PointType 特征点云数据类型  
     */    
    template<typename _PointType>
    struct FeatureInfo
    {   
        double time_stamp_ = 0;
        // 点类数据      <数据标识名，数据体>
        FeaturePointCloudContainer<_PointType> pointcloud_data_;     
    }; 

    //////////////////////////////////////////////////////结果信息发布 
    /**
    * @brief:  激光帧信息
    * @details 单独一个激光的数据   用与可视化
    * @param _PointcloudType 该激光的数据类型   
    */    
    template<typename _PointType>
    struct LidarResultInfo
    {
        double time_stamps_;  
        Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();    // 雷达的位姿
        std::unordered_map<std::string, 
            typename pcl::PointCloud<_PointType>::ConstPtr> feature_point_;   // 可视化 点云  
        std::unordered_map<std::string, 
            typename pcl::PointCloud<_PointType>::ConstPtr> local_map_;   // 可视化 局部  
    };

    // 保存多激光一个帧数据的特征
    template<typename _PointType>
    using MultiLidarResultInfo
        = std::unordered_map<type_id, LidarResultInfo<_PointType>>;  // <激光的ID，点云>

}
