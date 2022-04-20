/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-24 12:21:42
 * @Description: 点云匹配前的预处理基类 
 * @Others: 
 */

#ifndef _PROCESS_BASE_HPP_
#define _PROCESS_BASE_HPP_

#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Sensor/lidar_data_type.h"

namespace Algorithm {
    using Slam3D::LidarData;
    using Slam3D::FeatureInfo;
    /**
     * @brief: 点云处理基类  
     */    
    template<typename _InPointT, typename _OutPointT>
    class PointCloudProcessBase {
        public:
            PointCloudProcessBase() 
            {}
            virtual ~PointCloudProcessBase() {}
            /**
             * @brief: 
             * @details: 
             * @param data_in 激光的点云数据 
             * @param data_out 输出结果 
             */            
            virtual void Process(LidarData<_InPointT> const& data_in,
                FeatureInfo<_OutPointT> &data_out) = 0;  
    };
}
#endif
