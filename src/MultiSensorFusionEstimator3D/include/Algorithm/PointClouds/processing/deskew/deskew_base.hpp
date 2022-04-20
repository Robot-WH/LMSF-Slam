/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-21 21:38:57
 * @Description:  去畸变的基类 
 * @Others: 
 */

#pragma once 

#include <pcl/point_cloud.h>

namespace Algorithm {

    using PointCloud = pcl::PointCloud<pcl::PointXYZI>; 

    /**
     * @brief:  基于外部传感器去除激光雷达畸变的基类 
     * @details: 外部传感器例如 IMU、wheel...
     */    
    class ExternalSensorDeskewBase 
    {
        public:
            ExternalSensorDeskewBase(float const& SCAN_PERIOD) : SCAN_PERIOD_(SCAN_PERIOD)
            {}
            virtual ~ExternalSensorDeskewBase() {}

        

        private:
    
        private:
            float SCAN_PERIOD_;     // 激光每一帧的时间
    };
}
