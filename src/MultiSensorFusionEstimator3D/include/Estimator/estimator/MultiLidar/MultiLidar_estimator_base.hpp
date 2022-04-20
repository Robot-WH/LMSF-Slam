/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-27 16:38:57
 * @Description: 
 * @Others: 
 */

#pragma once

#include "Estimator/states.hpp"
#include "Sensor/sensor.hpp"
#include "LidarTracker/LidarTrackerBase.hpp"
#include "Common/data_manager.hpp"

namespace Slam3D{
    /**
     * @brief:  多激光融合估计器抽象基类 
     */    
    template<typename _PointT>
    class MultiLidarEstimatorBase
    {
        public:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            MultiLidarEstimatorBase(uint8_t NUM_OF_LIDAR = 2, 
                                                                uint8_t EXTRINSIC_ESTIMATE = 1,   // 外参估计默认开启
                                                                uint8_t EXTRINSIC_OPTI = 0 // 外参在线优化默认关闭 
            ) : EXTRINSIC_CALIB_STATUS_(0), 
                NUM_OF_LIDAR_(NUM_OF_LIDAR), 
                EXTRINSIC_ESTIMATE_(EXTRINSIC_ESTIMATE), 
                EXTRINSIC_OPTI_(EXTRINSIC_OPTI)
            {}
            virtual void Process(MultiLidarData<_PointT> const& data) = 0;     
            virtual void processMeasurements() = 0;  
            virtual void process() = 0;   
        protected:
            bool b_system_inited_ = false;   
            uint8_t EXTRINSIC_CALIB_STATUS_;     // 外参估计
            uint8_t EXTRINSIC_ESTIMATE_;  // 是否进行外参估计
            uint8_t EXTRINSIC_OPTI_;  //  外参在线优化 标志
            uint8_t NUM_OF_LIDAR_;   
            std::mutex m_buf_;  
    }; // class MultiLidarEstimatorBase 
}// namespace MultiLidarEstimator
