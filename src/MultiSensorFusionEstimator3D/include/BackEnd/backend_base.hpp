/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-13 18:43:59
 * @Description: 
 * @Others: 
 */
#pragma once 

#include "Common/color.hpp"
#include "Sensor/lidar_data_type.h"
#include "Common/keyframe.hpp"

namespace Slam3D {

    /**
     * @brief: 基于G2O的后端优化模块  
     * @details: 实现 
     * 1、融合激光里程计约束，平面先验约束，gnss 约束的滑动窗口优化
     * 2、回环之后进行全局优化  
     * @param {*}
     * @return {*}
     */    
    template<typename _FeatureT>
    class BackEndOptimizationBase
    {
        protected:
            std::string keyframes_save_path_ = "/home/lwh/code/lwh_ws-master/src/liv_slam-master/Map";  
            // 新添加的关键帧的处理队列
            std::vector<typename KeyFrame<_FeatureT>::Ptr> new_keyframe_queue_;
            // optOdom坐标系到map坐标系的变换     有GNSS的时候才有用
            Eigen::Isometry3d trans_odom2map_ = Eigen::Isometry3d::Identity();
            std::vector<typename KeyFrame<_FeatureT>::Ptr> wait_optimize_keyframes_;
            std::mutex keyframe_queue_mutex_;
            //std::thread backend_thread_;
        public:
            virtual ~BackEndOptimizationBase() {}
            /**
             * @brief: 添加激光里程计数据  
             * @details: 
             */            
            virtual void AddKeyFrame(FeatureInfo<_FeatureT> const& lidar_data, 
                                                                        Eigen::Isometry3d const& odom,
                                                                        Eigen::Isometry3d const& between_constraint) = 0; 
        protected:
            virtual void process() = 0;  
            virtual void dataParse() = 0;
            virtual void localOptimize() {};    // 局部优化   不一定都需要 
            virtual void globalOptimize() = 0; 
        
    };
}
