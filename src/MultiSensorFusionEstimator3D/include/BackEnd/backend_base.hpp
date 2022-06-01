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
#include "pose_graph_database.hpp"
#include "LoopDetection/loopDetection.hpp"

namespace Slam3D {

    template<typename _T>
    struct KeyFrameInfo
    {
        double time_stamps_;  
        std::deque<KeyFrame> new_keyframes_;
        std::deque<Vertex> vertex_database_; 
        std::deque<Edge> edge_database_; 
    };

    template<typename _T>
    struct LocalizationPointsInfo
    {
        double time_stamps_;  
        std::unordered_map<std::string, typename pcl::PointCloud<_T>::ConstPtr> scan_; 
        std::unordered_map<std::string, typename pcl::PointCloud<_T>::ConstPtr> map_;
    };

    template<typename _FeatureT>
    class BackEndOptimizationBase
    {
        protected:
            using KeyFramePtr = typename KeyFrame::Ptr; 
            using KeyFrameConstPtr = typename KeyFrame::ConstPtr; 

            std::string keyframes_save_path_ = "/home/lwh/code/lwh_ws-master/src/liv_slam-master/Map";  
            // 新添加的关键帧的处理队列
            std::deque<KeyFrame> new_keyframe_queue_;
            std::deque<FeaturePointCloudContainer<_FeatureT>> new_keyframe_points_queue_;
            // optOdom坐标系到map坐标系的变换     有GNSS的时候才有用
            Eigen::Isometry3d trans_odom2map_ = Eigen::Isometry3d::Identity();
            std::mutex keyframe_queue_mutex_;
            boost::shared_mutex keyframe_queue_sm_; 
            //std::thread backend_thread_;
        public:
            virtual ~BackEndOptimizationBase() {}
            /**
             * @brief: 添加激光里程计数据  
             * @details: 
             */            
            virtual void AddKeyFrame(CloudContainer<_FeatureT> const& lidar_data, 
                                                                        Eigen::Isometry3d const& odom,
                                                                        Eigen::Isometry3d const& between_constraint) = 0; 
            virtual void SetLoopDetection(std::shared_ptr<loopDetection<_FeatureT>> const& loop_detect) = 0; 
            virtual void Load() = 0; 
            virtual void ForceGlobalOptimaze() = 0; 
        protected:
            virtual void mapping() = 0;     // 处理线程
            virtual bool optimize() = 0;  
    }; // class
} // namespace 
