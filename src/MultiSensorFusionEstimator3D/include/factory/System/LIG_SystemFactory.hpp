/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: lwh
 * @Description: 激光+IMU+GNSS 系统构建工厂
 *                                
 */
#pragma once 

#include "Common/read_param.hpp"
#include "System/LIG_System.hpp"   
#include "factory/processing/pointcloud/pointcloud_processing_factory.hpp"
#include "factory/tracker/lidarTracker_factory.hpp"
#include "factory/registration/registration_factory.hpp"
#include "Algorithm/PointClouds/registration/ceres_edgeSurfFeatureRegistration.hpp"
#include "Algorithm/PointClouds/registration/edgeSurfFeatureRegistration.hpp"

namespace Slam3D {

    using Algorithm::PointCloudCommonProcess;  
    using Algorithm::RegistrationBase;
    using Algorithm::CeresEdgeSurfFeatureRegistration;
    using Algorithm::LOAMFeatureProcessorBase;
/**
 * @brief:  通用型多激光估计器   
 * @details:  如 NDT，点面ICP 等 
 * @param _PointType 最初使用的pcl 点云 点类型
 * @param _FeatureType 内部得到的特征的类型 
 */    
template<typename _PointType, typename _FeatureType>
class LidarImuGnssSystemFactory 
{   
    private:
        using SystemType = std::unique_ptr<LidarImuGnssFusionSystem<_PointType, _FeatureType>>; 
        // 特征处理类型
        using FeatureProcessorPtr = typename LidarImuGnssFusionSystem<_PointType, 
            _FeatureType>::FeatureProcessorPtr;
        // 激光tracker 类型 
        using LidarTrackerPtr = typename LidarImuGnssFusionSystem<_PointType,
                _FeatureType>::LidarTrackerPtr;
    public:

        LidarImuGnssSystemFactory() {}

        /**
         * @brief:  通过配置文件参数  创建多激光估计器  
         * @details:  基于 通用版本 ，直接法以及特征法 
         * @param config_path 配置文件路径 
         * @return {*}
         */    
        SystemType Create(std::string config_path)
        {
            ParametersReader param_read(config_path); 
            if (!param_read.IsOpened()) 
            {
                std::cout<<"LoadParameter()：参数读取失败！"<<std::endl;
                return nullptr; 
            }  
            // 先加载参数 
            SystemType system_ptr = nullptr;
            std::vector<LidarTrackerPtr> tracker_container;   
            std::vector<FeatureProcessorPtr> processor_container;  
            int lidar_num;   // 激光的个数  
            param_read.ReadParam("lidar_num", lidar_num); 
            // 创建点云tracker
            for (int i = 0; i < lidar_num; i++)
            {
                std::string lidar_tracking_mode;     // loam 或 scan - map  
                param_read.ReadParam("tracker.working_mode", lidar_tracking_mode);    
                // 匹配的模式  1、scan-map   2、loam (scan-scan + scan-map)
                if (lidar_tracking_mode == "scan_map")
                {
                    std::string local_map_type;
                    param_read.ReadParam("tracker.local_map.type", local_map_type);    
                    int time_sliding_window_size;
                    param_read.ReadParam("tracker.local_map.sliding_window.size", time_sliding_window_size);    
                    // 设置local map   local map 名称 + local map 类型 
                    // 匹配的方法
                    std::string registration_method;    
                    param_read.ReadParam("tracker.scan_map.registration_method", registration_method);
                    // 直接法 - NDT  
                    if (registration_method == "ndt")   
                    {
                        std::string ndt_type;     // loam 或 scan - map  
                        float ndt_resolution;
                        float transformation_epsilon;
                        float step_size;
                        float maximum_iterations;
                        float num_threads;
                        string nn_search_method; 
                        param_read.ReadParam("tracker.scan_map.registration.ndt.type", ndt_type);    
                        param_read.ReadParam("tracker.scan_map.registration.ndt.resolution", ndt_resolution);    
                        param_read.ReadParam("tracker.scan_map.registration.ndt.epsilon", transformation_epsilon);    
                        param_read.ReadParam("tracker.scan_map.registration.ndt.step_size", step_size);    
                        param_read.ReadParam("tracker.scan_map.registration.ndt.maximum_iterations", maximum_iterations);    
                        param_read.ReadParam("tracker.scan_map.registration.ndt.num_threads", num_threads);    
                        param_read.ReadParam("tracker.scan_map.registration.ndt.search_method", nn_search_method);  
                        // 使用NDT_OMP 库   
                        if (ndt_type == "ndt_omp")
                        {
                            // 构造tracker  
                            std::unique_ptr<LidarTrackerLocalMap<_FeatureType, 
                                pcl::Registration<_PointType, _PointType>>> tracker(
                                new LidarTrackerLocalMap<_FeatureType, 
                                pcl::Registration<_PointType, _PointType>>());
                            // 创建NDT
                            NdtOmpPtr<_PointType> ndt_ptr = 
                                make_ndtOmp<_PointType>(ndt_resolution, transformation_epsilon, 
                                step_size, maximum_iterations, num_threads, nn_search_method);  
                            tracker->SetRegistration(std::move(ndt_ptr));     // 匹配算法设置
                            if (local_map_type == "time_sliding_window")
                            {
                                tracker->SetLocalMap({POINTS_PROCESSED_NAME}, 
                                                                                local_map_type, 
                                                                                time_sliding_window_size);  
                            }
                            
                            tracker_container.push_back(std::move(tracker)); 
                        }
                        // 创建预处理器     将采样 + 距离滤波 ，对于NDT 不需要进行离群点滤波 ，因为ndt 本身具有滤除离群点的能力
                        PointCloudCommonProcess<_PointType> process(POINTS_PROCESSED_NAME);
                        // 降采样  
                        string downSample_type;
                        param_read.ReadParam("preprocess.downSample.type", downSample_type);    
                        if (downSample_type == "VoxelGrid")
                        {
                            float cell_size;
                            param_read.ReadParam("preprocess.downSample.voxelGrid.voxel_size", cell_size);    
                            process.SetVoxelGrid(downSample_type, cell_size);
                        }
                        // 距离滤波 
                        float distance_max, distance_min;
                        param_read.ReadParam("preprocess.distanceFilter.max", distance_max);    
                        param_read.ReadParam("preprocess.distanceFilter.min", distance_min);    
                        process.SetDistanceFilter(distance_min, distance_max);
                        // 创建点云处理器 
                        processor_container.emplace_back(
                            new PointCloudCommonProcess<_PointType>(process)); 
                    } 
                    else if (registration_method == "sparse_point_plane_icp")
                    {
                        // 构造tracker  
                        std::unique_ptr<LidarTrackerLocalMap<_FeatureType, 
                            RegistrationBase<_FeatureType>>> tracker(
                            new LidarTrackerLocalMap<_FeatureType, 
                            RegistrationBase<_FeatureType>>());

                        std::unique_ptr<RegistrationBase<_FeatureType>> registration_ptr;
                        registration_ptr.reset(
                            new CeresEdgeSurfFeatureRegistration<_FeatureType>("", "filtered")
                            // new EdgeSurfFeatureRegistration<_FeatureType>("loam_edge", "loam_surf")
                        ); 
                        tracker->SetRegistration(std::move(registration_ptr));  
                        tracker->SetLocalMap({"filtered"}, "sliding_Localmap");  
                        tracker_container.push_back(std::move(tracker)); 
                        // 创建预处理器     将采样 + 距离滤波 ，对于NDT 不需要进行离群点滤波 ，因为ndt 本身具有滤除离群点的能力
                        PointCloudCommonProcess<_PointType> process("filtered");
                        // 将采样  
                        string downSample_type;
                        param_read.ReadParam("preprocess.downSample.type", downSample_type);    
                        if (downSample_type == "VoxelGrid")
                        {
                            float cell_size;
                            param_read.ReadParam("preprocess.downSample.voxelGrid.voxel_size", cell_size);    
                            process.SetVoxelGrid(downSample_type, cell_size);
                        }
                        // 距离滤波 
                        float distance_max, distance_min;
                        param_read.ReadParam("preprocess.distanceFilter.max", distance_max);    
                        param_read.ReadParam("preprocess.distanceFilter.min", distance_min);    
                        process.SetDistanceFilter(distance_min, distance_max);
                        // 创建点云处理器 
                        processor_container.emplace_back(
                            new PointCloudCommonProcess<_PointType>(process)); 
                    } 
                    else if (registration_method == "feature_based")    
                    {
                         // 构造tracker  
                        std::unique_ptr<LidarTrackerLocalMap<_FeatureType, 
                            RegistrationBase<_FeatureType>>> tracker(
                            new LidarTrackerLocalMap<_FeatureType, 
                            RegistrationBase<_FeatureType>>());

                        std::unique_ptr<RegistrationBase<_FeatureType>> registration_ptr;
                        registration_ptr.reset(
                            new CeresEdgeSurfFeatureRegistration<_FeatureType>("loam_edge", "loam_surf")
                            // new EdgeSurfFeatureRegistration<_FeatureType>("loam_edge", "loam_surf")
                        ); 
                        tracker->SetRegistration(std::move(registration_ptr));  
                        tracker->SetLocalMap({"loam_edge", "loam_surf"}, "sliding_Localmap");  
                        tracker_container.push_back(std::move(tracker)); 
                        // 创建点云处理器 
                        processor_container.emplace_back(
                            new LOAMFeatureProcessorBase<_PointType, _FeatureType>(16, 2, 80)); 
                    }
                }
            }
            system_ptr.reset(new LidarImuGnssFusionSystem<_PointType, _FeatureType>(
                processor_container, tracker_container, lidar_num
            ));
            return system_ptr; 
        }
}; // class
} // namespace 