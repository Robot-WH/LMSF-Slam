/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-10 23:23:37
 * @Description: 多激光估计器的工厂函数  
 * @Others: 
 */

#pragma once 

#include "Common/read_param.hpp"
#include "Estimator/estimator/MultiLidar/MultiLidar_estimator_base.hpp"
#include "Estimator/estimator/MultiLidar/MultiLidar_estimator.hpp"
#include "factory/processing/pointcloud/pointcloud_processing_factory.hpp"
#include "factory/tracker/lidarTracker_factory.hpp"
#include "factory/estimator/multiLidar_estimator/multiLidar_estimator_factory_interface.hpp"
//#include "Algorithm/PointClouds/processing/FeatureExtract/LOAMFeatureProcessor_base.hpp"
#include "Algorithm/PointClouds/registration/ceres_edgeSurfFeatureRegistration.hpp"
#include "Algorithm/PointClouds/registration/edgeSurfFeatureRegistration.hpp"

namespace Slam3D {
    using Algorithm::RegistrationBase; 
    using Algorithm::CeresEdgeSurfFeatureRegistration;
/**
 * @brief:  采用LOAM 特征的多激光估计系统工厂  
 * @details:  多线雷达 16线， 32线 等
 * @param _PointType 最初使用的pcl 点云 点类型
 * @param _FeatureType 内部得到的特征的类型 
 */    
template<typename _PointType, typename _FeatureType>
class LoamFeatureMultiLidarEstimatorFactory : public MultiLidarEstimatorFactoryInterface<_PointType>
{   
private:
    using Base = MultiLidarEstimatorFactoryInterface<_PointType>;
    using TargetType = typename Base::MultiLidarEstimatorPtr;
    // 参数 
    std::string lidar_tracking_mode_;     // loam 或 scan - map  
    int lidar_num_;   // 激光的个数  

public:
    LoamFeatureMultiLidarEstimatorFactory() {}

    /**
     * @brief:  通过配置文件参数  创建多激光估计器  
     * @details:  基于 LOAM特征的版本 
     * @param config_path 配置文件路径 
     * @return {*}
     */    
    virtual TargetType Create(std::string config_path) override
    {
        // 先加载参数 
        TargetType estimator_ptr = nullptr;
        if (!LoadParameter(config_path))
        {
            return estimator_ptr; 
        }
        // 特征处理类型
        using FeatureProcessorPtr = typename MultiLidarEstimator<_PointType, 
            _FeatureType>::FeatureProcessorPtr;
        // 激光tracker 类型 
        using LidarTrackerPtr = typename MultiLidarEstimator<_PointType,
                _FeatureType>::LidarTrackerPtr;
        std::vector<LidarTrackerPtr> tracker_container;   
        std::vector<FeatureProcessorPtr> processor_container;  
        // 创建点云处理器  
        for (int i = 0; i < lidar_num_; i++)
        {
            // processor_container.emplace_back(
            //     new LOAMFeatureProcessorBase<_PointType, _FeatureType>(16, 2, 80)); 
        }
        // 创建点云tracker
        for (int i = 0; i < lidar_num_; i++)
        {
            if (lidar_tracking_mode_ == "scan_map")
            {
                 // 构造tracker  
                std::unique_ptr<LidarTrackerLocalMap<_FeatureType, 
                    RegistrationBase<_FeatureType>>> tracker(
                    new LidarTrackerLocalMap<_FeatureType, 
                    RegistrationBase<_FeatureType>>());
                // 设置local map   local map 名称 + local map 类型 
                tracker->SetLocalMap({"loam_edge", "loam_surf"}, "sliding_Localmap");  

                // using RegistrationAdapterType = RegistrationAdapterImpl<
                //     typename LidarTrackerLocalMap<_FeatureType>::LocalMapInput, 
                //     Sensor::FeatureInfo<_FeatureType>, RegistrationBase<_FeatureType>>;
                std::unique_ptr<RegistrationBase<_FeatureType>> registration_ptr;
                registration_ptr.reset(
                    new CeresEdgeSurfFeatureRegistration<_FeatureType>("loam_edge", "loam_surf")
                    // new EdgeSurfFeatureRegistration<_FeatureType>("loam_edge", "loam_surf")
                ); 
                tracker->SetRegistration(std::move(registration_ptr));  
                tracker_container.push_back(std::move(tracker)); 
            }
        }

        estimator_ptr.reset(new MultiLidarEstimator<_PointType, _FeatureType>(
            processor_container, tracker_container, lidar_num_
        ));
        return estimator_ptr; 
    }

private:
    virtual bool LoadParameter(std::string config_path) override
    {
        ParametersReader param_read(config_path); 
        if (!param_read.IsOpened()) 
        {
            std::cout<<"LoadParameter()：参数读取失败！"<<std::endl;
            return false; 
        }  
        param_read.ReadParam("lidar_tracking_mode", lidar_tracking_mode_);    
        // param_read.ReadParam("registration_mode", REGISTRATION_MODE);  
        // param_read.ReadParam("process.feature.method", FEATURE_TYPE);

        param_read.ReadParam("lidar_num", lidar_num_); 

        return true;  
    }
};

};
