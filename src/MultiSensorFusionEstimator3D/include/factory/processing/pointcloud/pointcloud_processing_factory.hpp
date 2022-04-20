/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-11 19:50:06
 * @Description: 
 * @Others: 
 */

#ifndef _POINTCLOUD_PROCESSING_FACTORY_HPP_
#define _POINTCLOUD_PROCESSING_FACTORY_HPP_

#include "Common/parameters.h"
#include "Algorithm/PointClouds/processing/common_processing.hpp"
//#include "Algorithm/PointClouds/processing/FeatureExtract/LOAMFeatureProcessor_base.hpp"

namespace Slam3D {

    // using Algorithm::PclCommonProcessing;  
    // using Algorithm::LOAMFeatureProcessorBase; 

    // template<class _PointType>
    // using PclCommonProcessingPtr = std::unique_ptr<PclCommonProcessing<_PointType>>;  
    // using FeatureProcessingPtr = std::unique_ptr<LOAMFeatureProcessorBase>;  
    // /**
    //  * @brief: 点云公共处理的工厂函数         
    //  */    
    // PclCommonProcessingPtr<PointType> make_commonProcessor()
    // {
    //     PclCommonProcessingPtr<PointType> common_processor(
    //                                     new PclCommonProcessing<PointType>(removal_nan_));  
    //     if (DOWNSAMPLING_TYPE_ == "VoxelGrid") 
    //     {
    //         common_processor->SetVoxelGrid(DOWNSAMPLING_TYPE_, voxel_grid_resolution_);   
    //     }
    //     if (OUTLIERREMOVAL_TYPE_ == "Geometric")
    //     {   
    //         common_processor->SetOutlierRemoval(OUTLIERREMOVAL_TYPE_, 
    //                                                                                                 radiusOutlierRemoval_radius_, 
    //                                                                                                 radiusOutlierRemoval_minNum_);  
    //     }
    //     common_processor->SetDistanceFilter(distanceFilter_min_, distanceFilter_max_); 
    //     return std::move(common_processor);  
    // }

    // /**
    //  * @brief: 点云特征提取的工厂函数         
    //  */    
    // FeatureProcessingPtr make_featureProcessor()
    // {
    //     FeatureProcessingPtr processor(new LOAMFeatureProcessorBase(
    //                                                                             SCAN_NUM_, distanceFilter_min_, 
    //                                                                             distanceFilter_max_, 0.1, 0.1, true));  
    //     return std::move(processor);  
    // }

};
#endif
