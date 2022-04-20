/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-24 17:01:51
 * @Description: 
 * @Others: 
 */

#ifndef _FILTER_FACTORY_HPP_
#define _FILTER_FACTORY_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace Algorithm {

template<class PointT>
using  VoxelGridPtr = std::unique_ptr<pcl::VoxelGrid<PointT>>;  
template<class PointT>
using RadiusOutlierRemovalPtr = std::unique_ptr<pcl::RadiusOutlierRemoval<PointT>>;  
template<class PointT>
using  StatisticalOutlierRemovalPtr = std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>>;  

/**
 * @brief: 构建降采样滤波器 voxel  grid 
 * @details 用voxel 中的所有点的均值代替其他点    
 * @param resolution voxel 的分辨率  
 */
template<class PointT>
VoxelGridPtr<PointT> make_voxelGrid(float const& resolution) 
{
    VoxelGridPtr<PointT> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(resolution, resolution, resolution);
    return std::move(voxelgrid);   
}

/**
 * @brief: 构建几何离群点滤波  
 * @details 
 * @param radius 考虑的半径 
 * @param min_neighbors 半径内最少的点的数量 
 */
template<class PointT>
RadiusOutlierRemovalPtr<PointT> make_radiusOutlierRemoval(float const& radius, 
                                                                                                                                      uint16_t const& min_neighbors) 
{
    RadiusOutlierRemovalPtr<PointT> rad(new pcl::RadiusOutlierRemoval<PointT>());
    rad->setRadiusSearch(radius);                                         
    rad->setMinNeighborsInRadius(min_neighbors);  
    return std::move(rad);   
}

/**
 * @brief: 构建统计离群点滤波  
 * @details  先遍历所有点，统计每个点距离最近的mean_k个邻居的距离，计算该距离的正态分布系数，
 *                      再遍历所有点，计算每个点距离最近的mean_k个邻居的平均距离， 如果该平均距离 在 μ ±k*σ  之外,
 *                      则滤除   
 * @param mean_k 考虑的邻居个数 
 * @param k 
 */
template<class PointT>
StatisticalOutlierRemovalPtr<PointT> make_statisticalOutlierRemoval(uint16_t const& mean_k, 
                                                                                                                                                    uint8_t const& k) 
{
    StatisticalOutlierRemovalPtr<PointT> sor;
    sor->setMeanK (mean_k);                                                                //设置在进行统计时考虑查询点邻近点数
    sor->setStddevMulThresh (k);                                     // 需要提出的点的平均距离大于标准差的倍数    μ ±k*σ  之外的点 
    return std::move(sor);  
}

}
#endif




