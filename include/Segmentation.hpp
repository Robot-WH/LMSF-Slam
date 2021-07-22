#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP


#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "omp.h"

typedef pcl::PointXYZI PointT;

class Segmentation
{
    public:
    /*
    // 提取出一个高度以下的点云 
    // out: clip_height一下的点云
    void clip_above(double clip_height, const pcl::PointCloud<PointT>::Ptr& in,
                                const pcl::PointCloud<PointT>::Ptr& floor)
    {
        pcl::ExtractIndices<PointT> cliper;
        // 设置要提取的输入点云
        cliper.setInputCloud(in);
        pcl::PointIndices indices;
        //omp_set_num_threads(2);
    #pragma omp for                // 没啥用 ??????
        for (size_t i = 0; i < in->points.size(); i++)
        {   // z轴高度小于 阈值   则放置到 indices   
            if (in->points[i].z <= clip_height)
            {
               floor->push_back(in->points[i]);     // 地面候选
               indices.indices.push_back(i);        // 将序序号提取出来  
            }
        }
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true);    //ture to remove the indices    剔除地面点
        cliper.filter(*in);          // 获取非地面点
    }
    */
   // 提取出一个高度以下的点云 
// out: clip_height一下的点云
// 返回剔除的点云
pcl::PointCloud<PointT> clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    pcl::ExtractIndices<PointT> cliper;
    pcl::PointCloud<PointT> removal;
    // 设置要提取的输入点云
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    //omp_set_num_threads(2);
#pragma omp for                // 没啥用 ??????
    for (size_t i = 0; i < in->points.size(); i++)
    {   // z轴高度大于 阈值   则放置到 indices   
        if (in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);        // 将序序号提取出来  
            removal.push_back(in->points[i]);    // 非地面点放置与  removal 
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);    //ture to remove the indices    剔除这部分 
    cliper.filter(*out);
    return removal;
}

/**
 * @brief filter points with non-vertical normals
 * @param cloud  input cloud
 * @return 法向量滤除的地面点
 */
pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& floor, pcl::PointCloud<pcl::PointXYZI>::Ptr& no_floor)  {
    // 法向量估计  
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(floor);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);       // 设置搜索方法
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);    // 法向量点云  
    ne.setKSearch(5);
    ne.compute(*normals);     // 求法向量

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    filtered->reserve(floor->size());
    //omp_set_num_threads(2);
#pragma omp for     // 没啥效果  
    for (int i = 0; i < floor->size(); i++) {
        // 与z轴向量点积   
        float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
        // 将夹角小于阈值的提取
        if (std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) 
            filtered->push_back(floor->at(i));
        else
            no_floor->push_back(floor->at(i));
    }
    std::cout<<"filtered size: "<<filtered->size()<<std::endl;         
    return filtered;
}

// 地面去除  RANSAC方法
// 输入: 待去除地面的点云
// 返回地面方程
Eigen::Vector4f floor_remove(pcl::PointCloud<PointT>::Ptr& cloud)    
{
    pcl::PointCloud<PointT>::Ptr no_floor_cloud(new pcl::PointCloud<PointT>());     // 地面的点云 
    pcl::PointCloud<PointT>::Ptr floor_cloud(new pcl::PointCloud<PointT>());     // 地面的点云 
    // filtering before RANSAC (height and normal filtering)
    pcl::PointCloud<PointT>::Ptr correct(new pcl::PointCloud<PointT>);           // 校正后的点云 
    pcl::transformPointCloud(*cloud, *correct, tilt_matrix);                     // 进行倾斜校正  
    *no_floor_cloud += clip_above(clip_height, correct, floor_cloud);            // 高度滤波
    *floor_cloud = *normal_filtering(floor_cloud, no_floor_cloud);               // 法线滤波
    // RANSAC提取平面
    // RANSAC 拟合平面 
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(floor_cloud));
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(0.3);                             // 与该平面距离小于该阈值的点为内点 
    ransac.computeModel();
    // 获取内点  
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // 提取非地面点  
    pcl::PointCloud<PointT>::Ptr outlier_cloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(floor_cloud);     // 从地面候选点中剔除掉地面点获得非地面点  
    extract.setIndices(inliers);     
    extract.setNegative(true);               //ture to remove the indices    剔除这部分内点的部分   即地面  
    extract.filter(*outlier_cloud);
    *no_floor_cloud += *outlier_cloud; 
    
    no_floor_cloud->width = no_floor_cloud->size();       // 点云的数量
    no_floor_cloud->height = 1;
    no_floor_cloud->is_dense = false;
    pcl::transformPointCloud(*no_floor_cloud, *cloud, static_cast<Eigen::Matrix4d>(tilt_matrix.inverse()));
    // 地面点数量
    std::cout<<"floor num: "<<inliers->indices.size()<<std::endl;
    // too few inliers    判定是否需要更新法向量
    // 数量达标
    if(inliers->indices.size() < floor_pts_thresh) {
        return {0,0,0,0};
    }
    // 获取平面法向量  
    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    int num = 0; 
    // 距离过大的点的数量阈值    超过这个值 地面就不理想了
    int num_thresh = inliers->indices.size()*0.1;
    float *data;
    // 判定模型的质量 
    for(int& i:inliers->indices)
    {
        data = floor_cloud->points[i].data;
        float dis = data[0]*coeffs[0]+data[1]*coeffs[1]+data[2]*coeffs[2]+data[3]*coeffs[3];     // 最好->0  
        if(fabs(dis)>0.2)
          num++;
        if(num>num_thresh)
          return {0,0,0,0};
    }
    std::cout<<"floor detect: "<<coeffs<<std::endl;
    //std::cout<<"bad point num: "<<num<<std::endl;
    if(!floor_init)
    {
      floor_init = true;
      Eigen::Vector3d z = {0,0,-1};
      Eigen::Vector3d e = {coeffs[0],coeffs[1],coeffs[2]};
      // 求旋转法向量   
      Eigen::Vector3d n = z.cross(e);
      double theta = atan2( n.norm(), z.dot(e));       // 旋转角  
      n.normalize();       // 单位化
      tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisd(theta, n).toRotationMatrix().inverse();    // 计算校正矩阵  
      std::cout<<"floor init ok !  tilt theta: "<<theta<<" n: "<<n<<std::endl;
      std::cout<<"check !  correct: "<<tilt_matrix*coeffs.cast<double>()<<std::endl;
    }
    
    return coeffs;
}

    private:
        // 法向量阈值   
        int normal_filter_thresh = 30;
        Eigen::Matrix4d tilt_matrix = Eigen::Matrix4d::Identity();     // 倾斜校正矩阵 
        bool floor_init = false;
        // 高度滤波 
        float clip_height=-0.5;  
        int floor_pts_thresh = 300; 
        int floor_normal_thresh = 30;
};





#endif