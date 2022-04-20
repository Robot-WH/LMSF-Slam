/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-03 17:34:47
 * @Description: 参考loam的特征提取  
 * @Others:  ref: floam   
 */

#ifndef _LOAMFEATUREPROCESSOR_BASE_HPP_
#define _LOAMFEATUREPROCESSOR_BASE_HPP_

#include "Sensor/lidar_data_type.h"
#include "../common_processing.hpp"
#include "../Filter/voxel_grid.hpp"

namespace Algorithm {

    /**
     * @brief: loam的特征提取方法基类     后续的子类通过装饰模式对其进行扩展 
     * @details 对于多线雷达采用分层的方式提取特征 
     *                     适合于线束不高的雷达使用，例如 16线、32线等 
     */    
    template<typename _InputPointT, typename _OutputFeatureT>
    class LOAMFeatureProcessorBase : public PointCloudProcessBase<_InputPointT, _OutputFeatureT>
    {
        private:
            using InputPointCloud = pcl::PointCloud<_InputPointT>;  
            using FeaturePointCloud = pcl::PointCloud<_OutputFeatureT>;  
            using InputPointCloudPtr = typename pcl::PointCloud<_InputPointT>::Ptr; 
            using FeaturePointCloudPtr = typename pcl::PointCloud<_OutputFeatureT>::Ptr; 
            using InputPointCloudConstPtr = typename pcl::PointCloud<_InputPointT>::ConstPtr; 
            using Base = PointCloudProcessBase<_InputPointT, _OutputFeatureT>; 
        public:
            LOAMFeatureProcessorBase(uint16_t const& N_SCANS_, float min_distance = 0, 
                                                                        float max_distance = 9999, float edge_thresh = 1, 
                                                                        float surf_voxel_grid_size = 0.1, bool RemovalBadPoints = true)    
            : Base(), N_SCANS_(N_SCANS_), min_distance_(min_distance), 
                max_distance_(max_distance), edge_thresh_(edge_thresh), 
                check_bad_points_(RemovalBadPoints)
            {
                std::cout<<"LOAMFeatureProcessor, N_SCANS_: "<<N_SCANS_<<" min_distance_: "
                <<min_distance_<<" max_distance_: "<<max_distance_<<" surf_voxel_grid_size: "<<surf_voxel_grid_size
                <<" edge_thresh: "<<edge_thresh_<<" RemovalBadPoints: "<<check_bad_points_
                <<std::endl;
                // 设置降采样滤波器
                down_sampling_edge_.Reset("VoxelGrid", surf_voxel_grid_size);
                down_sampling_surf_.Reset("VoxelGrid", 2 * surf_voxel_grid_size);
            }

            /**
             * @brief: 
             * @details: 
             * @param data_in 输入的激光数据 
             * @param data_out 输出特征结果   
             * @return 
             */            
            void Process(LidarData<_InputPointT> const& data_in, 
                FeatureInfo<_OutputFeatureT> &data_out) override
            {
                FeaturePointCloudPtr pc_out_edge(new FeaturePointCloud());    // 线特征
                FeaturePointCloudPtr pc_out_surf(new FeaturePointCloud());      // 面特征

                std::vector<InputPointCloudPtr> laserCloudScans;                    // 保存每一个scan的激光点 
                // 首先给输入的多线激光数据分scan
                splitScan(data_in.point_cloud, laserCloudScans);
                // 遍历雷达的每一个scan 分层提取特征
                for (int i = 0; i < N_SCANS_; i++)
                {
                    if (laserCloudScans[i]->points.size() < 20) continue;  
                    int total_points = laserCloudScans[i]->points.size() - 10;   // 需要提特征的总数 剔除掉前后5个点 不提特征
                    if (total_points < 6) continue;  
                    int sector_length = (int)((total_points / 6) + 0.5);        // 四舍五入      
                    std::vector<int> disable_point;                                         // 标记被选择的点  
                    disable_point.resize(laserCloudScans[i]->points.size(), 0); 
                    std::vector<int> is_edge_points;                                         // 标记被选择的点  
                    is_edge_points.resize(laserCloudScans[i]->points.size(), 0); 
                    // 检查该边缘特征是否合格
                    if (check_bad_points_)
                    {
                        checkBadEdgePoint(laserCloudScans[i], disable_point);  
                    }
                    // 每一个scan 分成6个sector 进行
                    for(int k = 0; k < 6; k++)
                    {
                        int sector_start = 5 + sector_length * k;      // 该sector起始点在laserCloudScans[i]的序号 
                        int sector_end = sector_start + sector_length -1;  
                        if (k == 5)
                        {
                            sector_end = (int)laserCloudScans[i]->points.size() - 6; 
                        }
                        // 保存这个sector的曲率信息 
                        std::vector<CurvatureInfo> cloudCurvature; 
                        cloudCurvature.reserve(total_points);  
                        // 计算这个sector 所有点的曲率 
                        for(int j = sector_start; j <= sector_end; j++)
                        {
                            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x 
                                                        + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x 
                                                        + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x 
                                                        + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x
                                                        + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x 
                                                        + laserCloudScans[i]->points[j + 5].x;
                            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y 
                                                        + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y 
                                                        + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y 
                                                        + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y 
                                                        + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y 
                                                        + laserCloudScans[i]->points[j + 5].y;
                            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z 
                                                        + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z 
                                                        + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z 
                                                        + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z 
                                                        + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z 
                                                        + laserCloudScans[i]->points[j + 5].z;
                            cloudCurvature.emplace_back(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
                        }
                        featureExtractionFromSector(laserCloudScans[i], cloudCurvature, disable_point, 
                                                                                      is_edge_points, pc_out_edge, pc_out_surf);
                    }
                }
                            
                data_out.pointcloud_data_.insert(make_pair("loam_edge", std::move(pc_out_edge)));  
                data_out.pointcloud_data_.insert(make_pair("loam_surf", std::move(pc_out_surf)));  
            } 


        protected:
            class  CurvatureInfo
            {
                public:
                    int id;
                    double value;  // 曲率值 
                    CurvatureInfo(int const& id_in, double const& value_in) 
                    : id(id_in), value(value_in) {}
            };

            /**
             * @brief: 在一个scan的sector中提取特征
             * @param pc_in 该scan的全部点
             * @param cloudCurvature 该sector中点的曲率信息 
             * @return 
             */            
            void featureExtractionFromSector(InputPointCloudConstPtr const& pc_in, 
                                                                                    std::vector<CurvatureInfo>& cloudCurvature, 
                                                                                    std::vector<int> &disable_point,
                                                                                    std::vector<int> &is_edge_points,
                                                                                    FeaturePointCloudPtr& pc_out_edge, 
                                                                                    FeaturePointCloudPtr& pc_out_surf)
            {
                std::sort(cloudCurvature.begin(), cloudCurvature.end(), 
                    [](const CurvatureInfo & a, const CurvatureInfo & b)
                { 
                    return a.value < b.value;  // 升序排序，   > 为降序
                });
                int PickedEdgeNum = 0;
                // 曲率从大到小遍历
                for (int i = cloudCurvature.size() - 1; i >= 0; i--)
                {
                    int ind = cloudCurvature[i].id; 
                    // 如果该点可以被选
                    if (disable_point[ind] == 0)
                    {
                        //  边缘特征的阈值     边缘特征的阈值大一点   这样提取的较为严谨  
                        if(cloudCurvature[i].value <= edge_thresh_)
                        {
                            break;
                        }
                        PickedEdgeNum++;
                        // 最多选择20个特征出来用 
                        if (PickedEdgeNum <= 20)
                        {
                            pc_out_edge->push_back(pc_in->points[ind]);    
                            is_edge_points[ind] = 1;
                        }
                        else
                        {
                            break;
                        }
                        // 防止聚集    该点临近的5个点都不能提取边缘特征 
                        // 与距离没关，距离近的自然不能提取特征，距离远的则在计算曲率时不准确更加不应该提特征
                        for(int k = 1; k <= 5; k++)
                        {
                            int n = ind + k >= disable_point.size() ? disable_point.size() - 1 :  ind + k;  
                            disable_point[n] = 1;
                        }
                        //  判断前5个点是否聚集   
                        for(int k=-1;k>=-5;k--)
                        {
                            int n = ind + k < 0 ? 0 :  ind + k;  
                            disable_point[n] = 1;
                        }
                    }
                }
                // 剩下的全部当成面特征    
                for (int i = 0; i <= (int)cloudCurvature.size() - 1; i++)
                {
                    int ind = cloudCurvature[i].id; 
                    // 不要考虑阈值  因为 由于靠近边缘点的原因，有些平面点曲率计算出来会很大, 这样会漏掉一些表面点
                    // 只要不是边缘点  全部作为表面点  
                   if (is_edge_points[ind] == 0)
                   {
                        pc_out_surf->push_back(pc_in->points[ind]);
                   }
                }
            }

            /**
             * @brief: 检测出不好的边缘点   提前标记出来  
             * @details:  不好的点主要是
             * 
             * @param pc_in 处理的某一个scan
             * @param disable_point 标记 
             */            
            virtual void checkBadEdgePoint(
                InputPointCloudConstPtr const& pc_in, std::vector<int> &disable_point) 
            {
                int scan_num = pc_in->points.size();
                for (int j = 5; j < scan_num - 6; j++)
                {
                    // 计算当前点和下一个点的夹角  
                    double angle_curr = atan2(pc_in->points[j].x, pc_in->points[j].y);  
                    double angle_after = atan2(pc_in->points[j + 1].x, pc_in->points[j + 1].y);  
                    double delta_angle = fabs(angle_curr - angle_after);  
                    if (delta_angle > M_PI) {
                        delta_angle = M_PI * 2 - delta_angle;  
                    }
                    // 角度距离为1度以上  则认为不连续   不应该参与
                    if (delta_angle > 0.0175)
                    {
                        disable_point[j + 5] = 1;
                        disable_point[j + 4] = 1;
                        disable_point[j + 3] = 1;
                        disable_point[j + 2] = 1;
                        disable_point[j + 1] = 1;
                        disable_point[j - 5] = 1;
                        disable_point[j - 4] = 1;
                        disable_point[j - 3] = 1;
                        disable_point[j - 2] = 1;
                        disable_point[j - 1] = 1;
                        disable_point[j] = 1;
                        j = j + 4;  
                        continue;  
                    }
                    // 检测遮挡  
                    double distance_curr = sqrt(pc_in->points[j].x * pc_in->points[j].x 
                                                                                + pc_in->points[j].y * pc_in->points[j].y
                                                                                + pc_in->points[j].z * pc_in->points[j].z);  
                    double distance_after = sqrt(pc_in->points[j + 1].x * pc_in->points[j + 1].x 
                                                                    + pc_in->points[j + 1].y * pc_in->points[j + 1].y
                                                                    + pc_in->points[j + 1].z * pc_in->points[j + 1].z);  
                    double angle;
                    if (distance_curr < distance_after)
                    {
                        angle = atan2(distance_curr * delta_angle, distance_after - distance_curr); 
                    } else {
                        angle = atan2(distance_after * delta_angle, distance_curr - distance_after); 
                    }

                    if (angle <= 0.17)     // 夹角小于 10度   则认为出现了遮挡  
                    {
                        if (distance_curr < distance_after)
                        {
                            disable_point[j + 5] = 1;
                            disable_point[j + 4] = 1;
                            disable_point[j + 3] = 1;
                            disable_point[j + 2] = 1;
                            disable_point[j + 1] = 1;
                            j = j + 4;  
                        } else {
                            disable_point[j] = 1;
                            disable_point[j - 1] = 1;
                            disable_point[j - 2] = 1;
                            disable_point[j - 3] = 1;
                            disable_point[j - 4] = 1;
                            disable_point[j - 5] = 1;
                        } 
                    }

                }
            }

            /**
             * @brief: 计算分线束模型  
             * @details 包含距离滤波  
             * @param[in] pc_in 输入点云
             * @param[out] laserCloudScans 保存各个scan层点的容器 , 并且是按顺序排列的
             */            
            virtual void splitScan(InputPointCloud const& pc_in, std::vector<InputPointCloudPtr> &laserCloudScans)
            {
                for (int i=0; i<N_SCANS_; i++) 
                {
                    laserCloudScans.emplace_back(new PointCloud());     
                }

                for (int i = 0; i < (int)pc_in.points.size(); i++)
                {
                    int scanID = 0;
                    double distance = sqrt(pc_in.points[i].x * pc_in.points[i].x 
                                                                    + pc_in.points[i].y * pc_in.points[i].y);
                    if (distance > max_distance_ || distance < min_distance_) 
                    {
                        continue;   // 距离滤波                           
                    }                   
                    // 垂直角度    
                    double angle = atan(pc_in.points[i].z / distance) * 180 / M_PI;
                    // 计算位于哪一个scan上 
                    if (N_SCANS_ == 16)
                    {
                        scanID = int((angle + 15) / 2 + 0.5);
                        if (scanID > (N_SCANS_ - 1) || scanID < 0)
                        {
                            continue;
                        }
                    }
                    else if (N_SCANS_ == 32)
                    {
                        scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                        if (scanID > (N_SCANS_ - 1) || N_SCANS_ < 0)
                        {
                            continue;
                        }
                    }
                    else if (N_SCANS_ == 64)
                    {   
                        if (angle >= -8.83)
                            scanID = int((2 - angle) * 3.0 + 0.5);
                        else
                            scanID = N_SCANS_ / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                        if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        printf("wrong scan number\n");
                    }
                    laserCloudScans[scanID]->push_back(pc_in.points[i]); 
                }
            }

        protected:
            uint16_t N_SCANS_;   
            float edge_thresh_;      // 提取边缘点的曲率阈值  
        private:
            bool check_bad_points_; 
            float min_distance_, max_distance_;   // 有效距离相关
            VoxelGridFilter<_OutputFeatureT> down_sampling_edge_; 
            VoxelGridFilter<_OutputFeatureT> down_sampling_surf_;  
    }; // class LOAMFeatureProcessorBase
} // namespace 
#endif