/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-22 11:12:23
 * @Description:  旋转激光雷达的预处理
 * @Others: 
 */

#pragma once
#include "Sensor/lidar_data_type.h"

namespace Algorithm {
    
    using Slam3D::LidarData;  

    /**
     * @brief: 机械旋转雷达的预处理 
     * @details:  求解每个点的时间戳
     */    
    template<typename _PointT>
    class RotaryLidarPreProcess
    {
        protected:
            float SCAN_PERIOD_; // 0.1  
        public:
            RotaryLidarPreProcess(double SCAN_PERIOD = 0.1) 
            : SCAN_PERIOD_(SCAN_PERIOD) {}

            virtual void Process(LidarData<_PointT> &lidar_data)
            {
                float start_ori, end_ori;
                findStartEndAngle(lidar_data.point_cloud, start_ori, end_ori);     // 计算起始点和终点的角度  
                bool half_passed = false;
                for (size_t i = 0; i < lidar_data.point_cloud.size(); i++)
                {
                    float ori = -atan2(lidar_data.point_cloud.points[i].y, 
                                                        lidar_data.point_cloud.points[i].x);
                    if (!half_passed)
                    {
                        if (ori < start_ori - M_PI / 2)
                        {
                            ori += 2 * M_PI;
                        }
                        else if (ori > start_ori + M_PI * 3 / 2)
                        {
                            ori -= 2 * M_PI;
                        }
                        if (ori - start_ori > M_PI)
                        {
                            half_passed = true;
                        }
                    }
                    else
                    {
                        ori += 2 * M_PI;
                        if (ori < end_ori - M_PI * 3 / 2)
                        {
                            ori += 2 * M_PI;
                        }
                        else if (ori > end_ori + M_PI / 2)
                        {
                            ori -= 2 * M_PI;
                        }
                    }
                    float rel_time = (ori - start_ori) / (end_ori - start_ori) * SCAN_PERIOD_;
                    // 将scanID和相对时间设置到点中
                    setPoint(rel_time, lidar_data.point_cloud.points[i]); 
                }
            }

        protected:
            /**
             * @brief: 求解一帧激光起始和终止点角度 
             */    
            virtual void findStartEndAngle(pcl::PointCloud<_PointT> const& laser_cloud_in, 
                float &start_ori, float &end_ori)
            {
                start_ori = -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
                end_ori = -atan2(laser_cloud_in.points[laser_cloud_in.points.size() - 1].y,
                                laser_cloud_in.points[laser_cloud_in.points.size() - 1].x) + 2 * M_PI;
                if (end_ori - start_ori > 3 * M_PI)
                {
                    end_ori -= 2 * M_PI;
                }
                else if (end_ori - start_ori < M_PI)
                {
                    end_ori += 2 * M_PI;
                }
            }

            virtual void setPoint(float const& rel_time, _PointT &p);
    }; 
    
    /**
     * @brief: 求解旋转式激光线束模型
     * @details: 针对旋转式雷达 
     */      
    template<>
    void RotaryLidarPreProcess<pcl::PointXYZI>::setPoint(float const& rel_time, pcl::PointXYZI &p)
    {
        p.intensity = rel_time;
    }

}
