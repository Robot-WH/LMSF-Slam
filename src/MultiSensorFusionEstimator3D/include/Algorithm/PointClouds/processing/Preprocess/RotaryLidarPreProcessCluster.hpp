/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-09 17:00:25
 * @Description:  旋转激光雷达预处理   增加聚类滤波
 * @Others: 
 */

#pragma once
#include "RotaryLidar_preprocessing.hpp"

namespace Algorithm {

    /**
     * @brief: 机械旋转雷达的预处理 
     * @details:  decorate 模式 
     */    
    template<typename _PointT>
    class RotaryLidarPreProcessCluster : public RotaryLidarPreProcess<_PointT>
    {
        private:
            RotaryLidarPreProcess<_PointT> base_preprocess_;  
        public:
            RotaryLidarPreProcessCluster(double SCAN_PERIOD = 0.1) 
            : RotaryLidarPreProcess<_PointT>(SCAN_PERIOD)
            , base_preprocess_(SCAN_PERIOD)
            {}

            virtual void Process(LidarData<_PointT> &lidar_data)
            {
      
            }

        protected:

            /**
             * @brief:  构造距离图像 
             */            
            void projectPointCloud() 
            {
                float verticalAngle, horizonAngle, range;
                uint32_t rowIdn, columnIdn, index, cloudSize;
                _PointT thisPoint;

                cloudSize = laserCloudIn->points.size();

                for (uint32_t i = 0; i < cloudSize; ++i) 
                {
                    thisPoint.x = laserCloudIn->points[i].x;
                    thisPoint.y = laserCloudIn->points[i].y;
                    thisPoint.z = laserCloudIn->points[i].z;
                    // 获取线数
                    rowIdn = getScanID(thisPoint.x, thisPoint.y, thisPoint.z); 
                    if (rowIdn < 0) continue;
                    // 求解横向index   
                    horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;    // atan2(y, x) 
                    /**
                     * @TODO: 改一下排列顺序  
                     */
                    columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + SCAN_NUM / 2;
                    if (columnIdn >= SCAN_NUM) columnIdn -= SCAN_NUM;

                    if (columnIdn < 0 || columnIdn >= SCAN_NUM) continue;

                    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y +
                                thisPoint.z * thisPoint.z);
                    rangeMat.at<float>(rowIdn, columnIdn) = range;

                    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

                    index = columnIdn + rowIdn * SCAN_NUM;
                    fullCloud->points[index] = thisPoint;

                    fullInfoCloud->points[index].intensity = range;
                }
            }

            int getScanID(double const& x, double const& y, double const& z)
            {
                int scanID = 0;  
                // 垂直的角度  
                angle = atan2(z, sqrt(x * x + y * y)) * 180 / M_PI;
                // 计算位于哪一个scan上 
                if (N_SCANS_ == 16)
                {
                    scanID = int((angle + 15) / 2 + 0.5);
                    if (scanID > (N_SCANS_ - 1) || scanID < 0)
                    {
                        return -1;
                    }
                }
                else if (N_SCANS_ == 32)
                {
                    scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                    if (scanID > (N_SCANS_ - 1) || N_SCANS_ < 0)
                    {
                        return -1;
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
                        return -1;
                    }
                }
                else
                {
                    return -1;  
                }
                return scanID;  
            }
    
            
    }; 
    


}


