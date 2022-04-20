/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-10 01:15:02
 * @Description: 基于距离图像的地面检测
 * @Others: 
 */
#pragma once

namespace Algorithm {

    /**
     * @brief: 基于距离图像的地面检测方法
     */    
    template<typename _PointT>
    class RangeImageGroundDetect
    {
        private:

        public:
            void Detect()
            {
                size_t lowerInd, upperInd;
                float diffX, diffY, diffZ, angle;

                for (uint16_t j = 0; j < SCAN_NUM; ++j) 
                {
                    for (uint16_t i = 0; i < groundScanInd; ++i) 
                    {
                        lowerInd = j + (i)*SCAN_NUM;
                        upperInd = j + (i + 1) * SCAN_NUM;

                        if (fullCloud->points[lowerInd].intensity == -1 ||
                            fullCloud->points[upperInd].intensity == -1) {
                            groundMat.at<int8_t>(i, j) = -1;
                            continue;
                        }

                        diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                        diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                        diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                        angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

                        if (abs(angle - sensorMountAngle) <= 10) {
                        groundMat.at<int8_t>(i, j) = 1;
                        groundMat.at<int8_t>(i + 1, j) = 1;
                        }
                    }
                    }
            }
    }; 
}
