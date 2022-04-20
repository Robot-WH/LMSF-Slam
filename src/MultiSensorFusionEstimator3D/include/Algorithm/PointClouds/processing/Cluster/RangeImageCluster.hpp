/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-09 21:23:23
 * @Description:  基于距离图像的聚类方法
 * @Others: 
 */
#pragma once
#include "RotaryLidar_preprocessing.hpp"

namespace Algorithm {

    /**
     * @brief: 基于距离图像的聚类方法
     * @details:  ref. legoloam ，只能用于机械旋转雷达 
     */    
    template<typename _PointT>
    class RangeImageCluster
    {
        private:
            RangeImageGroundDetect ground_detect_;
        public:
            
    }; 
}
