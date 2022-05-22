/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-09 21:06:13
 * @Description: 
 * @Others: 
 */


#pragma once 

#include "Estimator/estimator/MultiLidar/MultiLidar_estimator_base.hpp"

namespace Slam3D {

    template<typename _InputPointType, typename _FeaturePointType>
    class MultiLidarEstimatorFactoryInterface
    {
        public:
            using MultiLidarEstimatorPtr = 
                std::unique_ptr<MultiLidarEstimatorBase<_InputPointType, _FeaturePointType>>; 
            virtual ~MultiLidarEstimatorFactoryInterface() {}
            virtual MultiLidarEstimatorPtr Create(std::string config_path) = 0;
        private:
            virtual bool LoadParameter(std::string config_path) {};
    };
}


