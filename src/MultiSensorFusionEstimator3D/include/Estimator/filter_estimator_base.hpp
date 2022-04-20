/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-21 18:40:49
 * @Description: 
 * @Others: 
 */

#ifndef _FILTER_ESTIMATOR_BASE_HPP_
#define _FILTER_ESTIMATOR_BASE_HPP_ 

#include "Sensor/sensor_process_interface.hpp"
#include "Estimator/states.hpp"

namespace Slam3D {

    template<typename _StateType, int _StateDim, typename... _DataTypes>
    class FilterEstimatorBase : public  SensorProcessInterface<_DataTypes...> {
        public:
            FilterEstimatorBase() 
            {
                estimated_state_.Reset();  
                cov_.setZero();   
            }
            virtual ~FilterEstimatorBase() {}
            virtual bool Initialize(double const& timestamp) = 0;  
            virtual bool IsInitialized() const {
                return initialize_done_;  
            }
            _StateType const& GetState() const {
                return estimated_state_; 
            }
        protected:
             _StateType estimated_state_;      // 状态
             Eigen::Matrix<double, _StateDim, _StateDim> cov_;  // 协方差矩阵 
            bool initialize_done_ = false;  
    }; // class FilterEstimatorBase 
}// namespace Estimator 
#endif 
