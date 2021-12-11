/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-08 19:29:41
 * @Description: 
 * @Others: 
 */

#ifndef _PREDICTOR_INTERFACE_HPP_
#define _PREDICTOR_INTERFACE_HPP_

namespace Estimator{

/**
 * @brief:  滤波器预测的抽象类  
 * @details: 
 * @param _StatesType 估计的状态类型
 * @param _DataType 接受的数据类型，可以有多种，例如IMU与轮速  
 */
template<typename _StatesType, typename... _DataType>
class PredictorInterface {};
// 特化 
template<typename _StatesType, typename _CurrentDataType, typename... _DataType>
class PredictorInterface<_StatesType, _CurrentDataType, _DataType...> : public  PredictorInterface<_StatesType, _DataType...>{
    public:

        virtual _CurrentDataType const& GetLastData() const = 0;      
        virtual void SetLastData(_CurrentDataType const& data) = 0;  
        virtual void PredictInitialize(_CurrentDataType const& data) = 0;
        virtual void Predict(_StatesType &states, _CurrentDataType const& curr_data) = 0;  
};

} // namespace Estimator 

#endif