/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-08 22:08:16
 * @Description: 
 * @Others: 
 */

namespace Estimator{ 

/**
 * @brief:  
 * @details: 
 * @param _StateType 估计的状态
 * @param _PredictorType 预测器类型  , 预测器只有一种 ，IMU/WHEEL/IMU&WHEEL  
 * @param _CorrectorTypes 观测器类型包， 观测器可以有很多，如GNSS、激光等 
 */
template<typename _StateType, typename _PredictorType, typename... _CorrectorTypes>
class FilterEstimator {

    public:

        FilterEstimator(PredictorInterface predictor, _CorrectorTypes... correctors) : corrector_(correctors...) {

        }

        template<typename _DataType>
        void Predict(_DataType data) {

        }

        template<typename _DataType>
        void Correct(_DataType data) {

        }

    private:

        Corrector<_CorrectorTypes...> corrector_;      // 校正器  
        _PredictorType predictor_;      
};

};  