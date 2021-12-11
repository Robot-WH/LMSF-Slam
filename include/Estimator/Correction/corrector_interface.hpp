/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-09 23:47:41
 * @Description: 
 * @Others: 
 */

#ifndef _CORRECTOR_INTERFACE_HPP_
#define _CORRECTOR_INTERFACE_HPP_

namespace Estimator {

template<typename __StatesType, typename  __ObsType, int __StatesDim>
class CorrectorInterface {
    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
        * @brief 校正 
        * @details
        * @param observation 观测量 
        * @param states 当前估计的状态 
        * @param timestamp 校正的时间戳 
        * @param V 观测残差协方差 
        */
        virtual void Correct( __ObsType const& observation, 
                                    __StatesType &states, 
                                    double const& timestamp, 
                                    Eigen::MatrixXd const& V) = 0;  

};


}

#endif 
