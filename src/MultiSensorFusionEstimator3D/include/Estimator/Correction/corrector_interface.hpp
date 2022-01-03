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

/**
 * @brief:  广义滤波器校正抽象类 
 * @details:  主要针对 如eskf, ieskf
 * @param _StatesType 估计的状态类型  
 * @param _ObsType 观测量类型 
 */
template<typename _StateType, typename  _ObsType, int _StateDim>
class CorrectorInterface {
    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
        * @brief ESKF校正流程  
        * @details
        * @param[in] observation 观测量 
        * @param[in] timestamp 校正的时间戳 
        * @param[in] V 观测残差协方差 
        * @param[out] states 当前估计的状态 
        * @param[out] cov 后验协方差矩阵 
        */
        virtual void Correct( _ObsType const& observation, 
                                                    double const& timestamp, 
                                                    Eigen::MatrixXd const& V,
                                                    _StateType &states,
                                                    Eigen::Matrix<double, _StateDim, _StateDim> &cov) = 0;  

}; // class CorrectorInterface 
}; // namespace Estimator

#endif 
