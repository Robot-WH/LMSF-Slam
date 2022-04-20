/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-04 17:28:25
 * @Description:  ceres  Pose 参数化
 * @Others: 
 */
#pragma once
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "Math.hpp"

namespace Algorithm
{
    class PoseSE3Parameterization : public ceres::LocalParameterization 
    {
        public:
            
            PoseSE3Parameterization() {}
            virtual ~PoseSE3Parameterization() {}

            /**
             * @brief: 
             * @details: 
             * @param x Global states 这里是李群形式  四元数 + XYZ 
             * @param delta 即Local status 即参与优化时的状态 ，为李代数形式  欧拉角 + XYZ 
             * @param {double*} x_plus_delta
             * @return {*}
             */            
            virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const
            {
                Eigen::Map<const Eigen::Vector3d> trans(x + 4);
                Eigen::Map<const Eigen::Quaterniond> quater(x);
                Eigen::Quaterniond delta_q;
                Eigen::Vector3d delta_t;
                // se3 转 平移和旋转SO3  
                Math::GetTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), 
                                                                    delta_q, delta_t);
                Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
                Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);
                quater_plus = delta_q * quater;
                trans_plus = delta_q * trans + delta_t;
                return true;
            }
            /**
             * @brief: SE3关于se3 的jacobian 
             * @details: 因为 ceres只能定义 error 关于 李群的jacobian 所以要通过这个
             *                      将error关于李群的jacobian转换为关于李代数的jacobian 
             * @param {double*} x
             * @param {double*} jacobian
             */            
            virtual bool ComputeJacobian(const double* x, double* jacobian) const
            {
                Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
                (j.topRows(6)).setIdentity();
                (j.bottomRows(1)).setZero();
                return true;
            }

            virtual int GlobalSize() const { return 7; }     // 参数化 李群的维度
            virtual int LocalSize() const { return 6; }       // 参数化李代数的维度   
    }; // class 
} // namespace 
