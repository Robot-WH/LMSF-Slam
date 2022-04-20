/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-03 23:50:59
 * @Description: 
 * @Others: 
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

class QuaternionXYZPoseLocalParameterization : public ceres::LocalParameterization 
{
public:
    QuaternionXYZPoseLocalParameterization() {}
    virtual ~QuaternionXYZPoseLocalParameterization() {}

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief:  状态更新
     * @param[in] x Global states 这里是李群SE3形式  四元数 + XYZ 
     * @param[in] delta 即Local status 即参与优化时的状态 ，为李代数se3形式  欧拉角 + XYZ 
     * @param[out] x_plus_delta 更新后的Global states 输出 
     */         
    virtual bool Plus(const double *x,  const double *delta, double *x_plus_delta) const 
    {
        Eigen::Map<const Eigen::Vector3d> _p(x+4);
        Eigen::Map<const Eigen::Quaterniond> _q(x);   // 传入序列 (x, y, z, w)构造eigen 四元数   注意eigen 内部存储四元数的顺序是  (x, y, z, w) 
        Eigen::Map<const Eigen::Vector3d> delta_theta(delta);   
        Eigen::Map<const Eigen::Vector3d> delta_t(delta+3);     
        // 旋转向量规范化 
        Eigen::Vector3d unit_delta_theta = delta_theta.normalized(); 
        double theta = delta_theta.norm();               
        //  so3 -> SO3
        Eigen::Quaterniond delta_q;  
        if(theta<1e-10)
        {
            delta_q = Eigen::Quaterniond(1, delta_theta.x() / 2, delta_theta.y() / 2, delta_theta.z() / 2);
        }
        else
        {
            double sin_half_theta = sin(0.5*theta);
            delta_q = Eigen::Quaterniond(cos(0.5*theta), unit_delta_theta.x()*sin_half_theta, 
                                                                                                            unit_delta_theta.y()*sin_half_theta, 
                                                                                                            unit_delta_theta.z()*sin_half_theta);
        }

        Eigen::Map<Eigen::Vector3d> p(x_plus_delta+4);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);
        q = (_q*delta_q).normalized();
        p = _p + delta_t; 
        return true;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual bool ComputeJacobian(const double *x, double *jacobian) const 
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        (j.topRows(6)).setIdentity();
        (j.bottomRows(1)).setZero();

        return true;
    }

    virtual int GlobalSize() const { return 7;};
    virtual int LocalSize() const { return 6; };
};