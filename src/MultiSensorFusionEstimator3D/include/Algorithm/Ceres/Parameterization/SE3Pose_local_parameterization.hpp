/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-04 17:49:14
 * @Description: 
 * @Others: 
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

class SE3PoseLocalParameterization : public ceres::LocalParameterization {
public:
    SE3PoseLocalParameterization() {}
    virtual ~SE3PoseLocalParameterization() {}

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
        Eigen::Map<const Eigen::Quaterniond> _q(x);  // 传入序列 (x, y, z, w)构造eigen 四元数   注意eigen 内部存储四元数的顺序是  (x, y, z, w) 
        
        Eigen::Map<const Eigen::Vector3d> delta_se3_omega(delta);    // se3中旋转部分   - so3 
        Eigen::Map<const Eigen::Vector3d> delta_se3_upsilon(delta+3);    // se3中平移部分       
        // so3 旋转规范化 
        Eigen::Vector3d unit_delta_se3_omega = delta_se3_omega.normalized(); 
        double theta = delta_se3_omega.norm();                            
        // so3 -> SO3 
        Eigen::Quaterniond delta_q;  
        if(theta<1e-10)
        {
            delta_q = Eigen::Quaterniond(1, delta_se3_omega.x() / 2, 
                                                                                delta_se3_omega.y() / 2, 
                                                                                delta_se3_omega.z() / 2);
        }
        else
        {
            double sin_half_theta = sin(0.5*theta);
            delta_q = Eigen::Quaterniond(cos(0.5*theta), unit_delta_se3_omega.x()*sin_half_theta, 
                                                                                                            unit_delta_se3_omega.y()*sin_half_theta, 
                                                                                                            unit_delta_se3_omega.z()*sin_half_theta);
        }
        Eigen::Matrix3d J;
        if (theta<1e-10)
        {
            J = delta_q.matrix();   // 当theta很小时， 近似为罗德里格斯公式 
        }
        else
        {
            double c = sin(theta) / theta;
            J = Eigen::Matrix3d::Identity()*c 
                    + (1 - c)*unit_delta_se3_omega*unit_delta_se3_omega.transpose() 
                    + (1 - cos(theta))*Utility::getSkewMatrix(unit_delta_se3_omega) / theta ; 
        }
        Eigen::Vector3d delta_t;  
        delta_t = J*delta_se3_upsilon;  
        Eigen::Map<Eigen::Vector3d> p(x_plus_delta+4);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);
        q = (delta_q * _q).normalized();
        p = delta_q * _p + delta_t; 
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
}; // class 

