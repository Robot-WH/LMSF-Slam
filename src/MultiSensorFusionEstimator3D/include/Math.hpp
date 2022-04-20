/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-11 12:46:36
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <eigen3/Eigen/Dense>

namespace Math {

    // 获取反对称矩阵 
    template<typename T>
    static Eigen::Matrix<T, 3, 3> GetSkewMatrix(const Eigen::Matrix<T, 3, 1>& v) 
    {
        Eigen::Matrix<T, 3, 3> w;
        w <<  0.,   -v(2, 0),  v(1, 0),
            v(2, 0),  0.,   -v(0, 0),
            -v(1, 0),  v(0, 0),  0.;
        return w;
    }

    // 李代数转四元数 + XYZ 
    static void GetTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, 
                                                                            Eigen::Quaterniond& q, Eigen::Vector3d& t)
    {
        Eigen::Vector3d omega(se3.data());
        Eigen::Vector3d upsilon(se3.data()+3);
        Eigen::Matrix3d Omega = GetSkewMatrix(omega);

        double theta = omega.norm();
        double half_theta = 0.5 * theta;

        double imag_factor;
        double real_factor = cos(half_theta);       // 四元数实部 

        if (theta < 1e-10)
        {  // sin( theta / 2)泰勒展开  
            double theta_sq = theta * theta;
            double theta_po4 = theta_sq * theta_sq;
            imag_factor = 0.5-0.0208333 * theta_sq+0.000260417 * theta_po4;       // 同时除了 theta
        }
        else
        {
            double sin_half_theta = sin(half_theta);
            imag_factor = sin_half_theta / theta;
        }

        q = Eigen::Quaterniond(real_factor, 
                                                        imag_factor * omega.x(), 
                                                        imag_factor * omega.y(), 
                                                        imag_factor * omega.z());

        Eigen::Matrix3d J;
        if (theta < 1e-10)
        {
            J = q.matrix();
        }
        else
        {
            //  罗德里格斯 
            Eigen::Matrix3d Omega2 = Omega * Omega;
            J = (Eigen::Matrix3d::Identity() 
                    + (1 - cos(theta)) / (theta * theta) * Omega + (theta - sin(theta)) / (pow(theta,3)) * Omega2);
        }
        t = J*upsilon;
    }

    /**
     * @brief:  四元数 左乘矩阵
     * @param {Quaterniond const&} q
     * @return {*}
     */
    static Eigen::Matrix4d QuanternionLeftProductMatrix(Eigen::Quaterniond const& q) {
        Eigen::Matrix4d m;
        m << q.w(), -q.x(), -q.y(), -q.z(),
                    q.x(), q.w(), -q.z(), q.y(),
                    q.y(), q.z(), q.w(), -q.x(),
                    q.z(), -q.y(), q.x(), q.w();  
        return m;  
    }

    static Eigen::Matrix4d QuanternionRightProductMatrix(Eigen::Quaterniond const& q) {
        Eigen::Matrix4d m;
        m << q.w(), -q.x(), -q.y(), -q.z(),
                    q.x(), q.w(), q.z(), -q.y(),
                    q.y(), -q.z(), q.w(), q.x(),
                    q.z(), q.y(), -q.x(), q.w();  
        return m;  
    }

    // static void HouseHolderQR()
    // {

    // }

} // namespace Math
