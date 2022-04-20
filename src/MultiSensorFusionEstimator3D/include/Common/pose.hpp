
/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 * 
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

namespace Slam3D {

    class Pose
    {
        public:
            Pose()
            {
                q_ = Eigen::Quaterniond::Identity();
                t_ = Eigen::Vector3d::Zero();
                T_ = Eigen::Isometry3d::Identity();
                td_ = 0;
                cov_.setZero();
            }

            Pose(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const double &td=0)
            {
                q_ = q; 
                q_.normalize();
                t_ = t;
                T_.setIdentity(); 
                T_.linear() = q_.toRotationMatrix(); 
                T_.translation() = t_;
                td_ = td;
                cov_.setZero();
            }

            Pose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const double &td=0)
            {
                q_ = Eigen::Quaterniond(R);
                t_ = t;
                T_.setIdentity(); 
                T_.linear() = R; 
                T_.translation() = t_;
                td_ = td;
                cov_.setZero();
            }

            Pose(const Eigen::Isometry3d &T, const double &td=0)
            {
                q_ = Eigen::Quaterniond(T.rotation()); 
                q_.normalize();
                t_ = T.translation();
                T_ = T;
                td_ = td;
                cov_.setZero();
            }

            // pose1 * pose2
            static Pose poseTransform(const Pose &pose1, const Pose &pose2)
            {
                return Pose(pose1.q_*pose2.q_, pose1.q_*pose2.t_+pose1.t_);
            }

            void SetIdentity()
            {
                q_ = Eigen::Quaterniond::Identity();
                t_ = Eigen::Vector3d::Zero();
                T_ = Eigen::Isometry3d::Identity();
                td_ = 0;
                cov_.setZero();
            }

            void update()
            {
                T_.linear() = q_.toRotationMatrix(); 
                T_.translation() = t_;
            }

            Pose inverse() const
            {
                return Pose(q_.inverse(), -(q_.inverse()*t_));
            }

            Pose operator * (const Pose &pose)
            {
                return Pose(q_*pose.q_, q_*pose.t_+t_);
            }

            Eigen::Matrix<double, 6, 1> se3() const
            {
                Sophus::SE3d SE3_qt(q_, t_);
                Eigen::Matrix<double, 6, 1> xi = SE3_qt.log();
                // Sophus::SE3::hat(xi)
                // Sophus::SE3::vee(Sophus::SE3::hat(xi)) == xi
                return xi;
            }

            Eigen::Quaterniond const& q() const
            {
                return q_; 
            }

            Eigen::Vector3d const& t() const
            {
                return t_; 
            }

        private:
            double td_;   // 时间偏移 
            Eigen::Quaterniond q_; // q = [cos(theta/2), u*sin(theta/2)]
            Eigen::Vector3d t_;
            Eigen::Isometry3d T_;
            Eigen::Matrix<double, 6, 6> cov_;  

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // TODO: the Eigen bugs in initializing the class
    }; // class Pose
} // namespace common
