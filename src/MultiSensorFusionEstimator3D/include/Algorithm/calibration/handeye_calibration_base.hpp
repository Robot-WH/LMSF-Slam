/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-16 16:41:48
 * @Description: 
 * @Others: 
 */

#ifndef _HANDEYE_CALIBRATION_BASE_HPP_
#define _HANDEYE_CALIBRATION_BASE_HPP_

#include <eigen3/Eigen/Dense>
#include "Common/pose.hpp"
#include "Math.hpp"
#include "Common/color.hpp"

namespace Algorithm
{
    using Slam3D::Pose;  
    // 用于优先队列 将旋转小的元素放到前面 
    // 大顶堆   w 越大  theta 越小  
    struct rotCmp
    {
        bool operator()(const std::pair<uint16_t, std::pair<Pose, Pose>> &pose_r, 
                                        const std::pair<uint16_t, std::pair<Pose, Pose>> &pose_l)
        {   // 大顶堆 
            return (pose_l.second.first.q().w() > pose_r.second.first.q().w());    // w = cos(theta/2)    
        }
    };
    /**
     * @brief: 手眼标定的实现基础实现
     * @details: 
     */    
    class HandEyeCalibrationBase
    {
        private:
            const double EPSILON_R = 0.05;
            const double EPSILON_T = 0.1;
            const size_t N_POSE = 300;              // 最多包含的pose个数   
            double rot_cov_thre_;
            // 对pose 按照质量进行排序  
            std::priority_queue<std::pair<uint16_t, std::pair<Pose, Pose>>, 
			std::vector<std::pair<uint16_t, std::pair<Pose, Pose>>>, rotCmp> priority_pose_;
            // pose 数据库中的序号  
            std::queue<std::pair<uint16_t, std::pair<Pose, Pose>>> new_pose_pair_;    
            std::vector<std::pair<Pose, Pose>> pose_storage_;   

            Eigen::MatrixXd Q_;    // 计算旋转外参时的矩阵  
            Eigen::Quaterniond ext_q_result_;  
            Eigen::Vector3d ext_t_result_;  
            bool calib_done_ = false; 
            Pose primary_lidar_accum_pose_, sub_lidar_accum_pose_;

        public:  
            HandEyeCalibrationBase()
            {
                Q_ = Eigen::MatrixXd::Zero(N_POSE * 4, 4);
                pose_storage_.reserve(N_POSE);  
                rot_cov_thre_ = 0.25;  
            }
            // 添加一组pose数据     pose_laser 中装的是每个传感器的测量值  
            /**
             * @brief 
             * @param pose_primary 主传感器的pose
             * @param pose_sub 辅传感器pose 
             * @brief 添加一组多激光雷达帧间运动数据  
             * @return 本组运动是否合格   
             */
            bool AddPose(const  Pose& pose_primary, const  Pose& pose_sub)
            {        
                // 首先检查pose 
                bool check_pose = false;  
                if (!checkScrewMotion(pose_primary, pose_sub)) {
                    // std::cout<<"ScrewMotion error"<<std::endl;
                    check_pose = false;  
                    return check_pose;
                }

                if (pose_storage_.size() < N_POSE) 
                {
                    new_pose_pair_.emplace(pose_storage_.size(), 
                        make_pair(primary_lidar_accum_pose_, sub_lidar_accum_pose_));
                    priority_pose_.emplace(pose_storage_.size(), 
                        make_pair(primary_lidar_accum_pose_, sub_lidar_accum_pose_));  
                    pose_storage_.emplace_back(primary_lidar_accum_pose_, sub_lidar_accum_pose_); 
                } 
                else
                {   // 数据量大于300    则滑动窗口 
                    // maintain a min heap
                    uint16_t pos = priority_pose_.top().first;  
                    std::cout<<"num > 300, remove top w: "<< priority_pose_.top().second.first.q().w()<<std::endl;
                    pose_storage_[pos] = make_pair(primary_lidar_accum_pose_, sub_lidar_accum_pose_);  
                    new_pose_pair_.emplace(pos, make_pair(primary_lidar_accum_pose_, sub_lidar_accum_pose_));
                    priority_pose_.pop();   
                    priority_pose_.emplace(pos, make_pair(primary_lidar_accum_pose_, sub_lidar_accum_pose_));
                }   

                primary_lidar_accum_pose_.SetIdentity();
                sub_lidar_accum_pose_.SetIdentity();  
                std::cout<<"pose_storage_.size(): "<< pose_storage_.size()<<std::endl;
                if (pose_storage_.size() >= 3)
                    check_pose = true;   
                return check_pose;  
            }

            /**
             * @brief: 标定旋转 
             * @param[out] ext_q 标定出的旋转外参  
             * @return 是否成功
             */            
            bool CalibExRotation()
            {
                while (!new_pose_pair_.empty())
                {
                    auto pose_pair = new_pose_pair_.front();  
                    new_pose_pair_.pop();
                    
                    Eigen::Quaterniond const& primary_q = pose_pair.second.first.q();    // 主传感器的旋转
                    Eigen::Quaterniond const& second_q = pose_pair.second.second.q();  // 辅传感器的旋转  
                    uint16_t const& indice = pose_pair.first;  
                    // 求rubust核函数
                    // Eigen::Quaterniond r1 = primary_q;
                    // Eigen::Quaterniond r2 = calib_ext_[idx_data].q_ * pose_sub.q_ * calib_ext_[idx_data].inverse().q_;
                    // double angular_distance = 180 / M_PI * r1.angularDistance(r2); // calculate delta_theta=|theta_1-theta_2|
                    // double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;   // the derivative of huber norm
                    Eigen::Matrix4d L = Math::QuanternionLeftProductMatrix(primary_q);  
                    Eigen::Matrix4d R = Math::QuanternionRightProductMatrix(second_q);  
                  
                    Q_.block<4, 4>(indice * 4, 0) = (L - R);
                }
                // SVD 求解 AX = 0 
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q_, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3); // [w, x, y, z]     // 最小奇异值对应的特征向量为解
                if (x[0] < 0) x = -x; // use the standard quaternion
                Eigen::Vector4d rot_cov = svd.singularValues();    // singular value
                std::cout<<"rot_cov: "<<rot_cov.transpose()<<std::endl;
                // 若没有退化  AQ = 0 零空间为1 ， 则A的秩 = 4 -1 = 3， 那么ATA秩也为3， 因此A的奇异值只有一个为0
                //  因此 检查第二小奇异值， 看是否足够大，越大，解越稳定
                if (rot_cov[2] > rot_cov_thre_)
                {
                    ext_q_result_ = Eigen::Quaterniond(x[0], x[1], x[2], x[3]);
                    ext_q_result_.normalize(); 
                    return true; 
                }
                return false;  
            }

            bool CalibExTranslation()
            {
                if (calibExTranslationNonPlanar())
                {
                    calib_done_ = true;
                    return true;
                }
                return false;  
            }

            bool calibExTranslationNonPlanar()
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(pose_storage_.size() * 3, 3);
                Eigen::MatrixXd b = Eigen::MatrixXd::Zero(pose_storage_.size() * 3, 1);
                for (size_t i = 0; i < pose_storage_.size(); i++)
                {
                    const Pose &pose_primary = pose_storage_[i].first;
                    const Pose &pose_sub = pose_storage_[i].second;
                    // AngleAxisd ang_axis_ref(pose_primary.q_);
                    // AngleAxisd ang_axis_data(pose_sub.q_);
                    // // 计算指向旋转轴方向的平移差值 
                    // double t_dis = abs(pose_primary.t_.dot(ang_axis_ref.axis()) - pose_sub.t_.dot(ang_axis_data.axis()));
                    // double huber = t_dis > 0.04 ? 0.04 / t_dis : 1.0;
                    A.block<3, 3>(i * 3, 0) = (pose_primary.q().toRotationMatrix() - Eigen::Matrix3d::Identity());
                    b.block<3, 1>(i * 3, 0) = ext_q_result_ * pose_sub.t() - pose_primary.t();
                }
                Eigen::Vector3d x; 
                /**
                 *  TODO: 看看和QR分解的差别 
                 */
                x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
                ext_t_result_ = x;

                return true;
            }


            bool GetCalibResult(Eigen::Isometry3d &result)
            {
                if (calib_done_)
                {
                    result.linear() = ext_q_result_.toRotationMatrix();
                    result.translation() = ext_t_result_;  
                    return true;
                }
                return false;  
            }


        protected:
            /**
             * @brief: 检查运动是否符合条件 
             * @details: 
             * @param {Isometry3d} &pose_primary
             * @param {Isometry3d} &pose_sub
             * @return {*}
             */            
            bool checkScrewMotion(const Pose &pose_primary, const Pose &pose_sub)
            {
                // 先检查当前数据是否正确
                // 提取出数据的旋转部分  
                Eigen::AngleAxisd ang_axis_pri(pose_primary.q());
                Eigen::AngleAxisd ang_axis_sub(pose_sub.q());
                //  检查旋转  
                //  刚性连接的传感器   旋转角度应该相同
                double r_dis = abs(ang_axis_pri.angle() - ang_axis_sub.angle());   
                // 检查平移 
                // 沿着旋转方向上的平移   ？？？？？？？？？？？？？？？？？？？？？？？？？
                double t_dis = abs(pose_primary.t().dot(ang_axis_pri.axis()) 
                                                        - pose_sub.t().dot(ang_axis_sub.axis()));
                if ((r_dis > EPSILON_R) || (t_dis > EPSILON_T))
                {
                    if ((r_dis > EPSILON_R)) {
                        std::cout<<common::RED<<"r_dis > EPSILON_R !!"<<common::RESET<<std::endl;
                    }
                    if (t_dis > EPSILON_T) {
                        std::cout<<common::RED<<"t_dis > EPSILON_T!!"<<std::endl;
                    }
                    primary_lidar_accum_pose_.SetIdentity();
                    sub_lidar_accum_pose_.SetIdentity();  
                    return false;
                }
                // 如果旋转不够的话   则进行累计直到旋转足够  
                primary_lidar_accum_pose_ = primary_lidar_accum_pose_ * pose_primary;
                sub_lidar_accum_pose_ = sub_lidar_accum_pose_ * pose_sub; 
                Eigen::AngleAxisd ang_axis_accum_pri(primary_lidar_accum_pose_.q());
                Eigen::AngleAxisd ang_axis_accum_sub(sub_lidar_accum_pose_.q());

                if (ang_axis_accum_pri.angle() > 0 || ang_axis_accum_sub.angle() > 0)
                {
                    return true;  
                }
            }

    };


}

#endif