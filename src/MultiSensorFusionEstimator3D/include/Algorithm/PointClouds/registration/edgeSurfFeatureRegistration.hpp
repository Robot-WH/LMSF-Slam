/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-03 19:35:39
 * @Description: 
 * @Others: 
 */
#pragma once 

#include "registration_base.hpp"
#include "ceres_factor/edge_factor.hpp"
#include "ceres_factor/surf_factor.hpp"
#include "Algorithm/Ceres/Parameterization/PoseSE3Parameterization.hpp"
#include "FeatureMatch/EdgeFeatureMatch.hpp"
#include "FeatureMatch/surfFeatureMatch.hpp"
#include "tic_toc.h"

namespace Algorithm
{
    using Slam3D::FeaturePointCloudContainer;  
    /**
     * @brief: 基于GN/LM法的边缘/面特征匹配  
     */    
    template<typename _PointType>
    class EdgeSurfFeatureRegistration : public RegistrationBase<_PointType>
    {
        public:
            enum OptimizeMethod
            {
                GN = 0,  // 高斯牛顿
                LM // LM法  
            } method_;
        private:
            using SurfCostFactorInfo = typename SurfFeatureMatch<_PointType>::SurfCostFactorInfo;
            using EdgeCostFactorInfo = typename EdgeFeatureMatch<_PointType>::EdgeCostFactorInfo;

            std::vector<Eigen::Vector3d> origin_surf_points_;
            std::vector<Eigen::Vector3d> origin_edge_points_;
            std::vector<SurfCostFactorInfo> surf_matched_info_;
            std::vector<EdgeCostFactorInfo> edge_matched_info_;

            using Base = RegistrationBase<_PointType>; 
            std::string edge_name_, surf_name_;   // 线、面特征的标识
            // 匹配器 
            EdgeFeatureMatch<_PointType> edge_match_;
            SurfFeatureMatch<_PointType> surf_match_;
            // target pointcloud 
            typename pcl::PointCloud<_PointType>::ConstPtr surf_point_in_;
            typename pcl::PointCloud<_PointType>::ConstPtr edge_point_in_;
            // 求解结果
            Eigen::Quaterniond q_w_l_;
            Eigen::Vector3d t_w_l_;
            Eigen::MatrixXd Map;     //  退化时，对于X的remapping 矩阵

            uint16_t optimization_count_; 
            uint16_t edge_num_ = 0;  
            uint16_t surf_num_ = 0;  

            bool isDegenerate = false; 

        public:
            EdgeSurfFeatureRegistration(std::string const& edge_name, std::string const& surf_name)
            : edge_name_(edge_name), surf_name_(surf_name), optimization_count_(10), method_(GN)
            {
            }

            /**
             * @brief: 传入source 点云
             * @details:  j将匹配源点云传入到特征匹配模块的
             * @param source_input  点云名称+点云数据 std::pair<std::string, PointCloudPtr> 
             */            
            void SetInputSource(typename Base::SourceInput const& source_input) override
            {
                // TicToc tt;
                // tt.tic(); 
                // 通过名字判断是 线特征地图还是 面特征地图 
                if (source_input.first == edge_name_)
                { 
                    edge_match_.SetSearchTarget(source_input.second);  
                }
                else if (source_input.first == surf_name_)
                {
                    surf_match_.SetSearchTarget(source_input.second);  
                }
                // tt.toc("kdtree "); 
            }

            void SetInputTarget(FeaturePointCloudContainer<_PointType> const& target_input) override
            {   
                // 直接使用点云数据 
                if (target_input.find(edge_name_) != target_input.end())
                {
                    edge_point_in_ = target_input.find(edge_name_)->second; 
                }
                if (target_input.find(surf_name_) != target_input.end())
                {
                    surf_point_in_ = target_input.find(surf_name_)->second; 
                }
            }

            void SetMaxIteration(uint16_t const& n)
            {
                optimization_count_ = n;  
            }

            /**
             * @brief: 求解匹配 
             * @param[out] T 输入匹配预测初值   返回匹配的结果
             * @return {*}
             */            
            void Solve(Eigen::Isometry3d &T) override
            {
                TicToc tt;  
                tt.tic();  
                // 将预测位姿设定为优化前的初始值    
                q_w_l_ = Eigen::Quaterniond(T.rotation());
                t_w_l_ = T.translation();
                // 准备空间
                origin_surf_points_.reserve(surf_point_in_->size());
                origin_edge_points_.reserve(edge_point_in_->size());
                surf_matched_info_.reserve(surf_point_in_->size());
                edge_matched_info_.reserve(edge_point_in_->size());  
                // 迭代
                int iterCount = 0;
                for (iterCount = 0; iterCount < optimization_count_; iterCount++)
                {
                    // 每次迭代清空特征点集合
                    origin_surf_points_.clear();
                    surf_matched_info_.clear();
                    origin_edge_points_.clear();
                    edge_matched_info_.clear();
                    // 为每个特征构建残差factor  
                    addSurfCostFactor();
                    addEdgeCostFactor();  
                    // scan-to-map优化
                    // 对匹配特征点计算Jacobian矩阵，观测值为特征点到直线、平面的距离，构建高斯牛顿方程，迭代优化当前位姿，存transformTobeMapped
                    //if (GNOptimization(iterCount) == true)
                    if (method_ == GN) {
                        if (GNOptimization(iterCount) == true) {
                            break;              
                        }
                    } else {
                        if (LMOptimization(iterCount)) {
                        }
                    }
                }

                T.linear() = q_w_l_.toRotationMatrix();
                T.translation() = t_w_l_;
                tt.toc("solve");  
                std::cout<<"iterCount: "<<iterCount<<std::endl;
            }

        protected:

            void addSurfCostFactor()
            {
                surf_num_ = 0;

                for (int i = 0; i < (int)surf_point_in_->points.size(); i++)
                {
                    _PointType point_temp;
                    pointAssociateToMap(&(surf_point_in_->points[i]), &point_temp);

                    typename SurfFeatureMatch<_PointType>::SurfCostFactorInfo res;

                    if (surf_match_.Match(point_temp, res))
                    {
                        Eigen::Vector3d ori_point(surf_point_in_->points[i].x, 
                                                        surf_point_in_->points[i].y, 
                                                        surf_point_in_->points[i].z);
                        origin_surf_points_.push_back(std::move(ori_point));  
                        surf_matched_info_.push_back(std::move(res));  
                        surf_num_++;
                    }
                }
                if (surf_num_<20) {
                    // std::cout<<common::YELLOW<<"not enough surf points, surf_num_: "
                    // <<surf_num_<<common::RESET<< std::endl;
                }
            }

            void addEdgeCostFactor()
            {
                edge_num_ = 0;
                //std::cout<<"addEdgeCostFactor, points.size(): "<<(int)edge_point_in_->points.size()<<std::endl;
                for (int i = 0; i < (int)edge_point_in_->points.size(); i++)
                {
                    _PointType point_in_ref;
                    pointAssociateToMap(&(edge_point_in_->points[i]), &point_in_ref);
                    typename EdgeFeatureMatch<_PointType>::EdgeCostFactorInfo res;

                    if (edge_match_.Match(point_in_ref, res))
                    {
                        Eigen::Vector3d ori_point(edge_point_in_->points[i].x, 
                                                                                edge_point_in_->points[i].y, 
                                                                                edge_point_in_->points[i].z);
                        origin_edge_points_.push_back(std::move(ori_point));  
                        edge_matched_info_.push_back(std::move(res));  

                        edge_num_++;
                    }
                }
                if(edge_num_<20){
                    // std::cout<<common::YELLOW<<"not enough edge points, edge_num_: "
                    // <<edge_num_<<common::RESET<< std::endl;
                }
            }

            /**
             * @brief: 四元数 + xyz 优化   
             * @details: 
             * @param {int} iterCount
             * @return {*}
             */    
            bool GNOptimization(int iterCount)
            {
                // 当前帧匹配特征点数太少
                if (edge_num_ + surf_num_ < 10) {
                    std::cout<<common::YELLOW<<"not enough feature, num: "
                    <<edge_num_ + surf_num_<<common::RESET<< std::endl;
                    return false;
                }

                Eigen::MatrixXd J(Eigen::MatrixXd::Zero(edge_num_ + surf_num_, 6));              // 残差关于状态的jacobian矩阵
                Eigen::MatrixXd JTJ(Eigen::MatrixXd::Zero(6, 6));     // 残差关于状态的jacobian矩阵
                Eigen::MatrixXd R(Eigen::MatrixXd::Zero(edge_num_ + surf_num_, 1));   
                Eigen::MatrixXd JTR(Eigen::MatrixXd::Zero(6, 1));   
                Eigen::MatrixXd X(Eigen::MatrixXd::Zero(6, 1));   

                Eigen::Vector3d grad, pointOri;  
                float residual = 0;  
                // 遍历匹配特征点，构建Jacobian矩阵
                for (int i = 0; i < edge_num_ + surf_num_; i++) 
                {
                    if (i < edge_num_)
                    {
                        // 激光系下点的坐标 
                        pointOri = origin_edge_points_[i];
                        // 残差以及其梯度向量  
                        grad = edge_matched_info_[i].norm_; 
                        residual = edge_matched_info_[i].residuals_;
                    }
                    else
                    {
                        // 激光系下点的坐标 
                        pointOri = origin_surf_points_[i - edge_num_];
                        // 残差以及其梯度向量  
                        grad = surf_matched_info_[i - edge_num_].norm_; 
                        residual = surf_matched_info_[i - edge_num_].residuals_;
                    }

                    Eigen::Matrix<double, 3, 6> d_P_T;
                    // 左乘扰动 
                    //d_P_T.block<3, 3>(0, 0) = -Math::GetSkewMatrix<double>(q_w_l_ * pointOri);    // 关于旋转
                    //  右乘扰动  
                    d_P_T.block<3, 3>(0, 0) = (-q_w_l_.toRotationMatrix() 
                                                                            * Math::GetSkewMatrix<double>(pointOri));
                    d_P_T.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();     // 关于平移  
                    Eigen::Matrix<double, 1, 3> d_r_P;
                    d_r_P = grad.transpose();  
                    J.block<1, 6>(i, 0) = d_r_P * d_P_T;  // lidar -> camera
                    R(i, 0) = residual; // 点到直线距离、平面距离，作为观测值
                }
                // TicToc tt;
                // tt.tic();
                // X = J.colPivHouseholderQr().solve(-R);       //方式1：采用QR分解   速度 ：ldl > QR > SVD ,精度 ldl < QR <= SVD
                JTJ = J.transpose() * J;
                JTR = J.transpose() * R;
                X = JTJ.colPivHouseholderQr().solve(-JTR);    
                // tt.toc("QR solve ");
                // JTJ = J.transpose() * J;
                // JTR = J.transpose() * R;
                // // J^T·J·delta_x = -J^T·f 高斯牛顿
                // X = JTJ.ldlt().solve(-JTR);    //  方式2：采用LDL
                // tt.toc("LDLT solve ");
                // // 首次迭代，检查近似Hessian矩阵（J^T·J）是否退化，或者称为奇异，行列式值=0 todo
                if (iterCount == 0) {
                    //  对 ATA进行特征分解 
                    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JTJ);      // cov 是SelfAdjoint
                    Eigen::MatrixXd V = eigensolver.eigenvectors();
                    Eigen::MatrixXd V2 = V;
                    
                    isDegenerate = false;
                    float degeneracy_thresh = 100;   // 特征值阈值
                    // 从最小特征
                    for (int i = 5; i >= 0; i--) 
                    {   // 将小于阈值的特征值对应的特征向量设为0  
                        if (eigensolver.eigenvalues()[i] < degeneracy_thresh) 
                        {
                            V2.row(i) = Eigen::MatrixXd::Zero(1, 6);
                            isDegenerate = true;
                            std::cout<<common::RED<<"isDegenerate !!!, eigensolver: "<<
                            eigensolver.eigenvalues()[i]<<std::endl;
                        } 
                        else 
                        {
                            break;
                        }
                    }
                    Map = V.inverse() * V2;  
                }

                if (isDegenerate)
                {
                    X = Map * X;
                }

                // 更新当前位姿 x = x + delta_x
                t_w_l_[0] += X(3, 0);
                t_w_l_[1] += X(4, 0);
                t_w_l_[2] += X(5, 0);
                // 转四元数增量 
                Eigen::Vector3d delta_rot = {X(0, 0), X(1, 0), X(2, 0)};
                Eigen::AngleAxisd delta_rot_v(delta_rot.norm() / 2, delta_rot.normalized());
                Eigen::Quaterniond delta_q(delta_rot_v);  
                 // 更新旋转 
                // q_w_l_ = delta_q * q_w_l_;   // 左乘扰动采用左乘进行更新
                q_w_l_ = q_w_l_ * delta_q;     // 右乘扰动采用右乘进行更新
                float deltaR = delta_rot.norm() / 2;  
                float deltaT = sqrt(pow(X(3, 0) * 100, 2) + pow(X(4, 0) * 100, 2) 
                                                        + pow(X(5, 0) * 100, 2));
                // delta_x很小，认为收敛   
                if (deltaR < 0.0009 && deltaT < 0.05) {   // 角度变化 < 0.05度  位置变化 < 0.05 cm 
                    return true; 
                }
                return false; 
            }

            /**
             * @brief: 使用LM法进行求解 
             * @details 四元数 + xyz 优化   
             * @param {int} iterCount
             * @return {*}
             */    
            bool LMOptimization(int iterCount)
            {
            }

            void pointAssociateToMap(_PointType const *const pi, _PointType *const po)
            {
                Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
                Eigen::Vector3d point_w = q_w_l_ * point_curr + t_w_l_;
                *po = *pi; 
                po->x = point_w.x();
                po->y = point_w.y();
                po->z = point_w.z();
            }

    }; // class LineSurfFeatureRegistration 


}

