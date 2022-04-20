/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-02 17:04:27
 * @Description: 点云线、面特征匹配
 * @Others: 
 */

#pragma once 

#include "registration_base.hpp"
#include "ceres_factor/edge_factor.hpp"
#include "ceres_factor/surf_factor.hpp"
#include "Algorithm/Ceres/Parameterization/PoseSE3Parameterization.hpp"
#include "tic_toc.h"
#include "FeatureMatch/EdgeFeatureMatch.hpp"
#include "FeatureMatch/surfFeatureMatch.hpp"

namespace Algorithm
{
    using Slam3D::FeatureInfo;  

    template<typename _PointType>
    class CeresEdgeSurfFeatureRegistration : public RegistrationBase<_PointType>
    {
        private:
            using Base = RegistrationBase<_PointType>; 
            std::string edge_name_, surf_name_;   // 线、面特征的标识
            // target pointcloud 
            typename pcl::PointCloud<_PointType>::ConstPtr surf_point_in_;
            typename pcl::PointCloud<_PointType>::ConstPtr edge_point_in_;
            // 匹配器 
            EdgeFeatureMatch<_PointType> edge_match_;
            SurfFeatureMatch<_PointType> surf_match_;

            double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
            Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
            Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

            uint16_t optimization_count_; 

        public:
            CeresEdgeSurfFeatureRegistration(std::string const& edge_name, std::string const& surf_name)
            : edge_name_(edge_name), surf_name_(surf_name), optimization_count_(10)
            , surf_point_in_(nullptr), edge_point_in_(nullptr)
            {
            }

            /**
             * @brief: 传入source 点云
             * @details:  匹配逻辑是 target 与 source 匹配 
             * @param source_input  点云名称+点云数据 std::pair<std::string, PointCloudPtr> 
             */            
            void SetInputSource(typename Base::SourceInput const& source_input) override
            {
                // TicToc tt;
                // tt.tic(); 
                if (source_input.second->empty()) return;  
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

            void SetInputTarget(FeatureInfo<_PointType> const& target_input) override
            {   
                // 直接使用点云数据 
                if (target_input.pointcloud_data_.find(edge_name_) 
                        != target_input.pointcloud_data_.end())
                {
                    edge_point_in_ = target_input.pointcloud_data_.find(edge_name_)->second; 
                }
                if (target_input.pointcloud_data_.find(surf_name_) 
                        != target_input.pointcloud_data_.end())
                {
                    surf_point_in_ = target_input.pointcloud_data_.find(surf_name_)->second; 
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
                if (optimization_count_ > 2) 
                    optimization_count_ --;   
                // 将预测位姿设定为优化前的初始值    
                q_w_curr = Eigen::Quaterniond(T.rotation());
                t_w_curr = T.translation();
                for (int iterCount = 0; iterCount < optimization_count_; iterCount++)
                {
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::Problem::Options problem_options;
                    ceres::Problem problem(problem_options);

                    problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
                    
                    addEdgeCostFactor(problem,loss_function);
                    addSurfCostFactor(problem,loss_function);

                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;
                    options.check_gradients = false;
                    options.gradient_check_relative_precision = 1e-4;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    //tt.toc("Solve");
                }
                // 判断退化
                T = Eigen::Isometry3d::Identity();
                T.linear() = q_w_curr.toRotationMatrix();
                T.translation() = t_w_curr;
            }

        protected:

            void addSurfCostFactor(ceres::Problem& problem, ceres::LossFunction *loss_function)
            {
                if (surf_point_in_ == nullptr) return;  
                int surf_num=0;
                //std::cout<<"addSurfCostFactor, points.size(): "<<(int)surf_point_in_->points.size()<<std::endl;
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
                        ceres::CostFunction *cost_function = new se3PointSurfFactor(ori_point, res.norm_, res.D_);    
                        problem.AddResidualBlock(cost_function, loss_function, parameters);
                        surf_num++;
                    }
                    // std::vector<int> pointSearchInd;
                    // std::vector<float> pointSearchSqDis;
                    // kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

                    // Eigen::Matrix<double, 5, 3> matA0;
                    // Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                    // // 最远的那个点距离应该小于一个阈值  
                    // if (pointSearchSqDis[4] < 1.0)
                    // {        
                    //     for (int j = 0; j < 5; j++)
                    //     {
                    //         matA0(j, 0) = surf_map_->points[pointSearchInd[j]].x;
                    //         matA0(j, 1) = surf_map_->points[pointSearchInd[j]].y;
                    //         matA0(j, 2) = surf_map_->points[pointSearchInd[j]].z;
                    //     }
                    //     // find the norm of plane
                    //     Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);    // 法向量 n ( 未归一化)
                    //     double negative_OA_dot_norm = 1 / norm.norm();   // b  
                    //     norm.normalize(); // 归一化法向量 
                    //     // 判断该平面质量   
                    //     bool planeValid = true;
                    //     for (int j = 0; j < 5; j++)
                    //     {
                    //         // if OX * n > 0.2, then plane is not fit well
                    //         if (fabs(norm(0) * surf_map_->points[pointSearchInd[j]].x +
                    //                 norm(1) * surf_map_->points[pointSearchInd[j]].y +
                    //                 norm(2) * surf_map_->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                    //         {
                    //             planeValid = false;
                    //             break;
                    //         }
                    //     }
                    //     Eigen::Vector3d curr_point(surf_point_in_->points[i].x, 
                    //                                                             surf_point_in_->points[i].y, 
                    //                                                             surf_point_in_->points[i].z);
                    //     // 质量好  则进行优化  
                    //     if (planeValid)
                    //     {
                    //         ceres::CostFunction *cost_function = new se3PointSurfFactor(curr_point, norm, negative_OA_dot_norm);    
                    //         problem.AddResidualBlock(cost_function, loss_function, parameters);
                    //         surf_num++;
                    //     }
                    // }
                }
                if (surf_num<20) {
                    printf("not enough surf points");
                }
            }

            void addEdgeCostFactor(ceres::Problem& problem, 
                                                                ceres::LossFunction *loss_function)
            {
                if (edge_point_in_ == nullptr) return;  
                int edge_num=0;
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
                        // 传入变换前的点坐标以及匹配到的edge的两个点 
                        ceres::CostFunction *cost_function = new se3PointEdgeFactor(ori_point, res.points_set_[0], 
                                                                                                                                                            res.points_set_[1]);    
                        problem.AddResidualBlock(cost_function, loss_function, parameters);
                        edge_num++;
                    }

                    // std::vector<int> pointSearchInd;
                    // std::vector<float> pointSearchSqDis;
                    // kdtreeEdgeMap->nearestKSearch(point_in_ref, 5, pointSearchInd, pointSearchSqDis); 

                    // if (pointSearchSqDis[4] < 1.0)
                    // {
                    //     std::vector<Eigen::Vector3d> nearCorners;
                    //     Eigen::Vector3d center(0, 0, 0);
                    //     for (int j = 0; j < 5; j++)
                    //     {
                    //         Eigen::Vector3d tmp(edge_map_->points[pointSearchInd[j]].x,
                    //                             edge_map_->points[pointSearchInd[j]].y,
                    //                             edge_map_->points[pointSearchInd[j]].z);
                    //         center = center + tmp;
                    //         nearCorners.push_back(tmp);
                    //     }
                    //     center = center / 5.0;
                    //     // 计算协方差矩阵
                    //     Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                    //     for (int j = 0; j < 5; j++)
                    //     {
                    //         Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    //         covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                    //     }
                    //     // PCA分解 
                    //     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                    //     Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                    //     Eigen::Vector3d curr_point(edge_point_in_->points[i].x, 
                    //                                                             edge_point_in_->points[i].y, 
                    //                                                             edge_point_in_->points[i].z);
                    //     // 要足够像一条线   
                    //     if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    //     { 
                    //         Eigen::Vector3d point_on_line = center;
                    //         Eigen::Vector3d point_a, point_b;
                    //         point_a = 0.1 * unit_direction + point_on_line;
                    //         point_b = -0.1 * unit_direction + point_on_line;

                    //         ceres::CostFunction *cost_function = new se3PointEdgeFactor(curr_point, point_a, point_b);  
                    //         problem.AddResidualBlock(cost_function, loss_function, parameters);
                    //         edge_num++;   
                    //     }                           
                    // }
                }
                if(edge_num<20){
                    printf("not enough edge points");
                }
            }

            void pointAssociateToMap(_PointType const *const pi, _PointType *const po)
            {
                Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
                Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
                *po = *pi; 
                po->x = point_w.x();
                po->y = point_w.y();
                po->z = point_w.z();
                //po->intensity = 1.0;
            }
    }; // class LineSurfFeatureRegistration 


}
