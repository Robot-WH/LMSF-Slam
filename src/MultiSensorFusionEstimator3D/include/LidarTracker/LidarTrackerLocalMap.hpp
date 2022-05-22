/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-28 17:20:03
 * @Description: 
 * @Others: 
 */
#pragma once 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "LidarTrackerBase.hpp"
#include "factory/Map/LocalMap_factory.hpp"
#include "Algorithm/PointClouds/registration/registration_adapter.hpp"
#include "Algorithm/PointClouds/processing/process_base.hpp"
#include "tic_toc.h"
#include "Common/color.hpp"

namespace Slam3D {
    using Algorithm::RegistrationAdapterBase; 
    using Algorithm::RegistrationAdapterImpl;
    /**
     * @brief: tracker LoclMap匹配模式框架
     * @details:  适应多种Local Map构造方式 ， 以及多种匹配方式  a. 特征ICP  b. NDT/GICP
     * @param _PointType 使用的特征点的类型  
     * @param _RegistrationType 匹配算法的基类type 
     */    
    template<typename _PointType, typename _RegistrationType>
    class LidarTrackerLocalMap : public LidarTrackerBase<_PointType>
    {
        private:
            using PointCloudPtr = typename pcl::PointCloud<_PointType>::Ptr;  
            using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;  
            using LocalMapInput = std::pair<std::string, PointCloudConstPtr>;      // Local map 类型 
            using RegistrationAdapterPtr = std::unique_ptr<RegistrationAdapterBase< 
                LocalMapInput, FeaturePointCloudContainer<_PointType>>>;
            using LocalMapContainer = std::unordered_map<std::string, 
                std::unique_ptr<PointCloudLocalMapBase<_PointType>>>;  // <local map id, 对象指针>
            using LocalMapPointsContainer = std::unordered_map<std::string, PointCloudConstPtr>;
            enum LocalMapUpdataType
            {
                NO_UPDATA = 0,
                MOTION_UPDATA,
                TIME_UPDATA
            };
            bool init_;   
            bool local_map_full_ = false;  
            double THRESHOLD_TRANS_, THRESHOLD_ROT_;  
            double TIME_INTERVAL_;  
            // 匹配算法 
            // 输入源点云类型：LocalMapInput          输入目标点云类型：FeatureInfo<_PointType>
            RegistrationAdapterPtr registration_ptr_;  
            LocalMapContainer local_map_;  // 每一种特征都会维护一个local map 
            Eigen::Isometry3d prev_pose_;  // 上一帧的位姿
            Eigen::Isometry3d curr_pose_;   // 上一帧的位姿
            Eigen::Isometry3d predict_trans_;  // 预测位姿
            Eigen::Isometry3d motion_increment_;  // 运动增量 
            Eigen::Isometry3d last_keyframe_pose_;  // 上一个关键帧的位姿
            double last_keyframe_time_;  // 上一个关键帧的位姿

        public:
            LidarTrackerLocalMap() : init_(false), registration_ptr_(nullptr)
            ,THRESHOLD_TRANS_(0.3), THRESHOLD_ROT_(0.1), TIME_INTERVAL_(10)
            {}

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief:  设置Local map
             * @param feature_type 使用的每种特征的标识名
             * @param Local_map_type 局部地图的类型  - 滑动窗口/区域
             * @param param 对应的参数  
             * @return {*}
             */            
            template<typename... ParamType>
            void SetLocalMap(std::vector<std::string> const& feature_type,
                std::string const& Local_map_type, ParamType... param)
            {
                // 对每种特征都构建local map
                for (std::string const& type : feature_type)
                {
                    local_map_.insert(std::make_pair(type, 
                        make_localMap<_PointType>(Local_map_type, type, param...)));
                }
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 匹配算法设置  
             */            
            void SetRegistration(std::unique_ptr<_RegistrationType> registration_ptr)
            {
                registration_ptr_.reset(
                    new RegistrationAdapterImpl< LocalMapInput, FeaturePointCloudContainer<_PointType>, 
                                                                                    _RegistrationType>(std::move(registration_ptr)));  
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 激光tracker 求解 
             * @details:  求解出当前激光与上一帧激光数据的增量  
             * @param[in] data 特征点云数据
             * @param[in] timestamp 时间戳  
             * @param[out] deltaT 输出的增量
             */            
            virtual void Solve(FeaturePointCloudContainer<_PointType> const& data, 
                                                    double const& timestamp,
                                                    Eigen::Isometry3d &deltaT) override
            {
                // local map 初始化
                if (init_ == false)
                {   
                    updateLocalMap(data, Eigen::Isometry3d::Identity(), MOTION_UPDATA);
                    curr_pose_ = Eigen::Isometry3d::Identity();  
                    prev_pose_ = Eigen::Isometry3d::Identity();  
                    motion_increment_ = Eigen::Isometry3d::Identity();  
                    last_keyframe_pose_ = Eigen::Isometry3d::Identity();  
                    last_keyframe_time_ = timestamp;  
                    init_ = true;  
                    return;  
                }
                // 位姿预测
                // 判断是否有预测位姿
                if (deltaT.matrix() == Eigen::Isometry3d::Identity().matrix()) {
                    curr_pose_ = prev_pose_ * motion_increment_; // 采用匀速运动学模型预测
                } else {
                    curr_pose_ = prev_pose_ * deltaT;  
                }

                RegistrationLocalMap(data, curr_pose_);
                //std::cout<<"curr_pose_: "<<std::endl<<curr_pose_.matrix()<<std::endl;
                motion_increment_ = prev_pose_.inverse() * curr_pose_;    // 当前帧与上一帧的运动增量
                deltaT = motion_increment_;
                prev_pose_ = curr_pose_;  
                // 判定是否需要更新local map  
                LocalMapUpdataType updata_type = needUpdataLocalMap(curr_pose_, timestamp);  
                // 判定是否需要更新localmap
                if (updata_type)
                {
                    // std::cout<<common::RED<<"UPDATA!"<<common::RESET<<std::endl;
                    last_keyframe_pose_ = curr_pose_; // 更新关键帧Pose 
                    last_keyframe_time_ = timestamp;   
                    updateLocalMap(data, curr_pose_, updata_type); // 更新localmap  
                    //tt.toc("updateLocalMap: ");  
                    // 如果local map 满了  那么调整更新阈值
                    if (!local_map_full_)
                    {   // 如果local map 满了 那么就更新阈值  
                        if (local_map_.begin()->second->is_full())
                        {
                            //std::cout<<"LOCAL MAP FULL-------"<<std::endl;
                            // THRESHOLD_TRANS_ = 1;
                            // THRESHOLD_ROT_ = 0.5;   
                            local_map_full_ = true;
                        }
                    }
                    return;
                }
               // tt.toc("done: ");  
            }

            /**
             * @brief: 某一帧激光特征与Local map进行匹配   
             * @details:    
             * @param data 
             * @return 匹配是否成功 
             */            
            bool RegistrationLocalMap(FeaturePointCloudContainer<_PointType> const& data, 
                                                                        Eigen::Isometry3d &predict_pose) override
            {
                TicToc tt;  
                tt.tic();  
                // 匹配
                registration_ptr_->SetInputTarget(data); 
                registration_ptr_->Registration(predict_pose);       
                tt.toc("Registration ");  
            }

            /**
             * @brief: 获取在local map 坐标系的当前坐标 
             */            
            Eigen::Isometry3d const& GetCurrPoseInLocalFrame() const override
            {
                return curr_pose_;
            }

            LocalMapPointsContainer GetLocalMap() const override
            {
                LocalMapPointsContainer local_map_points;
                for (auto iter = local_map_.begin(); iter != local_map_.end(); ++iter) 
                {   
                    local_map_points.insert(iter->second->GetLocalMap()); 
                }
                return local_map_points;  
            }

        protected:
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 更新局部地图 
             * @param feature_map 用于更新local map 的点云数据  
             * @param T 相对于local map坐标系的变换矩阵
             * @param updata_type local map的更新方式 
             */            
            void updateLocalMap(FeaturePointCloudContainer<_PointType> const& feature_map, 
                Eigen::Isometry3d const& T, LocalMapUpdataType const& updata_type)
            {
                // 变换点云到odom坐标 
                PointCloudPtr transformed_cloud(new pcl::PointCloud<_PointType>());
                // 遍历全部特征的点云数据 pointcloud_data_ 
                for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter) 
                {
                    if (iter->second->empty()) continue;   // 判断数据是否为空
                    // 找到与特征名字相同的local map 
                    if (local_map_.find(iter->first) != local_map_.end())
                    {   // 更新地图
                        pcl::transformPointCloud (*(iter->second), *transformed_cloud, T.matrix());
                        if (updata_type == MOTION_UPDATA)
                        {   
                            //std::cout<<common::YELLOW<<"MOTION_UPDATA!"<<common::RESET<<std::endl;
                            local_map_[iter->first]->AddFrameForMotion(transformed_cloud);  
                        }
                        else if (updata_type == TIME_UPDATA)
                        {
                            //std::cout<<common::YELLOW<<"TIME_UPDATA!"<<common::RESET<<std::endl;
                            local_map_[iter->first]->AddFrameForTime(transformed_cloud);  
                        }
                        // 将更新后的地图设置为匹配的local map 
                        registration_ptr_->SetInputSource(local_map_[iter->first]->GetLocalMap());     
                    }
                }
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 检查是否需要更新局部地图
             * @details:  1、检查运动是否足够(local map 没满时，阈值低一些)    2、时间是否足够久 10s  
             */            
            LocalMapUpdataType needUpdataLocalMap(
                Eigen::Isometry3d const& curr_pose, double const& curr_time)
            {
                // 检查时间
                if (curr_time - last_keyframe_time_ > TIME_INTERVAL_) {
                    //std::cout<<common::GREEN<<"NEED TIME_UPDATA!"<<common::RESET<<std::endl;
                    return TIME_UPDATA;
                } 
                // 检查运动
                // 求出相对于上一个关键帧的变换
                Eigen::Isometry3d delta_transform = last_keyframe_pose_.inverse() * curr_pose;
                double delta_translation = delta_transform.translation().norm();         
                // 旋转矩阵对应 u*theta  对应四元数  e^u*theta/2  = [cos(theta/2), usin(theta/2)]
                Eigen::Quaterniond q_trans(delta_transform.rotation());
                q_trans.normalize();   
                double delta_angle = std::acos(q_trans.w())*2;     // 获得弧度    45度 约等于 0.8  
                // 满足关键帧条件
                if (delta_translation > THRESHOLD_TRANS_ || delta_angle > THRESHOLD_ROT_) 
                {
                    //std::cout<<common::GREEN<<"NEED MOTION_UPDATA!"<<common::RESET<<std::endl;
                    return MOTION_UPDATA;
                }
                return NO_UPDATA;  
            }
    }; // class LidarTrackerLocalMap
} // namespace 