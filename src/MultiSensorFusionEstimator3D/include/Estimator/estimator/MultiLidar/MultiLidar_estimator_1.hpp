/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-22 01:23:01
 * @Description:  多激光SLAM估计器  
 * @Others: 
 */
#ifndef _MULTILIDAR_ESTIMATOR_HPP_
#define _MULTILIDAR_ESTIMATOR_HPP_

#include "Estimator/estimator/MultiLidar/MultiLidar_estimator_base.hpp"
#include "Algorithm/PointClouds/processing/FeatureExtract/LOAMFeatureProcessor_base.hpp"
#include "Algorithm/calibration/handeye_calibration_base.hpp"
#include "LidarTracker/LidarTrackerLocalMap.hpp"
#include "Common/pose.hpp"
#include "Common/color.hpp"
#include "Common/keyframe_updater.hpp"
#include "BackEnd/backend.hpp"
#include <omp.h>  

namespace Slam3D{

    using Algorithm::HandEyeCalibrationBase;
    using Algorithm::PointCloudProcessBase;
    using common::DataManager;

    /**
     * @brief:  多激光融合估计器
     * @param _PointT 输入的点的类型
     * @param _FeatureT 点云处理后得到的特征类型 
     */    
    template<typename _PointT, typename _FeatureT>
    class MultiLidarEstimator : public MultiLidarEstimatorBase<_PointT, _FeatureT>
    {
        public:
            using Base = MultiLidarEstimatorBase<_PointT, _FeatureT>; 
            using FeatureProcessorPtr = std::unique_ptr<PointCloudProcessBase<_PointT, _FeatureT>>;  
            using LidarTrackerPtr = std::unique_ptr<LidarTrackerBase<_FeatureT>>;  
        private:
            std::vector<FeatureProcessorPtr> processor_;   // 每个激光一个点云处理器  
            std::vector<LidarTrackerPtr> lidar_tracker_;   
            HandEyeCalibrationBase extrinsics_init_;    // 外参初始化
            KeyframeUpdater keyframe_updater_; 
            std::thread estimate_thread_;
            // 提取的特征数据       <时间戳，vector(每一个雷达的特征数据)>  
            std::queue<std::vector<CloudContainer<_FeatureT>>> feature_buf_;     
            std::vector<CloudContainer<_FeatureT>> cur_feature_;   // 当前特征数据  
            std::vector<Eigen::Isometry3d> pose_lidar_cur_;  
            Eigen::Isometry3d lidar_lidar_estrinsic_;  

        public:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 
             * @param processor 使用的点云处理器
             * @param lidar_tracker 使用的激光里程计  
             * @param mode 运行模式设置 
             * @param NUM_OF_LASER 激光的数量 
             */                
            MultiLidarEstimator(std::vector<FeatureProcessorPtr> &processor,   // 每个激光一个点云处理器  
                                                        std::vector<LidarTrackerPtr> &lidar_tracker,   // 每个激光一个tracker
                                                        uint8_t NUM_OF_LASER = 2) 
            : MultiLidarEstimatorBase<_PointT, _FeatureT>(NUM_OF_LASER)
            , processor_(std::move(processor)), lidar_tracker_(std::move(lidar_tracker))
            {
                if (NUM_OF_LASER == 1) 
                {
                    Base::EXTRINSIC_ESTIMATE_ = 0;     // 不标定外参   
                }
                // 启动估计线程
                estimate_thread_ = std::thread(&MultiLidarEstimator::processMeasurements, this); 
                Base::backend_ = std::shared_ptr<BackEndOptimization<_FeatureT>>(
                    new BackEndOptimization<_FeatureT>("g2o"));
                pose_lidar_cur_.resize(Base::NUM_OF_LIDAR_, Eigen::Isometry3d::Identity());  
                lidar_lidar_estrinsic_.setIdentity();  
                // 数据管理器注册
                DataManager::GetInstance().Registration<MultiLidarResultInfo<_FeatureT>>("frontend_info", 1); 
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 对于Lidar数据的处理
             * @details 先进行点云处理  ，然后进行tracking  
             * @param data
             * @return {*}
             */                
            virtual void Process(MultiLidarData<_PointT> const& data) override 
            {        
                double t = data.timestamp;  
                assert(data.all_lidar_data.size() == Base::NUM_OF_LIDAR_);
                std::vector<CloudContainer<_FeatureT>> all_lidar_feature;       // 每一个激光雷达的特征数据
                all_lidar_feature.resize(Base::NUM_OF_LIDAR_); 
                //  openmp加速  
                TicToc tt;       
                tt.tic();                        
                #pragma omp parallel for num_threads(Base::NUM_OF_LIDAR_) // Base::NUM_OF_LIDAR_
                for (uint8_t i = 0; i < Base::NUM_OF_LIDAR_; i++)
                {
                    uint8_t id = data.all_lidar_data[i].first;  
                    processor_[id]->Process(data.all_lidar_data[i].second, all_lidar_feature[id]);  
                    all_lidar_feature[id].time_stamp_ = t; 
                    // std::cout<<"id: "<<static_cast<uint16_t>(id)<<"process done, before: "<<data.lidar_data_container_[i].first.size() <<
                    // " after: "<< feature_frame[id]->size()<<std::endl;
                }
                //tt.toc("lidar process "); 
                Base::m_buf_.lock();
                // 保存处理后的每一时刻的多激光数据  
                feature_buf_.push( std::move(all_lidar_feature));
                Base::m_buf_.unlock();
            }
            
            //  处理线程   
            virtual void processMeasurements() override
            {
                while (1)
                {
                    Base::m_buf_.lock();
                    if (!feature_buf_.empty())
                    {  // 获取最早时刻的全部激光雷达数据 
                        cur_feature_ = std::move(feature_buf_.front()); 
                        feature_buf_.pop();
                        Base::m_buf_.unlock();
                        // std::cout<<"feature_buf_ size: "<<feature_buf_.size()<<std::endl;
                        // // 核心 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        process();
                    }
                    Base::m_buf_.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
            /**
             *  @brief 估计器 核心函数     
             */
            virtual void process() override
            {
                std::vector<Eigen::Isometry3d> delta_pose_;    // 运动增量    
                delta_pose_.resize(Base::NUM_OF_LIDAR_, Eigen::Isometry3d::Identity());  
                MultiLidarResultInfo<_FeatureT> result;    // 保存多个雷达估计的结果  
                std::mutex m_result;  
                // 进行外参标定 
                if (Base::EXTRINSIC_ESTIMATE_ == 1 && Base::EXTRINSIC_CALIB_STATUS_ != 2)
                {
                    std::cout<<common::YELLOW<<"calibration extrinsic param !"<<common::RESET<<std::endl;
                    // 如果处于初始化阶段  
                    if (Base::EXTRINSIC_CALIB_STATUS_ == 0)
                    {
                        // 每个激光雷达单独计算里程计  
                        TicToc tt;  
                        tt.tic();  
                        #pragma omp parallel for num_threads(Base::NUM_OF_LIDAR_)
                        for (uint8_t id = 0; id < Base::NUM_OF_LIDAR_; id++)
                        {   // 运行里程计   
                            // TicToc tt;  
                            // tt.tic();  
                            CloudContainer<_FeatureT>& cur_cloud_feature = cur_feature_[id];   // 获取特征数据 
                            lidar_tracker_[id]->Solve(cur_cloud_feature, delta_pose_[id]);   
                            pose_lidar_cur_[id] = pose_lidar_cur_[id] * delta_pose_[id];         // 当前帧的绝对运动   
                            LidarResultInfo<_FeatureT> curr_lidar_res_info;
                            curr_lidar_res_info.time_stamps_ = cur_feature_[id].time_stamp_;
                            curr_lidar_res_info.pose_ = pose_lidar_cur_[id].matrix().cast<float>();
                            curr_lidar_res_info.feature_point_ = std::move(cur_cloud_feature.pointcloud_data_);  
                            m_result.lock();  
                            result.insert(make_pair(id, std::move(curr_lidar_res_info))); 
                            m_result.unlock();  
                        }
                        // tt.toc("lidar tracking "); 
                        DataManager::GetInstance().AddData("frontend_info", std::move(result));  
                        // 进行外参初始化 
                        if (extrinsics_init_.AddPose(Pose(delta_pose_[0]), 
                                                                                    Pose(delta_pose_[1])))
                        {
                            if (extrinsics_init_.CalibExRotation())
                            {
                                if (extrinsics_init_.CalibExTranslation())
                                {
                                    extrinsics_init_.GetCalibResult(lidar_lidar_estrinsic_);
                                    std::cout<<"lidar lidar calibrate done, estrinsic:"<<std::endl
                                    <<lidar_lidar_estrinsic_.matrix()<<std::endl;
                                    Base::EXTRINSIC_CALIB_STATUS_ = 1;      // 进入外参优化模式 
                                }
                            }
                        }
                    } // 外参优化阶段   将外参优化当成一个匹配问题  
                        // 用初始化的外参作为初值， 将辅助雷达的特征与主雷达local map 进行匹配
                    else if (Base::EXTRINSIC_CALIB_STATUS_ == 1) 
                    {
                        Eigen::Isometry3d primary_lidar_pose;
                        Eigen::Isometry3d sub_lidar_pose;

                        for (uint8_t id = 0; id < Base::NUM_OF_LIDAR_; id++)
                        {    
                            // 只对主雷达进行tracking
                            CloudContainer<_FeatureT>& cur_cloud_feature = cur_feature_[id];   // 获取特征数据
                            // 主雷达
                            if (id == 0)
                            {
                                lidar_tracker_[0]->Solve(cur_cloud_feature, delta_pose_[0]);   
                                // 匹配后获取主雷达在tracker local frame 的pose 
                                primary_lidar_pose = lidar_tracker_[0]->GetCurrPoseInLocalFrame();
                                // 辅雷达在主雷达local map中的pose 
                                sub_lidar_pose = primary_lidar_pose * lidar_lidar_estrinsic_;  
                                pose_lidar_cur_[0] = pose_lidar_cur_[0] * delta_pose_[0];       // 主雷达的运动 
                            } else {
                                lidar_tracker_[0]->RegistrationLocalMap(cur_cloud_feature, sub_lidar_pose);
                                lidar_lidar_estrinsic_ = primary_lidar_pose.inverse() * sub_lidar_pose; 
                                pose_lidar_cur_[1] = pose_lidar_cur_[0] * lidar_lidar_estrinsic_;  
                            }
                            
                            LidarResultInfo<_FeatureT> curr_lidar_res_info;
                            curr_lidar_res_info.time_stamps_ = cur_feature_[id].time_stamp_;
                            curr_lidar_res_info.pose_ = pose_lidar_cur_[id].matrix().cast<float>();
                            curr_lidar_res_info.feature_point_ = std::move(cur_cloud_feature.pointcloud_data_);  
                            m_result.lock();  
                            result.insert(make_pair(id, std::move(curr_lidar_res_info))); 
                            m_result.unlock();  
                        }
                        // tt.toc("lidar tracking "); 
                        DataManager::GetInstance().AddData("frontend_info", std::move(result));  
                        std::cout<<common::GREEN<<"calibrate optimize done!"<<common::RESET<<std::endl;
                        std::cout<<"estrinsic:"<<std::endl<<lidar_lidar_estrinsic_.matrix()<<std::endl;
                    }
                }
                // 多激光标定完成后 ，根据外参将 多激光拼接为一个激光雷达，使用一个tracker 进行跟踪
                CloudContainer<_FeatureT>& cur_cloud_feature = cur_feature_[0];   // 获取特征数据 
                lidar_tracker_[0]->Solve(cur_cloud_feature, delta_pose_[0]);   
                pose_lidar_cur_[0] = pose_lidar_cur_[0] * delta_pose_[0];   // 当前帧的绝对运动   
                // 检查是否需要添加关键帧到后端
                if (keyframe_updater_.NeedUpdate(pose_lidar_cur_[0], cur_cloud_feature.time_stamp_) 
                        != KeyframeUpdater::FALSE)
                {
                    // 将激光里程计数据添加到后端中进行处理 
                    Base::backend_->AddKeyFrame(cur_cloud_feature, pose_lidar_cur_[0], 
                                                                            keyframe_updater_.GetOdomIncrement());   
                }
                // 将即时估计的结果保存起来  用于数据发布
                LidarResultInfo<_FeatureT> curr_lidar_res_info;   
                curr_lidar_res_info.time_stamps_ = cur_feature_[0].time_stamp_;
                curr_lidar_res_info.pose_ = pose_lidar_cur_[0].matrix().cast<float>();
                curr_lidar_res_info.feature_point_ = std::move(cur_cloud_feature.pointcloud_data_);  
                curr_lidar_res_info.local_map_ = lidar_tracker_[0]->GetLocalMap(); 
                m_result.lock();  
                result.insert(make_pair(0, std::move(curr_lidar_res_info))); 
                m_result.unlock();  
                // tt.toc("lidar tracking "); 
                DataManager::GetInstance().AddData("frontend_info", std::move(result));  
            }
    }; // class MultiLidarEstimator 
}// namespace MultiLidarEstimator
#endif
