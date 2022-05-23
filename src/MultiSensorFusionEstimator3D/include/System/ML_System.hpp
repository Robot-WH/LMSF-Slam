/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-22 01:23:01
 * @Description:  多激光SLAM系统: 负责SLAM 系统各个模块的管理、 调度 
 * @Others: 注：系统主要分为 数据处理、前端里程计、后端优化、回环检测 
 */
#pragma once 

#include <omp.h>  
#include "Algorithm/PointClouds/processing/FeatureExtract/LOAMFeatureProcessor_base.hpp"
#include "Algorithm/calibration/handeye_calibration_base.hpp"
#include "Algorithm/PointClouds/processing/common_processing.hpp"
#include "Algorithm/PointClouds/processing/deskew/deskew_base.hpp"
#include "Algorithm/PointClouds/processing/Preprocess/RotaryLidar_preprocessing.hpp"
#include "LidarTracker/LidarTrackerLocalMap.hpp"
#include "Common/pose.hpp"
#include "Common/color.hpp"
#include "Common/keyframe_updater.hpp"
#include "BackEnd/backend_lifelong.hpp"

namespace Slam3D {

    using Algorithm::RotaryLidarPreProcess;
    using Algorithm::HandEyeCalibrationBase;
    using Algorithm::PointCloudProcessBase;
    using common::DataManager;

    /**
     * @brief:  多激光融合系统
     * @param _PointT 输入的点的类型
     * @param _FeatureT 点云处理后实际使用的特征类型 
     */    
    template<typename _PointT, typename _FeatureT>
    class MultiLidarSystem
    {
        public:
            using PreProcessorPtr = std::unique_ptr<RotaryLidarPreProcess<_PointT>>;
            using FeatureProcessorPtr = std::unique_ptr<PointCloudProcessBase<_PointT, _FeatureT>>;  
            using LidarTrackerPtr = std::unique_ptr<LidarTrackerBase<_FeatureT>>;  
        private:
            /**
             * @todo 这个路径目前写死了   应该改为从yaml 文件中读取的形式  
             */            
            std::string database_save_path_ = "/home/lwh/SlamDataBase";  
            // 模块  
            std::vector<PreProcessorPtr> pre_processors_;  // 预处理器  
            std::vector<FeatureProcessorPtr> processors_;   // 每个激光一个点云处理器  
            std::vector<LidarTrackerPtr> lidar_trackers_;  // 每个激光一个tracker, 初始化后只使用一个tracker 
            std::shared_ptr<BackEndOptimizationBase<_FeatureT>> backend_;    // 后端优化模块 可选g2o/gtsam  
            std::shared_ptr<loopDetection<_FeatureT>> loop_detect_; 
            HandEyeCalibrationBase extrinsics_init_;    // 外参初始化
            KeyframeUpdater keyframe_updater_; 

            std::thread estimate_thread_;
            // 提取的特征数据       <时间戳，vector(每一个雷达的特征数据)>  
            std::queue<std::vector<CloudContainer<_FeatureT>>> feature_buf_;     
            std::vector<CloudContainer<_FeatureT>> cur_feature_;   // 当前特征数据  
            std::vector<Eigen::Isometry3d> pose_lidar_cur_;  
            Eigen::Isometry3d lidar_lidar_estrinsic_;  

            bool b_system_inited_ = false;   
            uint8_t EXTRINSIC_CALIB_STATUS_;     // 外参估计
            uint8_t EXTRINSIC_ESTIMATE_;  // 是否进行外参估计
            uint8_t EXTRINSIC_OPTI_;  //  外参在线优化 标志
            uint8_t NUM_OF_LIDAR_;   
            std::mutex m_buf_;  

        public:
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 
             * @param processor 使用的点云处理器
             * @param lidar_tracker 使用的激光里程计  
             * @param mode 运行模式设置 
             * @param NUM_OF_LIDAR_ 激光的数量 
             */                
            MultiLidarSystem( 
                std::vector<FeatureProcessorPtr> &processor,   // 每个激光一个点云处理器  
                std::vector<LidarTrackerPtr> &lidar_tracker,   // 每个激光一个tracker
                uint8_t NUM_OF_LIDAR = 2) 
                : processors_(std::move(processor)), 
                  lidar_trackers_(std::move(lidar_tracker)),
                  NUM_OF_LIDAR_(NUM_OF_LIDAR)
            {
                if (NUM_OF_LIDAR_ == 1) 
                {
                    EXTRINSIC_ESTIMATE_ = 0;     // 不标定外参   
                }
                for (uint8_t i = 0; i < NUM_OF_LIDAR_; i++)
                {
                    pre_processors_.emplace_back(
                            new RotaryLidarPreProcess<_PointT>()); 
                }
                // 后端
                backend_ = std::shared_ptr<LifeLongBackEndOptimization<_FeatureT>>(
                    new LifeLongBackEndOptimization<_FeatureT>("g2o"));
                loop_detect_ = std::make_shared<loopDetection<_FeatureT>>();
                backend_->SetLoopDetection(loop_detect_);  
                // 启动估计线程
                estimate_thread_ = std::thread(&MultiLidarSystem::processMeasurements, this); 
                // 设置数据库保存路径
                PoseGraphDataBase::GetInstance().SetSavePath(database_save_path_);  

                pose_lidar_cur_.resize(NUM_OF_LIDAR_, Eigen::Isometry3d::Identity());  
                lidar_lidar_estrinsic_.setIdentity();  
                // 数据管理器注册
                DataManager::GetInstance().Registration<MultiLidarResultInfo<_FeatureT>>("frontend_info", 1); 
                // 加载历史数据库
                SystemLoad();
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief: 对于Lidar数据的处理
             * @details 先进行点云处理  ，然后进行tracking  
             * @param data
             * @return {*}
             */                
            void Process(MultiLidarData<_PointT> &data)
            {        
                double t = data.timestamp;  
                assert(data.all_lidar_data.size() == NUM_OF_LIDAR_);
                std::vector<CloudContainer<_FeatureT>> all_cloud_data;   // 每一个激光雷达的特征数据
                all_cloud_data.resize(NUM_OF_LIDAR_); 
                //  openmp加速  
                TicToc tt;       
                tt.tic();             
                // 如果有数据    则进行数据处理
                // 1、给出时间戳   2、去除畸变  
                //  openmp加速  
                #pragma omp parallel for num_threads(NUM_OF_LIDAR_)
                for (uint8_t i = 0; i < NUM_OF_LIDAR_; i++)
                {        
                    pre_processors_[i]->Process(data.all_lidar_data[i].second);       
                    // 去除畸变  
                }
                // 对于预处理完成的数据   进行后续的处理
                #pragma omp parallel for num_threads(NUM_OF_LIDAR_) // NUM_OF_LIDAR_
                for (uint8_t i = 0; i < NUM_OF_LIDAR_; i++)
                {
                    uint8_t id = data.all_lidar_data[i].first;   // 激光雷达的id号
                    processors_[id]->Process(data.all_lidar_data[i].second, all_cloud_data[id]);  
                    all_cloud_data[id].time_stamp_ = t; 
                    // 如果 没有 POINTS_PROCESSED_NAME的点云  那么需要将 原点云(仅经过预处理)的点云加入
                    if (all_cloud_data[id].pointcloud_data_.find(POINTS_PROCESSED_NAME) 
                            == all_cloud_data[id].pointcloud_data_.end())
                    {
                        typename pcl::PointCloud<_FeatureT>::Ptr points_ptr(new pcl::PointCloud<_FeatureT>());
                        pcl::copyPointCloud(data.all_lidar_data[i].second.point_cloud, *points_ptr); 
                        all_cloud_data[id].pointcloud_data_[POINTS_PROCESSED_NAME] = points_ptr;  
                    }
                }
                //tt.toc("lidar process "); 
                m_buf_.lock();
                // 保存处理后的每一时刻的多激光数据  
                feature_buf_.push( std::move(all_cloud_data));
                m_buf_.unlock();
            }

            // 数据保存
            void SavePoseGraph()
            {
                PoseGraphDataBase::GetInstance().Save();  
                loop_detect_->Save(database_save_path_);     
            }

            // 保存全局地图
            void SaveGlobalMap()
            {
            }
            
            /**
             * @brief: 加载数据 
             * @details:  1、后端加载   2、回环加载 
             */            
            void SystemLoad()
            {
                backend_->Load();  
                loop_detect_->Load(database_save_path_); // 加载场景识别数据库 
            }

        private:
            //  处理线程   
            void processMeasurements() 
            {
                while (1)
                {
                    m_buf_.lock();
                    if (!feature_buf_.empty())
                    {  // 获取最早时刻的全部激光雷达数据 
                        cur_feature_ = std::move(feature_buf_.front()); 
                        feature_buf_.pop();
                        m_buf_.unlock();
                        // std::cout<<"feature_buf_ size: "<<feature_buf_.size()<<std::endl;
                        // // 核心 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        process();
                    }
                    m_buf_.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }

            /**
             *  @brief 估计器 核心函数     
             */
            void process() 
            {
                std::vector<Eigen::Isometry3d> delta_pose_;    // 运动增量    
                delta_pose_.resize(NUM_OF_LIDAR_, Eigen::Isometry3d::Identity());  
                MultiLidarResultInfo<_FeatureT> result;    // 保存多个雷达估计的结果  
                std::mutex m_result;  
                // 进行外参标定 
                if (EXTRINSIC_ESTIMATE_ == 1 && EXTRINSIC_CALIB_STATUS_ != 2)
                {
                    std::cout<<common::YELLOW<<"calibration extrinsic param !"<<common::RESET<<std::endl;
                    // 如果处于初始化阶段  
                    if (EXTRINSIC_CALIB_STATUS_ == 0)
                    {
                        // 每个激光雷达单独计算里程计  
                        TicToc tt;  
                        tt.tic();  
                        #pragma omp parallel for num_threads(NUM_OF_LIDAR_)
                        for (uint8_t id = 0; id < NUM_OF_LIDAR_; id++)
                        {   // 运行里程计   
                            // TicToc tt;  
                            // tt.tic();  
                            CloudContainer<_FeatureT>& cur_cloud_feature = cur_feature_[id];   // 获取特征数据 
                            lidar_trackers_[id]->Solve(cur_cloud_feature.pointcloud_data_, 
                                                                                    cur_cloud_feature.time_stamp_, delta_pose_[id]);   
                            pose_lidar_cur_[id] = pose_lidar_cur_[id] * delta_pose_[id];         // 当前帧的绝对运动   
                            LidarResultInfo<_FeatureT> curr_lidar_res_info;
                            curr_lidar_res_info.time_stamps_ = cur_cloud_feature.time_stamp_;
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
                                    EXTRINSIC_CALIB_STATUS_ = 1;      // 进入外参优化模式 
                                }
                            }
                        }
                    } // 外参优化阶段   将外参优化当成一个匹配问题  
                        // 用初始化的外参作为初值， 将辅助雷达的特征与主雷达local map 进行匹配
                    else if (EXTRINSIC_CALIB_STATUS_ == 1) 
                    {
                        Eigen::Isometry3d primary_lidar_pose;
                        Eigen::Isometry3d sub_lidar_pose;

                        for (uint8_t id = 0; id < NUM_OF_LIDAR_; id++)
                        {    
                            // 只对主雷达进行tracking
                            CloudContainer<_FeatureT>& cur_cloud_feature = cur_feature_[id];   // 获取特征数据
                            // 主雷达
                            if (id == 0)
                            {
                                lidar_trackers_[0]->Solve(cur_cloud_feature.pointcloud_data_, 
                                                                                    cur_cloud_feature.time_stamp_, delta_pose_[id]);   
                                // 匹配后获取主雷达在tracker local frame 的pose 
                                primary_lidar_pose = lidar_trackers_[0]->GetCurrPoseInLocalFrame();
                                // 辅雷达在主雷达local map中的pose 
                                sub_lidar_pose = primary_lidar_pose * lidar_lidar_estrinsic_;  
                                pose_lidar_cur_[0] = pose_lidar_cur_[0] * delta_pose_[0];       // 主雷达的运动 
                            } else {
                                lidar_trackers_[0]->RegistrationLocalMap(cur_cloud_feature.pointcloud_data_, 
                                                                                                                        sub_lidar_pose);
                                lidar_lidar_estrinsic_ = primary_lidar_pose.inverse() * sub_lidar_pose; 
                                pose_lidar_cur_[1] = pose_lidar_cur_[0] * lidar_lidar_estrinsic_;  
                            }
                            
                            LidarResultInfo<_FeatureT> curr_lidar_res_info;
                            curr_lidar_res_info.time_stamps_ = cur_cloud_feature.time_stamp_;
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
                /**
                 * @todo 这样并不合理，原因：1、每个激光雷达应该与自己产生的local map进行匹配，将多个激光的点云
                 *                                                                    直接拼接在一起使用，会造成local map规模过大，增加无效搜索，
                 *                                                                    因此应该将每个激光雷达的local map 分别管理。
                 *                                                             2、由于外参还是可能会有一些误差的，因此直接拼接，后续也无法继续优化外参了
                 */
                CloudContainer<_FeatureT>& cur_cloud_feature = cur_feature_[0];   // 获取特征数据 
                lidar_trackers_[0]->Solve(cur_cloud_feature.pointcloud_data_, 
                                                                        cur_cloud_feature.time_stamp_, delta_pose_[0]);   
                pose_lidar_cur_[0] = pose_lidar_cur_[0] * delta_pose_[0];   // 当前帧的绝对运动   
                // 检查是否需要添加关键帧到后端
                if (keyframe_updater_.NeedUpdate(pose_lidar_cur_[0], cur_cloud_feature.time_stamp_) 
                        != KeyframeUpdater::FALSE)
                {
                    // 将激光里程计数据添加到后端中进行处理 
                    backend_->AddKeyFrame(cur_cloud_feature, pose_lidar_cur_[0], 
                                                                            keyframe_updater_.GetOdomIncrement());   
                }
                // 将即时估计的结果保存起来  用于数据发布
                LidarResultInfo<_FeatureT> curr_lidar_res_info;   
                curr_lidar_res_info.time_stamps_ = cur_cloud_feature.time_stamp_;
                curr_lidar_res_info.pose_ = pose_lidar_cur_[0].matrix().cast<float>();
                curr_lidar_res_info.feature_point_ = std::move(cur_cloud_feature.pointcloud_data_);  
                curr_lidar_res_info.local_map_ = lidar_trackers_[0]->GetLocalMap(); 
                m_result.lock();  
                result.insert(make_pair(0, std::move(curr_lidar_res_info))); 
                m_result.unlock();  
                // tt.toc("lidar tracking "); 
                DataManager::GetInstance().AddData("frontend_info", std::move(result));  
            }
    }; // class MultiLidarSystem 
}// namespace 
