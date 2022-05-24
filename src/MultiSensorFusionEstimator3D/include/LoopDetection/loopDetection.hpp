/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-29 16:56:00
 * @Description: 闭环检测 - 基于激光描述子 + 几何位置    
 * @Others: 
 */
#pragma once 

#include "Sensor/lidar_data_type.h"
#include "BackEnd/pose_graph_database.hpp"
#include "Common/keyframe.hpp"
#include "Common/color.hpp"
#include "factory/registration/registration_factory.hpp"
#include "tic_toc.h"
#include "Algorithm/PointClouds/registration/alignEvaluate.hpp"
#include "SceneRecognitionScanContext.hpp"

namespace Slam3D
{
    #define LOOP_DEBUG 0

    /**
     * @brief: 基于雷达的闭环检测 - 1、先验位姿检测   2、点云描述子检测 
     *@param _PointType 回环时进行点云匹配的点类型 
     */    
    template<typename _PointType>
    class loopDetection
    {   
        private:
            using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;  
            using SourceT = std::pair<std::string, PointCloudConstPtr>;     // 匹配源类型    <id, 数据>
            using RegistrationAdapterPtr = std::unique_ptr<RegistrationAdapterBase< 
                SourceT, FeaturePointCloudContainer<_PointType>>>;   
            
            double DETECT_TIME_INTERVAL_ = 1.0;   // 检测的最小时间间隔    s   
            uint16_t DETECT_FRAME_INTERVAL_ = 3;   // 检测的最小帧间隔     
            uint16_t MIN_LOOP_FRAME_INTERVAL_ = 100;    //  一个回环的最小间隔  
            double MAX_LOOP_DISTANCE_ = 10;   //  回环之间   的最大距离  
            double MAX_ODOM_ERROR_ = 50;  // 最大里程计误差  
            std::mutex pcl_mt_;  
            std::mutex loop_mt_; 
            std::deque<FeaturePointCloudContainer<_PointType>> pointcloud_process_; 
            KeyFrame::Ptr curr_keyframe_; 
            double last_detect_time_ = -100;    
            uint32_t last_detect_id_ = 0;  
            uint32_t kdtree_size_ = 0;  
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses;  // 历史关键帧的位姿kdtree 
            std::thread loop_thread_;  
            NdtOmpPtr<_PointType> rough_ndt_ptr_;   // 粗匹配ndt
            NdtOmpPtr<_PointType> refine_ndt_ptr_;   // 细匹配ndt  

            RegistrationAdapterPtr rough_registration_;
            RegistrationAdapterPtr refine_registration_;

            std::deque<Edge> new_loops_;  
            // 针对 _PointType 点云的 匹配质量评估器 
            PointCloudAlignmentEvaluate<_PointType> align_evaluator_;
            SceneRecognitionScanContext<_PointType> scene_recognizer_; 

            std::vector<std::string> rough_registration_specific_name_;
            std::vector<std::string> refine_registration_specific_name_;
        
        public:
            loopDetection()
            {
                kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
                loop_thread_ = std::thread(&loopDetection::LoopDetect, this);  
                // 创建NDT
                NdtOmpPtr<_PointType> rough_ndt_ptr = make_ndtOmp<_PointType>(3.0, 0.01, 
                    0.3, 30, 4, "DIRECT1");  
                NdtOmpPtr<_PointType> refine_ndt_ptr = make_ndtOmp<_PointType>(1.0, 0.01, 
                    0.1, 30, 4, "DIRECT1");  
                // 将匹配算法默认设置为NDT 
                rough_registration_.reset(
                    new RegistrationAdapterImpl<SourceT, FeaturePointCloudContainer<_PointType>, 
                        pcl::Registration<_PointType, _PointType>>(std::move(rough_ndt_ptr), POINTS_PROCESSED_NAME));  
                refine_registration_.reset(
                    new RegistrationAdapterImpl<SourceT, FeaturePointCloudContainer<_PointType>, 
                        pcl::Registration<_PointType, _PointType>>(std::move(refine_ndt_ptr), POINTS_PROCESSED_NAME));  
                // 获取 匹配算法所需要的点云名字
                rough_registration_specific_name_ = rough_registration_->GetUsedPointsName();
                refine_registration_specific_name_ = refine_registration_->GetUsedPointsName();
            }
            virtual ~loopDetection() {}

            /**
             * @brief: 传入一帧激光数据 
             * @details 放入处理队列中 
             * @param {*}
             * @return {*}
             */            
            void AddData(FeaturePointCloudContainer<_PointType> const& scan_in) 
            {
                scene_recognizer_.AddKeyFramePoints(scan_in);
            }

            /**
             * @brief: 基于激光点云的重定位 
             * @details: 
             * @param scan_in 输入全部激光信息
             * @return <匹配历史帧的id, 重定位位姿>
             */            
            std::pair<int64_t, Eigen::Isometry3d> Relocalization(
                FeaturePointCloudContainer<_PointType> const& scan_in)
            {   
                TicToc tt; 
                // step1 先识别出相似帧  
                std::pair<int64_t, Eigen::Isometry3d> res = scene_recognizer_.FindSimilarPointCloud(scan_in);  
                if (res.first == -1) return res; 
                // step2 采用粗匹配 + 细匹配模式求解出位姿
                PoseGraphDataBase& poseGraph_database = PoseGraphDataBase::GetInstance(); 
                std::unordered_map<std::string, typename pcl::PointCloud<_PointType>::Ptr> owned_localmap;  
                // 将粗匹配所需要的点云local map 提取出来 
                for (auto const& name : rough_registration_specific_name_)
                {   // 从数据库中查找 名字为 name 的点云 
                    typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                    if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                        res.first, 5, name, local_map))
                    {
                        std::cout<<common::RED<<"Find local map ERROR "<<name<<common::RESET<<std::endl;
                        res.first = -1;
                        return res;  
                    }
                    rough_registration_->SetInputSource(std::make_pair(name, local_map)); 
                    owned_localmap[name] = local_map; 
                }
                rough_registration_->SetInputTarget(scan_in);
                // 当前帧位姿转换到世界系下
                Eigen::Isometry3d historical_pose;
                if (!poseGraph_database.SearchVertexPose(res.first, historical_pose)) {
                        std::cout<<common::RED<<"ERROR : not find historical pose "<<common::RESET<<std::endl;
                        res.first = -1;
                        return res;  
                }
                res.second = historical_pose * res.second;  
                // 回环first的点云
                if (!rough_registration_->Registration(res.second)) {
                    res.first = -1;
                    return res;  
                }
                // 细匹配
                // 将细匹配所需要的点云local map 提取出来 
                for (auto const& name : refine_registration_specific_name_)
                {
                    typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                    // 如果 细匹配所需要的local map 在之前粗匹配时  已经提取了
                    if (owned_localmap.find(name) != owned_localmap.end()) {
                        local_map = owned_localmap[name];  
                    } else {
                        if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                            res.first, 5, name, local_map))
                        {
                            res.first = -1;
                            return res;  
                        }
                        owned_localmap[name] = local_map; 
                    }
                    refine_registration_->SetInputSource(std::make_pair(name, local_map));  
                }
                refine_registration_->SetInputTarget(scan_in);
                if (!refine_registration_->Registration(res.second)) {
                    res.first = -1;
                    return res;  
                }
                // step3 检验   使用数据处理后的全部点云进行检验 而非特征
                typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                // 检验需要的是  POINTS_PROCESSED_NAME 的点云 
                if (owned_localmap.find(POINTS_PROCESSED_NAME) != owned_localmap.end()) {
                    local_map = owned_localmap[POINTS_PROCESSED_NAME];  
                } else {  // 如果之前没有构造出 POINTS_PROCESSED_NAME 的local map 那么这里构造
                    if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                        res.first, 5, POINTS_PROCESSED_NAME, local_map)) {
                        res.first = -1;
                        return res;  
                    }
                }
                align_evaluator_.SetTargetPoints(local_map); 
                double score = align_evaluator_.AlignmentScore(scan_in.at(POINTS_PROCESSED_NAME), 
                                                                                                                        res.second.matrix().cast<float>(), 0.1, 0.6); 
                tt.toc("Relocalization ");
                // score的物理意义 是 平均平方残差
                if (score > 0.05) {
                    res.first = -1;
                    return res;  
                }
                std::cout << common::GREEN<<"relocalization success!"<<std::endl;
                std::cout << common::GREEN<<"score: "<<score<<std::endl;
                return res;  
            }

            /**
             * @brief: 获取新增的回环信息
             */            
            std::deque<Edge> GetNewLoops()
            {
                loop_mt_.lock(); 
                std::deque<Edge> new_loop_copy = new_loops_;
                new_loops_.clear();
                loop_mt_.unlock();  
                return new_loop_copy;  
            }

            // 保存回环模块数据    
            void Save(std::string const& path)
            {
                scene_recognizer_.Save(path); 
            }

            // 加载回环模块数据    
            void Load(std::string const& path)
            {
                scene_recognizer_.Load(path); 
                pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud;
                keyframe_position_cloud = PoseGraphDataBase::GetInstance().GetKeyFramePositionCloud();
                if (!keyframe_position_cloud->empty()) {
                    kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
                    std::cout<<common::GREEN<<"位置点云KDTREE初始化完成!"<<common::RESET<<std::endl;
                }
            }

            /**
             * @brief: 在历史关键帧中查找位置与给定目标接近的
             * @details: 
             * @return 查找到的数量 
             */            
            int HistoricalPositionSearch(pcl::PointXYZ const& pos, double const& max_search_dis,
                                                                                        uint16_t const& max_search_num, std::vector<int> &search_ind,
                                                                                        std::vector<float> &search_dis)
            {   
                if (max_search_dis <= 0) {
                    // KNN搜索
                    return kdtreeHistoryKeyPoses->nearestKSearch(pos, max_search_num, search_ind, search_dis); 
                } 
                // 在历史关键帧中查找与当前关键帧距离小于阈值的集合  
                return kdtreeHistoryKeyPoses->radiusSearch(pos, max_search_dis, search_ind, search_dis, 
                                                                                                max_search_num); 
            }

        protected:
            /**
             * @brief: 回环处理线程 
             * @details 基于距离的回环检测
             *                      需求：在无GPS的情况下，认为最大存在 50m 的回环误差，
             *                                      且回环的最大距离设定为10m，车速最大 10m/s。
             *                      思路：若进行距离检测，范围100m内都没有候选关键帧存在，
             *                                      那么可以调低回环检测的频率(间隔3s)
             * 
             *                       进行距离回环检测的条件：由于只认为真实相距10m以内的是回环，
             *                        
             */        
            void LoopDetect() 
            {
                while(1)
                {
                    // 读取最新添加到数据库的关键帧进行回环检测  
                    KeyFrame curr_keyframe_;
                    PoseGraphDataBase& poseGraph_database = PoseGraphDataBase::GetInstance(); 
                    if (!poseGraph_database.GetNewKeyFrame(curr_keyframe_)) 
                    {
                        std::chrono::milliseconds dura(50);
                        std::this_thread::sleep_for(dura);
                        continue;  
                    }
                    // 每隔1s 进行一次回环检测，且本次回环检测至少与上一个回环相距3个帧  避免回环过于密集
                    if (curr_keyframe_.time_stamp_ - last_detect_time_ > DETECT_TIME_INTERVAL_
                          && curr_keyframe_.id_ - last_detect_id_ > DETECT_FRAME_INTERVAL_)
                    {   
                        TicToc tt;  
                        last_detect_time_ = curr_keyframe_.time_stamp_;
                        last_detect_id_ = curr_keyframe_.id_;  
                        // 机器人位置点云 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                            poseGraph_database.GetKeyFramePositionCloud();  

                        if (keyframe_position_cloud->size() < MIN_LOOP_FRAME_INTERVAL_) 
                        {
                            std::chrono::milliseconds dura(50);
                            std::this_thread::sleep_for(dura);
                            continue;  
                        }
                        // 判断是否需要更新位姿kdtree  
                        if (keyframe_position_cloud->size() - kdtree_size_ 
                                >= MIN_LOOP_FRAME_INTERVAL_ ) 
                        {
                            TicToc tt;
                            std::cout<<common::GREEN<<"updata loop kdtree!"<<common::RESET<<std::endl;
                            kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
                            kdtree_size_ = keyframe_position_cloud->size();  
                            tt.toc("updata loop kdtree ");    // 耗时 0.5 ms 
                        }
                        tt.tic(); 
                        // 场景识别模块工作  寻找相似帧
                        std::pair<int64_t, Eigen::Isometry3d> res{-1, Eigen::Isometry3d::Identity()};
                        res = scene_recognizer_.LoopDetect(curr_keyframe_.id_);   // 传入待识别的帧id 
                        tt.toc(" scene_recognize  "); 
                        // 如果场景识别没有找到相似帧   用位置搜索继续找回环
                        if (res.first == -1)
                        {
                            // 在历史关键帧中查找与当前关键帧距离小于阈值的集合  
                            std::vector<int> search_ind_;  
                            std::vector<float> search_dis_;
                            kdtreeHistoryKeyPoses->radiusSearch(keyframe_position_cloud->points[curr_keyframe_.id_], 
                                                                                                                MAX_LOOP_DISTANCE_, 
                                                                                                                search_ind_, 
                                                                                                                search_dis_, 
                                                                                                                0); 
                            // 在候选关键帧集合中，找到与当前帧间隔较远的帧 作为候选帧  
                            // 从距离近的帧开始遍历
                            for (int i = 0; i < (int)search_ind_.size(); ++i)
                            {
                                int id = search_ind_[i];
                                // 当前帧的idx 减去 搜索到的帧的idx  足够 大  
                                if ((keyframe_position_cloud->size() - 1) - id > MIN_LOOP_FRAME_INTERVAL_)
                                {
                                    std::cout<<common::GREEN<<"find loop!, id: "<<id<<", curr id: "<<curr_keyframe_.id_
                                    <<common::RESET<<std::endl;
                                    res.first = id;
                                    break;
                                }
                            }

                            if (res.first == -1) 
                            {
                                std::chrono::milliseconds dura(50);
                                std::this_thread::sleep_for(dura);
                                continue;  
                            }
                            // poseGraph_database.SearchKeyFramePose(curr_keyframe_.id_, res.second);  // 读取该帧的pose 
                            poseGraph_database.SearchVertexPose(curr_keyframe_.id_, res.second);  // 读取该帧的pose 
                            // 同一个地点  z轴的值应该相同
                            Eigen::Isometry3d historical_pose;
                            // poseGraph_database.SearchKeyFramePose(res.first, historical_pose);
                            poseGraph_database.SearchVertexPose(res.first, historical_pose);  // 读取该帧的pose 
                            res.second.translation()[2] = historical_pose.translation()[2];   // 位姿初始值
                            tt.tic(); 
                        } else {
                            // 当前帧位姿转换到世界系下
                            Eigen::Isometry3d historical_pose;
                            poseGraph_database.SearchVertexPose(res.first, historical_pose);
                            res.second = historical_pose * res.second;    // 位姿初始值
                        }
                        // 进行匹配  
                        // 粗匹配 + 细匹配的模式 
                        // // 去数据库中读取点云构造local map用于匹配  
                        // typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                        // if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(res.first, 5, 
                        //                                                                                                                                             "filtered", local_map))
                        // {
                        //     std::chrono::milliseconds dura(50);
                        //     std::this_thread::sleep_for(dura);
                        //     continue;  
                        // }
                        // #if (LOOP_DEBUG == 1)
                        //     pcl::io::savePCDFileBinary("/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loop_localmap.pcd"
                        //                                                                 , *local_map);
                        // #endif
                        // tt.toc("build local map ");
                        // // 当前点云
                        // typename pcl::PointCloud<_PointType>::Ptr curr_points(new pcl::PointCloud<_PointType>());
                        // poseGraph_database.GetKeyFramePointCloud<_PointType>("filtered", 
                        //                                                                                                                                 curr_keyframe_.id_, 
                        //                                                                                                                                 curr_points);
                        

                        // rough_ndt_ptr_->setInputTarget(local_map);
                        // rough_ndt_ptr_->setInputSource(curr_points); 

                        // typename pcl::PointCloud<_PointType>::Ptr aligned(new pcl::PointCloud<_PointType>());
                        // tt.tic(); 
                        // rough_ndt_ptr_->align(*aligned, res.second.matrix().cast<float>());  // 进行配准     predict_trans = setInputSource -> setInputTarget
                        // // std::cout<<"GetFitnessScore :"<<GetFitnessScore()<<std::endl;
                        // if (!rough_ndt_ptr_->hasConverged()) {
                        //     std::cout<<common::RED<<"loop match un-converged!"<<std::endl;
                        //     continue; 
                        // }
                        // // 对粗匹配进行评估 
                        // // 回环first的点云
                        // Eigen::Matrix4f T = rough_ndt_ptr_->getFinalTransformation();  
                        // align_evaluator_.SetTargetPoints(local_map); 
                        // // 1 m 一下为内点   内点占比至少为 0.8
                        // double score = align_evaluator_.AlignmentScore(curr_points, T, 1, 0.7); 
                        // std::cout << common::GREEN<<"loop match converged, score: "<< score<<std::endl;
                        // if (score > 1)
                        // {
                        //     #if (LOOP_DEBUG == 1)
                        //         static uint16_t bad_ind = 0; 
                        //         typename pcl::PointCloud<_PointType>::Ptr res_points(new pcl::PointCloud<_PointType>());
                        //         *res_points = *local_map + *aligned; 
                        //         pcl::io::savePCDFileBinary("/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/bad_loop_res_"
                        //                                                                 + std::to_string(bad_ind++) + ".pcd"
                        //                                                                     , *res_points);
                        //     #endif
                        //     continue; 
                        // }
                        // tt.toc("loop rough match ");
                        // tt.tic();  
                        // 粗匹配
                        std::unordered_map<std::string, typename pcl::PointCloud<_PointType>::Ptr> owned_localmap;  
                        FeaturePointCloudContainer<_PointType>  owned_curr_scan; 
                        // 将粗匹配所需要的点云提取出来 
                        for (auto const& name : rough_registration_specific_name_)
                        {   // local map 
                            typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                            if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                                res.first, 5, name, local_map))
                            {
                                std::cout<<common::RED<<"Find local map ERROR "<<name<<common::RESET<<std::endl;    
                                continue;
                            }
                            rough_registration_->SetInputSource(std::make_pair(name, local_map)); 
                            owned_localmap[name] = local_map; 
                            // 当前点云
                            typename pcl::PointCloud<_PointType>::Ptr curr_points(new pcl::PointCloud<_PointType>());
                            if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(name, 
                                                                                                                                                                    curr_keyframe_.id_, 
                                                                                                                                                                    curr_points)) 
                            {
                                std::cout<<common::RED<<"loop rough, Find curr points ERROR "
                                <<name<<common::RESET<<std::endl;
                                continue;
                            }
                            owned_curr_scan[name] = curr_points; 
                        }
                        rough_registration_->SetInputTarget(owned_curr_scan);
                        // 回环first的点云
                        if (!rough_registration_->Registration(res.second)) {
                            continue; 
                        }
                        // 匹配评估
                        typename pcl::PointCloud<_PointType>::Ptr evaluative_local_map(new pcl::PointCloud<_PointType>());
                        // 检验需要的是  名为POINTS_PROCESSED_NAME 的点云 
                        if (owned_localmap.find(POINTS_PROCESSED_NAME) != owned_localmap.end()) {
                            evaluative_local_map = owned_localmap[POINTS_PROCESSED_NAME];  
                        } else {  // 如果之前没有构造出 POINTS_PROCESSED_NAME 的local map 那么这里构造
                            if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                                res.first, 5, POINTS_PROCESSED_NAME, evaluative_local_map)){
                                continue; 
                            }
                            owned_localmap[POINTS_PROCESSED_NAME] = evaluative_local_map; 
                        }
                        align_evaluator_.SetTargetPoints(evaluative_local_map); 
                        typename pcl::PointCloud<_PointType>::ConstPtr evaluative_curr_scan(new pcl::PointCloud<_PointType>());
                        // 用于检验的当前点云
                        if (owned_curr_scan.find(POINTS_PROCESSED_NAME) != owned_curr_scan.end()) {
                            evaluative_curr_scan = owned_curr_scan[POINTS_PROCESSED_NAME];
                        } else {
                            typename pcl::PointCloud<_PointType>::Ptr evaluative_curr_scan_temp(new pcl::PointCloud<_PointType>());
                            if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(POINTS_PROCESSED_NAME, 
                                                                                                                                                                curr_keyframe_.id_, 
                                                                                                                                                                evaluative_curr_scan_temp)) 
                            {
                                std::cout<<common::RED<<"Find evaluative curr points ERROR, name: "
                                <<POINTS_PROCESSED_NAME<<common::RESET<<std::endl;
                                continue;  
                            }
                            owned_curr_scan[POINTS_PROCESSED_NAME] = evaluative_curr_scan; 
                            evaluative_curr_scan = evaluative_curr_scan_temp;  
                        }
                        double score = align_evaluator_.AlignmentScore(evaluative_curr_scan, 
                                                                                                                                res.second.matrix().cast<float>(), 1, 0.6); 
                        if (score > 1) {
                            continue; 
                        }
                        // 细匹配  
                        // refine_ndt_ptr_->setInputTarget(local_map);
                        // refine_ndt_ptr_->setInputSource(curr_points); 
                        // refine_ndt_ptr_->align(*aligned, T);
                        // if (!refine_ndt_ptr_->hasConverged()) {
                        //     std::cout<<common::RED<<"loop refine match un-converged!"<<std::endl;
                        //     continue; 
                        // }
                        // T = refine_ndt_ptr_->getFinalTransformation();  
                        // 将细匹配所需要的点云提取出来 
                        for (auto const& name : refine_registration_specific_name_)
                        {
                            typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                            // 如果 细匹配所需要的local map 在之前粗匹配时  已经提取了   那么直接用原数据
                            if (owned_localmap.find(name) != owned_localmap.end()) {
                                local_map = owned_localmap[name];  
                            } else {
                                if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(res.first, 5, 
                                        name, local_map)) {
                                    continue;  
                                }
                                owned_localmap[name] = local_map; 
                            }
                            refine_registration_->SetInputSource(std::make_pair(name, local_map));  
                            // 如果 细匹配所需要的 curr_data 不存在  则构造
                            if (owned_curr_scan.find(name) == owned_curr_scan.end()) 
                            {
                                typename pcl::PointCloud<_PointType>::Ptr curr_scan(new pcl::PointCloud<_PointType>());
                                if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(name, 
                                                                                                                                                                        curr_keyframe_.id_, 
                                                                                                                                                                        curr_scan)) 
                                {
                                    std::cout<<common::RED<<"loop refine, Find curr points ERROR "
                                    <<name<<common::RESET<<std::endl;
                                    continue;
                                }
                                owned_curr_scan[name] = curr_scan; 
                            }
                        }
                        refine_registration_->SetInputTarget(owned_curr_scan);
                        if (!refine_registration_->Registration(res.second)) {
                            continue; 
                        }
                        // 对细匹配进行评估
                        score = align_evaluator_.AlignmentScore(evaluative_curr_scan, res.second.matrix().cast<float>(), 0.1, 0.6); 
                        std::cout << common::GREEN<<"loop refine match converged, score: "<< score<<std::endl;
                        if (score > 0.05)
                        {
                            continue;  
                        }
                        #if (LOOP_DEBUG == 1)
                            // 检测回环匹配是否准确  
                            static uint16_t ind = 0; 
                            typename pcl::PointCloud<_PointType>::Ptr res_points(new pcl::PointCloud<_PointType>());
                            *res_points = *local_map + *aligned; 
                            pcl::io::savePCDFileBinary("/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loop_res_"
                                                                                    + std::to_string(ind++) + ".pcd"
                                                                                        , *res_points);
                        #endif
                        // 添加新增回环边
                        Edge new_loop; 
                        new_loop.link_id_.first = res.first;
                        new_loop.link_id_.second = curr_keyframe_.id_;

                        Eigen::Isometry3d historical_pose;
                        poseGraph_database.SearchVertexPose(res.first, historical_pose);
                        new_loop.constraint_ = historical_pose.inverse() * res.second;// T second -> first
                        loop_mt_.lock();
                        new_loops_.push_back(std::move(new_loop));  
                        loop_mt_.unlock();  
                    }
                    std::chrono::milliseconds dura(50);
                    std::this_thread::sleep_for(dura);
                }
            }
    }; // class 
} // namespace 

