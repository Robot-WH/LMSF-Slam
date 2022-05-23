/*
 * @Copyright(C): 
 * @Author: lwh
 * @Version: 1.0
 * @Description: 对各种优化库进行了适配，通过多态进行优化库的切换
 * @Others: 
 */

#pragma once 

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "backend_base.hpp"
#include "GraphOptimization/graph_optimization.hpp"
#include "GraphOptimization/graph_optimization_gtsam.hpp"
#include "GraphOptimization/graph_optimization_g2o.hpp"
#include "Common/data_manager.hpp"

namespace Slam3D {

    using common::DataManager; 

    /**
     * @brief: 后端优化模块 , 优化器可选用gtsam/g2o 
     * @details: 实现 
     * 1、融合激光里程计约束，平面先验约束，gnss 约束的滑动窗口优化
     * 2、回环之后进行全局优化  
     * @param {*}
     * @return {*}
     */    
    template<typename _FeatureT>
    class LifeLongBackEndOptimization : public BackEndOptimizationBase<_FeatureT>
    {
        private:
            enum WorkMode 
            {
                RELOCALIZATION = 0, // 开机之后处于该模式   接着会进行重定位找回定位 
                LOCALIZATION,
                MAPPING
            };
            using base = BackEndOptimizationBase<_FeatureT>; 
            using KeyFramePtr = typename base::KeyFramePtr; 
            uint32_t KF_index_ = 0;  
            // 局部优化每次处理的最大帧数  
            int max_keyframes_per_update_ = 10;
            int planeConstraint_optimize_freq_ = 5;
            bool enable_GNSS_optimize_ = false;  
            bool enable_planeConstraint_optimize_ = false;
            bool enable_incremental_mapping_ = false;  // 开启增量建图 
            // 回环模块 
            std::shared_ptr<loopDetection<_FeatureT>> loop_detect_ = nullptr;  
            // 优化器
            std::unique_ptr<GraphOptimizerInterface> optimizer_;  
            // 后端处理线程
            std::thread backend_thread_;  
            uint16_t new_external_constranit_num_;  // 新加的外部约束数量(GPS，地面, IMU..)  决定是否要进行一次全局优化  
            WorkMode work_mode; 
        public:

            LifeLongBackEndOptimization(std::string optimizer_type)
            {
                if (optimizer_type == "gtsam") {
                    optimizer_ = std::make_unique<GtsamGraphOptimizer>(); 
                } else if (optimizer_type == "g2o") {
                    optimizer_ = std::make_unique<G2oGraphOptimizer>(); 
                } else if (optimizer_type == "ceres") {
                }

                backend_thread_ = std::thread(&LifeLongBackEndOptimization::process, this);  // 启动线程   
                // 数据注册   
                DataManager::GetInstance().Registration<KeyFrameInfo<_FeatureT>>("keyframes_info", 1);
                DataManager::GetInstance().Registration<Eigen::Isometry3d>("odom_to_map", 1);
                work_mode = MAPPING;   // 默认为建图模式 
                std::cout<<common::GREEN<<"进入建图模式 ----------------- "<<std::endl;
            }

            /**
             * @brief: 外界调用的数据库载入接口 
             */            
            void Load() override
            {    // pose graph 加载
                if (!PoseGraphDataBase::GetInstance().Load()) 
                {
                    std::cout<<common::YELLOW<<"数据库载入失败，准备建图...... "<<std::endl;
                    return;
                }  
                work_mode = RELOCALIZATION;   // 默认为建图模式 
                std::cout<<common::GREEN<<"载入历史数据库，准备重定位......"<<std::endl;
                // 发布在载后的数据 
                KeyFrameInfo<_FeatureT> keyframe_info; 
                keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetAllVertex(); 
                keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetAllEdge(); 
                DataManager::GetInstance().AddData("keyframes_info", std::move(keyframe_info));   // 发布图关键帧  
            }

            /**
             * @brief: 添加激光里程计数据  
             * @details: 
             * @param lidar_data 激光雷达的数据  
             * @param odom 当前帧的前端里程计位姿 
             * @param between_constraint 与上一个关键帧的位姿约束  
             */            
            void AddKeyFrame(CloudContainer<_FeatureT> const& lidar_data, 
                                                        Eigen::Isometry3d const& odom, 
                                                        Eigen::Isometry3d const& between_constraint) override
            {
                // assert(loop_detect_ != nullptr);    // 检查回环模块是否被初始化
                static double last_keyframe_t = -1;
                // 检查时间戳 看是否乱序    
                if (lidar_data.time_stamp_ <= last_keyframe_t)
                {
                    std::cout<<common::RED<<"Backend -- AddKeyFrame(): timestamp error"
                    <<common::RESET<<std::endl;
                    return;
                }
                last_keyframe_t = lidar_data.time_stamp_; 
                if (work_mode == RELOCALIZATION) 
                {
                    // 重定位
                    std::cout<<common::GREEN<<"-----------------RELOCALIZATION!-----------------"<<std::endl;
                    std::pair<int64_t, Eigen::Isometry3d> res = loop_detect_->Relocalization(lidar_data.pointcloud_data_); 
                    // 重定位失败 ，若开启增量建图，则进行建图，否则延迟一下，继续重定位 
                    if (res.first == -1) {
                        if (enable_incremental_mapping_) {
                            work_mode = MAPPING; // 进入建图模式 
                            std::cout<<common::YELLOW<<"-----------------RELOCALIZATION FAIL!-----------------"<<std::endl;
                            std::cout<<common::GREEN<<"-----------------Begin Mapping !-----------------"<<std::endl;
                        } 
                    } else {
                        work_mode = LOCALIZATION; // 进入定位模式 
                        base::trans_odom2map_ = res.second * odom.inverse(); 
                        DataManager::GetInstance().AddData("odom_to_map", base::trans_odom2map_);      // 发布坐标变换
                    }
                } else if (work_mode == LOCALIZATION) {
                    // 定位如果丢失，若开启增量建图 incremental mapping 则会进行建图，否则进入重定位模式 
                    std::cout<<common::GREEN<<"-----------------LOCALIZATION!-----------------"<<std::endl;
                } else if (work_mode == MAPPING) {
                    mapping(lidar_data, odom, between_constraint);
                }
            }
            // 回环模块在system中初始化  再设置进来  
            virtual void SetLoopDetection(std::shared_ptr<loopDetection<_FeatureT>> const& loop_detect) override {
                loop_detect_ = loop_detect;  
            }

        protected:

            /**
             * @brief: 建图模式
             */            
            void mapping(CloudContainer<_FeatureT> const& lidar_data, 
                                            Eigen::Isometry3d const& odom, 
                                            Eigen::Isometry3d const& between_constraint)
            {
                // 线程锁开启
                std::lock_guard<std::mutex> lock(base::keyframe_queue_mutex_); 
                // 把关键帧点云存储到硬盘里     不消耗内存
                // 如果未来维护关键帧包括关键帧的删除或替换的话 , 那么 KF_index_ 的也需要去维护 
                for (auto iter = lidar_data.pointcloud_data_.begin(); 
                            iter != lidar_data.pointcloud_data_.end(); ++iter) 
                {  // 存到数据库中  
                    PoseGraphDataBase::GetInstance().AddKeyFramePointCloud(iter->first, KF_index_, *(iter->second));
                }
                // 通过点云与里程计和累计距离等来创建关键帧   实际的关键帧中就可以不用包含点云数据了  
                KeyFrame keyframe(lidar_data.time_stamp_, odom, KF_index_);
                keyframe.between_constraint_ = between_constraint;      // 获取该关键帧与上一关键帧之间的相对约束  last<-curr       
                base::new_keyframe_queue_.push_back(keyframe);     // 加入处理队列
                KF_index_++;
                // 数据加入到回环模块进行处理
                loop_detect_->AddData(lidar_data.pointcloud_data_);    // 点云
                // 将数据上传到数据管理器   供其他模块使用 
                KeyFrameInfo<_FeatureT> keyframe_info; 
                keyframe_info.time_stamps_ = lidar_data.time_stamp_;  
                keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetAllVertex(); 
                keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetAllEdge(); 
                keyframe_info.new_keyframes_ = base::new_keyframe_queue_;  
                DataManager::GetInstance().AddData("keyframes_info", std::move(keyframe_info));   // 发布图关键帧  
                DataManager::GetInstance().AddData("odom_to_map", base::trans_odom2map_);      // 发布坐标变换
            }
            
            /**
             * @brief: 定位模式 
             */            
            void localization()
            {
            }

            /**
             * @brief: 后端建图线程  
             */            
            void process() override
            {
                while(true)
                {
                    // 数据处理 
                    if (!processData()) {
                        std::chrono::milliseconds dura(1000);
                        std::this_thread::sleep_for(dura);
                        continue;  
                    }
                    PoseGraphDataBase& database = PoseGraphDataBase::GetInstance();  
                    if (optimize()) 
                    {   
                        TicToc tt;
                        // 优化完成后 更新数据库  
                        for(int i=0; i < optimizer_->GetNodeNum(); i++) {
                            database.UpdateVertexPose(i, optimizer_->ReadOptimizedPose(i)); 
                        }
                        tt.toc("update dataset ");
                        base::keyframe_queue_mutex_.lock();  
                        // 计算坐标转换矩阵
                        base::trans_odom2map_ = database.GetLastVertex().pose_
                            * database.GetLastKeyFrameData().odom_.inverse();  
                        base::keyframe_queue_mutex_.unlock();  
                    }
                    std::chrono::milliseconds dura(1000);
                    std::this_thread::sleep_for(dura);
                }
            }    

            /**
             * @brief: 对于新加入的关键帧new_keyframe_queue_，对每一个关键帧匹配约束，
             * @details: 约束对应好后放入 wait_optimize_keyframes_容器
             */            
            bool processData()
            {
                std::lock_guard<std::mutex> lock(base::keyframe_queue_mutex_); 
                // 对新加入的关键帧进行处理      
                // 如果没有新的关键帧  
                if (base::new_keyframe_queue_.empty()) {
                    return false;
                }
                // 处理的数量  
                int num_processed = std::min<int>(base::new_keyframe_queue_.size(), 
                                                                                            max_keyframes_per_update_);
                // 遍历全部关键帧队列       
                for (int i = 0; i < num_processed; i++) 
                {
                    // 从keyframe_queue中取出关键帧
                    auto& keyframe = base::new_keyframe_queue_[i];
                    uint64_t id = PoseGraphDataBase::GetInstance().ReadVertexNum();
                    Eigen::Isometry3d corrected_pose = base::trans_odom2map_ * keyframe.odom_; 
                    // 添加节点  
                    if (id == 0) {
                        // 第一个节点默认 fix
                        //optimizer_->AddSe3Node(keyframe.correct_pose_, id, true);    
                        optimizer_->AddSe3Node(corrected_pose, id, true);
                        //  添加到数据库中   图优化中的node 和 数据库中的关键帧 序号是一一对应的
                        PoseGraphDataBase::GetInstance().AddKeyFrameData(keyframe);   
                        PoseGraphDataBase::GetInstance().AddVertex(id, corrected_pose);  
                        PoseGraphDataBase::GetInstance().AddPosePoint(corrected_pose);  
                        continue;
                    }
                    optimizer_->AddSe3Node(corrected_pose, id); 
                    // 观测噪声
                    Eigen::Matrix<double, 1, 6> noise;
                    noise << 0.0025, 0.0025, 0.0025, 0.0001, 0.0001, 0.0001;
                    optimizer_->AddSe3Edge(id - 1, id, keyframe.between_constraint_, noise);  
                    PoseGraphDataBase::GetInstance().AddEdge(id - 1, id, keyframe.between_constraint_, noise);  

                    // 与GNSS进行匹配 
                    // 寻找有无匹配的GPS   有则
                    // if (!pairGnssOdomInSingle(GNSS_queue, keyframe))                // return true 匹配完成   false 继续等待数据  
                    // { 
                    //     if (ros::Time::now().toSec() - keyframe->stamp.toSec() < 1)   // 最多等待1s 
                    //     {
                    //         break;
                    //     }
                    // }
                    // else
                    // { 
                    //     // 匹配上GNSS数据了
                    //     // 按照一定频率添加GNSS约束  
                    //     static int gnss_freq_count = 0;
                    //     if(gnss_freq_count <= 0) 
                    //     {
                    //         keyframe->GNSS_Valid = true;  
                    //         gnss_freq_count = GNSS_optimize_freq;  
                    //     }
                    //     else 
                    //     {
                    //         gnss_freq_count--;
                    //     }
                        
                    //     if(keyframe->gnss_matched) 
                    //     {
                    //         Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();  
                            
                    //         gt_pose.block<3,1>(0, 3) = keyframe->utm_coord; 
                    //         gt_pose.block<3,3>(0, 0) =  keyframe->orientation.matrix(); 
                    //         SaveTrajectory(gt_pose, keyframe->Pose.matrix(), "/slam_data/trajectory", "/slam_data/trajectory/ground_truth.txt",
                    //                         "/slam_data/trajectory/est_path.txt");
                    //     }
                    // }

                    // 按照一定频率添加先验平面约束  
                    if (enable_planeConstraint_optimize_) {
                        static int planeConstraint_freq_count = 0;
                        if (planeConstraint_freq_count <= 0) {
                            keyframe.planeConstraint_valid_ = true;  
                            planeConstraint_freq_count = planeConstraint_optimize_freq_;  
                        } else {
                            planeConstraint_freq_count--;
                        }
                    }     
                    //  添加到数据库中  
                    PoseGraphDataBase::GetInstance().AddKeyFrameData(keyframe);   
                    PoseGraphDataBase::GetInstance().AddVertex(id, corrected_pose);  
                    PoseGraphDataBase::GetInstance().AddPosePoint(corrected_pose);  
                }

                base::new_keyframe_queue_.erase(base::new_keyframe_queue_.begin(), 
                    base::new_keyframe_queue_.begin() + num_processed);     //  [first,last) 
                
                return true;
            }

            /**
             * @brief: 全局pose graph优化
             * @details: 
             * @return 是否进行了优化 
             */            
            bool optimize() override
            {
                static bool do_optimize = false;  
                static bool has_loop = false;  
                // 提取所有新添加的回环数据
                std::deque<Edge>  new_loops = loop_detect_->GetNewLoops();
                for (uint16_t i = 0; i < new_loops.size(); i++) 
                {
                    // 添加回环边
                    Eigen::Matrix<double, 1, 6> noise;
                    noise << 0.0025, 0.0025, 0.0025, 0.0001, 0.0001, 0.0001;
                    optimizer_->AddSe3Edge(new_loops[i].link_id_.first, new_loops[i].link_id_.second, 
                                                                            new_loops[i].constraint_, noise);  
                    // 回环数据记录到数据库中
                    PoseGraphDataBase::GetInstance().AddEdge(new_loops[i]); 
                    has_loop = true;   // 只要有回环就触发优化
                }
                // 如果累计的外部约束超过10个  也触发一次优化 
                if (new_external_constranit_num_ > 10) {
                    do_optimize = true; 
                    new_external_constranit_num_ = 0; 
                }
                // optimize the pose graph
                // 执行优化
                static int optimize_dormant = 0;
                if (optimize_dormant <= 0)
                {
                    TicToc tt;
                    if (!do_optimize && !has_loop) return false;  
                    std::cout<<common::GREEN<<"Begin optimize! ------------------"
                    <<common::RESET<<std::endl;
                    optimizer_->Optimize(has_loop);  
                    std::cout<<common::GREEN<<"optimize done! ------------------"
                    <<common::RESET<<std::endl;
                    tt.toc("optimize ");
                    has_loop = false; 
                    do_optimize = false;  
                } else {
                    optimize_dormant--;  
                    return false;  
                 }
                return true;  
            }
    }; // class 
} // namespace 
