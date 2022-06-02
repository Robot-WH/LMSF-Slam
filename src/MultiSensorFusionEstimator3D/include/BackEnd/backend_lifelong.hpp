/*
 * @Copyright(C): 
 * @Author: lwh
 * @Version: 1.0
 * @Description: 对各种优化库进行了适配，通过多态进行优化库的切换
 * @Others: 
 */

#pragma once 

#include <stdexcept>
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
    #define debug 1

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
            using PointCloudConstPtr = typename pcl::PointCloud<_FeatureT>::ConstPtr;  
            using SourceT = std::pair<std::string, PointCloudConstPtr>;     // 匹配源类型    <id, 数据>
            using RegistrationAdapterPtr = std::unique_ptr<RegistrationAdapterBase< 
                SourceT, FeaturePointCloudContainer<_FeatureT>>>;   
            // 局部优化每次处理的最大帧数  
            int max_keyframes_per_update_ = 10;
            int planeConstraint_optimize_freq_ = 5;
            bool enable_GNSS_optimize_ = false;  
            bool enable_planeConstraint_optimize_ = false;
            bool enable_map_update_ = true;  // 开启地图更新 
            bool has_loop_ = false;  
            // 回环模块 
            std::shared_ptr<loopDetection<_FeatureT>> loop_detect_ = nullptr;  
            // 优化器
            std::unique_ptr<GraphOptimizerInterface> optimizer_;  
            // 定位匹配
            RegistrationAdapterPtr localize_registration_;
            // 匹配评估器
            PointCloudAlignmentEvaluate<_FeatureT> align_evaluator_;
            // 后端处理线程
            std::thread mapping_thread_;  
            std::thread localization_thread_;  
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
                // 将定位匹配算法默认设置为NDT 
                NdtOmpPtr<_FeatureT> ndt = make_ndtOmp<_FeatureT>(1.0, 0.01, 
                    0.1, 30, 4, "DIRECT1");  
                localize_registration_.reset(
                    new RegistrationAdapterImpl<SourceT, FeaturePointCloudContainer<_FeatureT>, 
                        pcl::Registration<_FeatureT, _FeatureT>>(std::move(ndt), POINTS_PROCESSED_NAME));  
                work_mode = MAPPING;  
                mapping_thread_ = std::thread(&LifeLongBackEndOptimization::mapping, this);  // 启动线程  
                localization_thread_ = std::thread(&LifeLongBackEndOptimization::localization, this);   
                // 数据注册   
                DataManager::GetInstance().Registration<KeyFrameInfo<_FeatureT>>("keyframes_info", 1);
                DataManager::GetInstance().Registration<Eigen::Isometry3d>("odom_to_map", 1);
                DataManager::GetInstance().Registration<LocalizationPointsInfo<_FeatureT>>("loc_points", 1);
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
                // 恢复历史图优化
                optimizer_->Rebuild(PoseGraphDataBase::GetInstance().GetAllVertex(), 
                                                            PoseGraphDataBase::GetInstance().GetAllEdge()); 
                // 从优化器中读回节点位姿
                for(int i=0; i < optimizer_->GetNodeNum(); i++) {
                    PoseGraphDataBase::GetInstance().UpdateVertexPose(i, optimizer_->ReadOptimizedPose(i)); 
                }
                // 发布在载后的数据 
                KeyFrameInfo<_FeatureT> keyframe_info; 
                keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetAllVertex(); 
                keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetAllEdge(); 
                DataManager::GetInstance().AddData("keyframes_info", std::move(keyframe_info));   // 发布图关键帧  
            }

            /**
             * @brief: 添加激光里程计数据  
             * @param lidar_data 激光雷达的数据  
             * @param odom 当前帧的前端里程计位姿 
             * @param between_constraint 与上一个关键帧的位姿约束  
             */            
            void AddKeyFrame(CloudContainer<_FeatureT> const& lidar_data, 
                                                        Eigen::Isometry3d const& odom, 
                                                        Eigen::Isometry3d const& between_constraint) override
            {
                assert(loop_detect_ != nullptr);    
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
                    // 重定位失败 ，若开启地图更新，此时会重新建立一条新的轨迹，否则延迟一下，继续重定位 
                    if (res.first == -1) {
                        if (enable_map_update_) {
                        } 
                    } else {
                        work_mode = LOCALIZATION; // 进入定位模式 
                        //localization_thread_ = std::thread(&LifeLongBackEndOptimization::localization, this);  
                        base::trans_odom2map_ = res.second * odom.inverse(); 
                        DataManager::GetInstance().AddData("odom_to_map", base::trans_odom2map_);      // 发布坐标变换
                    }
                    return; 
                } 
                boost::unique_lock<boost::shared_mutex> lock(base::keyframe_queue_sm_);
                // 将输入的数据放入待处理队列 
                KeyFrame keyframe(lidar_data.time_stamp_, odom);
                keyframe.between_constraint_ = between_constraint;      // 获取该关键帧与上一关键帧之间的相对约束  last<-curr 
                base::new_keyframe_queue_.push_back(keyframe);     
                base::new_keyframe_points_queue_.push_back(lidar_data.pointcloud_data_); 
                // 如果是建图阶段   则需要发布图优化相关数据   供其他模块使用    
                if (work_mode != MAPPING) return;  
                KeyFrameInfo<_FeatureT> keyframe_info; 
                keyframe_info.time_stamps_ = keyframe.time_stamp_;  
                keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetAllVertex(); 
                keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetAllEdge(); 
                keyframe_info.new_keyframes_ = base::new_keyframe_queue_;  
                DataManager::GetInstance().AddData("keyframes_info", std::move(keyframe_info));   // 发布图关键帧  
                DataManager::GetInstance().AddData("odom_to_map", base::trans_odom2map_);      // 发布坐标变换
            }

            // 回环模块在system中初始化  再设置进来  
            void SetLoopDetection(std::shared_ptr<loopDetection<_FeatureT>> const& loop_detect) override {
                loop_detect_ = loop_detect;  
            }

            // 强制执行一次全局优化   save的时候用
            void ForceGlobalOptimaze() override {
                optimizer_->Optimize();  
            }

        protected:
            /**
             * @brief: 建图模式下将数据传输到其他模块
             * @param keyframe 关键帧数据
             * @param pointcloud_data 关键帧对应的点云
             */            
            void DataFurtherProcess(uint64_t const& id, FeaturePointCloudContainer<_FeatureT> pointcloud_data)
            {
                // 把关键帧点云存储到硬盘里     不消耗内存
                for (auto iter = pointcloud_data.begin(); iter != pointcloud_data.end(); ++iter) 
                {  // 存到数据库中  
                    PoseGraphDataBase::GetInstance().AddKeyFramePointCloud(iter->first, 
                        id, *(iter->second));
                }
                // 点云数据加入到回环模块进行处理
                loop_detect_->AddData(pointcloud_data);    // 点云
                std::cout<<common::YELLOW<<"DataFurtherProcess done"<<common::RESET<<std::endl;
            }
            
            /**
             * @brief: 定位线程
             * @details 1、通过位姿点云查找距离当前帧最近的历史关键帧
             *                      2、通过广度优先搜索查找节点的相邻帧，并拼接成local map
             *                      3、执行scan-map匹配，求取odom-map校正矩阵         
             *                      4、匹配评估，并根据结果进行模式切换
             */            
            void localization()
            {
                while(1)
                {
                    // if (work_mode != LOCALIZATION) return;  
                    if (work_mode != LOCALIZATION) {
                        std::chrono::milliseconds dura(100);
                        std::this_thread::sleep_for(dura);
                        continue;  
                    }
                    base::keyframe_queue_sm_.lock_shared();
                    assert(loop_detect_ != nullptr);    // 检查回环模块是否被初始化
                    assert(base::new_keyframe_queue_.size() == base::new_keyframe_points_queue_.size()); 
                    // 如果没有新的关键帧  
                    if (base::new_keyframe_queue_.empty()) 
                    {
                        base::keyframe_queue_sm_.unlock_shared();
                        std::chrono::milliseconds dura(10);
                        std::this_thread::sleep_for(dura);
                        continue;  
                    }
                    std::cout<<common::GREEN<<"-----------------LOCALIZATION!-----------------"<<std::endl;
                    // 取出最早的帧  进行map匹配
                    KeyFrame& keyframe = base::new_keyframe_queue_.front(); 
                    FeaturePointCloudContainer<_FeatureT>& points = base::new_keyframe_points_queue_.front();
                    base::keyframe_queue_sm_.unlock_shared();
                    TicToc tt;  
                    // std::cout<<common::GREEN<<" keyframe.odom_: "<< keyframe.odom_.matrix()<<std::endl;
                    Eigen::Isometry3d pose_in_map = base::trans_odom2map_ * keyframe.odom_;  
                    // std::cout<<common::RED<<"base::trans_odom2map_: "<<base::trans_odom2map_.matrix()<<std::endl;
                    // std::cout<<common::GREEN<<"before loc pose_in_map: "<<pose_in_map.matrix()<<std::endl;
                    pcl::PointXYZ curr_pos(pose_in_map.translation().x(), 
                                                                    pose_in_map.translation().y(), 
                                                                    pose_in_map.translation().z());
                    std::vector<int> search_ind;
                    std::vector<float> search_dis;
                    loop_detect_->HistoricalPositionSearch(curr_pos, 0, 10, search_ind, search_dis);   // 搜索最近的历史关键帧
                    //std::cout<<"near node: "<<search_ind[0]<<std::endl;
                    /**
                     * @todo 如果啥都搜不到呢？
                     */                    
                    if (!search_ind.empty()) 
                    {
                        LocalizationPointsInfo<_FeatureT> loc_points;  
                        // 将定位所需要的点云local map 提取出来 
                        for (auto const& name : localize_registration_->GetUsedPointsName())
                        {   // 从数据库中查找 名字为 name 的点云 
                            typename pcl::PointCloud<_FeatureT>::Ptr local_map(new pcl::PointCloud<_FeatureT>());

                            for (int i = 0; i < search_ind.size(); i++)
                            {
                                typename pcl::PointCloud<_FeatureT>::Ptr origin_points(new pcl::PointCloud<_FeatureT>());
                                if (!PoseGraphDataBase::GetInstance().GetKeyFramePointCloud<_FeatureT>(name, 
                                        search_ind[i], origin_points))
                                {
                                    std::cout<<common::RED<<"错误: 找不到定位用的地图 "
                                    <<name<<common::RESET<<std::endl;
                                    throw std::bad_exception();  
                                }
                                // 读取节点的位姿
                                pcl::PointCloud<_FeatureT> trans_points;   // 转换到世界坐标系下的点云 
                                Eigen::Isometry3d pose;
                                PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[i], pose);
                                pcl::transformPointCloud(*origin_points, trans_points, pose.matrix()); // 转到世界坐标  
                                *local_map += trans_points; 
                            }
                            localize_registration_->SetInputSource(std::make_pair(name, local_map)); 
                            loc_points.map_[name] = local_map; 
                            //loc_points.scan_[name] =  points.at(name); 
                        }
                        loc_points.time_stamps_ = keyframe.time_stamp_; 
                        DataManager::GetInstance().AddData("loc_points", loc_points);      // 发布定位map 
                        localize_registration_->SetInputTarget(points);
                        if (!localize_registration_->Registration(pose_in_map)) {
                            std::cout<<common::RED<<"错误: 定位匹配无法收敛！转换到重定位模式..."
                            <<common::RESET<<std::endl;
                            work_mode = RELOCALIZATION;
                            continue;  
                        }
                        //std::cout<<common::GREEN<<"after loc pose_in_map: "<<pose_in_map.matrix()<<std::endl;
                        tt.toc("localization ");
                        tt.tic(); 
                        // 匹配评估
                        typename pcl::PointCloud<_FeatureT>::ConstPtr eva_local_map(new pcl::PointCloud<_FeatureT>());
                        std::string required_name = align_evaluator_.GetTargetName();    // 获取检验模块需要的点云标识名
                        if (loc_points.map_.find(required_name) != loc_points.map_.end()) {
                            eva_local_map = loc_points.map_[required_name];  
                        } else {  
                            typename pcl::PointCloud<_FeatureT>::Ptr local_map(new pcl::PointCloud<_FeatureT>());
                            for (int i = 0; i < search_ind.size(); i++)
                            {
                                typename pcl::PointCloud<_FeatureT>::Ptr origin_points(new pcl::PointCloud<_FeatureT>());
                                if (!PoseGraphDataBase::GetInstance().GetKeyFramePointCloud<_FeatureT>(required_name, 
                                    search_ind[i], origin_points))
                                {
                                    std::cout<<common::RED<<"错误：定位模式找不到evaluate map"<<common::RESET<<std::endl;
                                    throw std::bad_exception();  
                                }
                                // 读取节点的位姿
                                pcl::PointCloud<_FeatureT> trans_points;   // 转换到世界坐标系下的点云 
                                Eigen::Isometry3d pose;
                                PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[i], pose);
                                pcl::transformPointCloud(*origin_points, trans_points, pose.matrix()); // 转到世界坐标  
                                *local_map += trans_points; 
                            }
                            eva_local_map = local_map;
                        }
                        align_evaluator_.SetTargetPoints(eva_local_map);      // 0-15ms 
                        std::pair<double, double> res = align_evaluator_.AlignmentScore(points.at(required_name), 
                                                                                            pose_in_map.matrix().cast<float>(), 0.1, 0.5); // 0-10ms 
                        tt.toc("evaluate ");                                                                                                          
                        std::cout<<"score: "<<res.first<<std::endl;
                        std::cout<<"overlap_ratio: "<<res.second<<std::endl;
                        // 若重叠率很低 < 0.5，得分会 远远大于1， 那么认为定位丢失，此时进行重定位
                        if (res.first > 1) // 进行重定位
                        {
                            work_mode = RELOCALIZATION;
                            #if (debug == 1)
                                pcl::PointCloud<_FeatureT> input_transformed;
                                // cloud 通过  relpose 转到  input_transformed  
                                pcl::transformPointCloud (*points.at(required_name), 
                                                                                        input_transformed, 
                                                                                        pose_in_map.matrix().cast<float>());
                                static uint16_t ind = 0; 
                                typename pcl::PointCloud<_FeatureT>::Ptr res_points(new pcl::PointCloud<_FeatureT>());
                                *res_points = *eva_local_map;
                                 *res_points += input_transformed; 
                                pcl::io::savePCDFileBinary(
                                    "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_all.pcd"
                                    , *res_points);
                                pcl::io::savePCDFileBinary(
                                    "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_map.pcd"
                                    , *eva_local_map);
                                pcl::io::savePCDFileBinary(
                                    "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_scan.pcd"
                                    , *points.at(required_name));
                                pcl::io::savePCDFileBinary(
                                    "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_scan_trans.pcd"
                                    , input_transformed);
                            #endif
                            continue;  
                        }
                        // 如果得分很低 <= 0.04  认为定位很好
                        // 同时 若重叠率 < 0.9 且 > 0.5，则环境有变化，此时进行地图更新
                        if (res.first <= 0.04 && res.second < 0.9 && res.second > 0.5)
                        {   // 地图更新 
                            if (enable_map_update_) 
                            {
                                std::cout<<common::GREEN<<"-----------------MAP UDAPATE!-----------------"<<std::endl;
                                Eigen::Isometry3d front_pose; 
                                PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[0], front_pose); 
                                Eigen::Isometry3d relpose = front_pose.inverse() * pose_in_map; 
                                // 重新设置该关键帧的连接关系  与 约束 
                                keyframe.adjacent_id_ = search_ind[0];  
                                keyframe.between_constraint_ = relpose;      // 获取该关键帧与上一关键帧之间的相对约束  last<-curr       
                                work_mode = MAPPING;
                                //return;  // 退出线程
                                base::trans_odom2map_ = pose_in_map * keyframe.odom_.inverse();  
                                //std::cout<<common::RED<<"update base::trans_odom2map_: "<<base::trans_odom2map_.matrix()<<std::endl;
                                DataManager::GetInstance().AddData("odom_to_map", base::trans_odom2map_);      // 发布坐标变换
                                continue; 
                            }
                        } 
                        base::trans_odom2map_ = pose_in_map * keyframe.odom_.inverse();  
                        //std::cout<<common::RED<<"update base::trans_odom2map_: "<<base::trans_odom2map_.matrix()<<std::endl;
                        DataManager::GetInstance().AddData("odom_to_map", base::trans_odom2map_);      // 发布坐标变换
                        base::keyframe_queue_sm_.lock();
                        base::new_keyframe_queue_.pop_front(); 
                        base::new_keyframe_points_queue_.pop_front();
                        base::keyframe_queue_sm_.unlock(); 
                    } 
                    std::chrono::milliseconds dura(10);
                    std::this_thread::sleep_for(dura);
                }
            }

            /**
             * @brief: 后端建图线程  
             */            
            void mapping() override
            {
                while(true)
                {
                    //if (work_mode != MAPPING) return;  
                    if (work_mode != MAPPING) {
                        std::chrono::milliseconds dura(1000);
                        std::this_thread::sleep_for(dura);
                        continue;  
                    }
                    // 数据处理 
                    if (!processData()) 
                    {
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
                        base::keyframe_queue_sm_.lock();  
                        // 计算坐标转换矩阵
                        base::trans_odom2map_ = database.GetLastVertex().pose_ 
                                                                                  * database.GetLastKeyFrameData().odom_.inverse();  
                        base::keyframe_queue_sm_.unlock();  
                        if (has_loop_) 
                        {
                            work_mode = LOCALIZATION; // 进入定位模式 
                            // localization_thread_ = std::thread(&LifeLongBackEndOptimization::localization, this);  
                            has_loop_ = false;  
                        }
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
                boost::unique_lock<boost::shared_mutex> lock(base::keyframe_queue_sm_);
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
                    auto& points = base::new_keyframe_points_queue_[i];  
                    keyframe.id_ = PoseGraphDataBase::GetInstance().ReadVertexNum();
                    Eigen::Isometry3d corrected_pose = base::trans_odom2map_ * keyframe.odom_; 
                    DataFurtherProcess(keyframe.id_, points);
                    // 添加节点  
                    if (keyframe.id_ == 0) {
                        // 第一个节点默认 fix
                        optimizer_->AddSe3Node(corrected_pose, keyframe.id_, true);
                        //  添加到数据库中   图优化中的node 和 数据库中的关键帧 序号是一一对应的
                        PoseGraphDataBase::GetInstance().AddKeyFrameData(keyframe);   
                        PoseGraphDataBase::GetInstance().AddVertex(keyframe.id_, corrected_pose);  
                        PoseGraphDataBase::GetInstance().AddPosePoint(corrected_pose);  
                        continue;
                    }
                    optimizer_->AddSe3Node(corrected_pose, keyframe.id_); 
                    // 观测噪声
                    Eigen::Matrix<double, 1, 6> noise;
                    noise << 0.0025, 0.0025, 0.0025, 0.0001, 0.0001, 0.0001;
                    // 如果邻接节点id 没有被设置  说明就是与上一个节点连接 
                    if (keyframe.adjacent_id_ == -1) {
                        keyframe.adjacent_id_ = keyframe.id_ - 1;  
                    }
                    optimizer_->AddSe3Edge(keyframe.adjacent_id_, keyframe.id_, keyframe.between_constraint_, noise);  
                    PoseGraphDataBase::GetInstance().AddEdge(keyframe.adjacent_id_, keyframe.id_, 
                                                                                                                  keyframe.between_constraint_, noise);  

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
                    PoseGraphDataBase::GetInstance().AddVertex(keyframe.id_, corrected_pose);  
                    PoseGraphDataBase::GetInstance().AddPosePoint(corrected_pose);  
                }

                base::new_keyframe_queue_.erase(base::new_keyframe_queue_.begin(), 
                    base::new_keyframe_queue_.begin() + num_processed);     //  [first,last) 
                base::new_keyframe_points_queue_.erase(base::new_keyframe_points_queue_.begin(), 
                    base::new_keyframe_points_queue_.begin() + num_processed);     //  [first,last) 
                
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
                // 提取所有新添加的回环数据
                std::deque<Edge>  new_loops = loop_detect_->GetNewLoops();
                for (uint16_t i = 0; i < new_loops.size(); i++) 
                {
                    // 添加回环边
                    optimizer_->AddSe3Edge(new_loops[i].link_id_.first, new_loops[i].link_id_.second, 
                                                                            new_loops[i].constraint_, new_loops[i].noise_);  
                    // 回环数据记录到数据库中
                    PoseGraphDataBase::GetInstance().AddEdge(new_loops[i]); 
                    has_loop_ = true;   
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
                    if (!do_optimize && !has_loop_) return false;  
                    optimizer_->Optimize(has_loop_);  
                    tt.toc("optimize ");
                    do_optimize = false;  
                } else {
                    optimize_dormant--;  
                    return false;  
                 }
                return true;  
            }
    }; // class 
} // namespace 
