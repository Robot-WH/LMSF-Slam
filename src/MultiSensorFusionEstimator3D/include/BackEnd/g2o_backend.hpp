/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-13 17:08:09
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_so3_prior.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
//#include "g2o/solvers/csparse/linear_solver_csparse.h"
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "backend_base.hpp"
#include "Common/data_manager.hpp"
#include "LoopDetection/loopDetection.hpp"

namespace Slam3D {

    using common::DataManager; 

    /**
     * @brief: 基于G2O的后端优化模块  
     * @details: 实现 
     * 1、融合激光里程计约束，平面先验约束，gnss 约束的滑动窗口优化
     * 2、回环之后进行全局优化  
     * @param {*}
     * @return {*}
     */    
    template<typename _FeatureT>
    class BackEndOptimizationG2O : public BackEndOptimizationBase<_FeatureT>
    {
        private:
            using base = BackEndOptimizationBase<_FeatureT>; 
            using KeyFramePtr = typename base::KeyFramePtr; 
            // 对于 单边  BlockSolverTraits<优化状态维度，残差维度>
            // 对于多边 BlockSolverTraits<各个node优化状态维度>
            // 这里由于存在多边也有单边， 所以 -1表示 自动去适配  
            using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;     // 重命名 块求解器
            // LinearSolverDense  使用dense cholesky分解法 
            // LinearSolverEigen 稀疏cholesky法   效果和CSparse相似    
            // 因为SLAM的优化问题一般都具有稀疏性，所以一般不用Dense方法 
            using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;    
                         
            uint32_t KF_index_ = 0;  
            // 局部优化每次处理的最大帧数  
            int max_keyframes_per_update_ = 10;
            int planeConstraint_optimize_freq_ = 5;
            std::vector<KeyFramePtr> wait_optimize_keyframes_;
            bool enable_GNSS_optimize_ = false;  
            bool enable_planeConstraint_optimize_ = false;
            // 回环模块 
            loopDetection<_FeatureT> loop_detect_;  
            // 后端处理线程
            std::thread backend_thread_;  
            g2o::SparseOptimizer optimizer_;
            uint32_t vertexCnt_ = 0;     // 图优化中节点的数量 
            uint32_t edgeCnt_ = 0; // 边的数量   
            std::deque<g2o::VertexSE3*> vertexs_;    
            uint16_t new_external_constranit_num_;  // 新加的外部约束数量(GPS，地面, IMU..)  决定是否要进行一次全局优化  

        public:
            BackEndOptimizationG2O()
            {
                // 图优化初始化
                // 第1步：创建一个线性求解器LinearSolver
                std::unique_ptr<LinearSolver> linear_solver(new LinearSolver());
                // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
                std::unique_ptr<BlockSolver> solver_ptr(new BlockSolver(std::move(linear_solver)));
                // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
                g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));     // 构造优化方法  LM  
                // 第4步：设置优化方法到 稀疏求解器（SparseOptimizer）
                optimizer_.setAlgorithm(solver);           

                backend_thread_ = std::thread(&BackEndOptimizationG2O::process, this);  // 启动线程   
                // 数据注册   
                DataManager::GetInstance().Registration<KeyFrameInfo<_FeatureT>>("keyframes_info", 1);
                DataManager::GetInstance().Registration<Eigen::Isometry3d>("odom_to_map", 1);
            }

            /**
             * @brief: 添加激光里程计数据  
             * @details: 
             * @param lidar_data 激光雷达的数据  
             * @param 
             */            
            void AddKeyFrame(CloudContainer<_FeatureT> const& lidar_data, 
                                                        Eigen::Isometry3d const& odom, 
                                                        Eigen::Isometry3d const& between_constraint) override
            {
                static double last_keyframe_t = -1;
                // 检查时间戳 看是否乱序    
                if (lidar_data.time_stamp_ <= last_keyframe_t)
                {
                    std::cout<<common::RED<<"Backend -- AddKeyFrame(): timestamp error"
                    <<common::RESET<<std::endl;
                    return;
                }
                // std::cout<<common::GREEN<<"Backend -- AddKeyFrame()"
                //     <<common::RESET<<std::endl;
                last_keyframe_t = lidar_data.time_stamp_; 
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
                // 获取在MAP系下的坐标   注意  这里只是为了可视化   在关键帧处理函数中会用最新的优化矩阵去转换         
                keyframe.correct_pose_ = base::trans_odom2map_ * keyframe.odom_;  
                base::new_keyframe_queue_.push_back(keyframe);     // 加入处理队列
                // 数据加入到回环模块进行处理
                loop_detect_.AddData(lidar_data.pointcloud_data_);    // 点云
                // 更新后端优化数据   用于可视化 
                KeyFrameInfo<_FeatureT> keyframe_info; 
                keyframe_info.time_stamps_ = lidar_data.time_stamp_;  
                keyframe_info.keyframe_database_ = PoseGraphDataBase::GetInstance().GetKeyFrameDataBase();
                keyframe_info.new_keyframes_ = base::new_keyframe_queue_;  
                keyframe_info.loops_ = PoseGraphDataBase::GetInstance().GetAllLoop();  
                DataManager::GetInstance().AddData("keyframes_info", std::move(keyframe_info));   // 发布图关键帧  
                DataManager::GetInstance().AddData("odom_to_map", base::trans_odom2map_);      // 发布坐标变换
                KF_index_++;
            }

        protected:

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            void process() override
            {
                while(true)
                {
                    // 数据处理 
                    processData();
                    PoseGraphDataBase& database = PoseGraphDataBase::GetInstance();  
                    if (optimize()) 
                    {
                        // 优化完成后 更新数据库  
                        for(int i=0; i<vertexs_.size(); i++)
                        {
                            database.ModifyOneKeyFramePose(i, vertexs_[i]->estimate()); 
                        }
                        base::keyframe_queue_mutex_.lock();  
                        // 计算坐标转换矩阵
                        base::trans_odom2map_ = database.GetLastKeyFrame().correct_pose_
                            * database.GetLastKeyFrame().odom_.inverse();  
                        
                        for(int i = 0; i < base::new_keyframe_queue_.size(); i++)
                        {
                            base::new_keyframe_queue_[i].correct_pose_ = 
                                base::trans_odom2map_ * base::new_keyframe_queue_[i].odom_;  
                        }
                        base::keyframe_queue_mutex_.unlock();  
                    }

                    std::chrono::milliseconds dura(1000);
                    std::this_thread::sleep_for(dura);
                }
            }    

            /**
             * @brief: 对于新加入的关键帧new_keyframe_queue_，对每一个关键帧匹配约束，
             * @details: 约束对应好后放入 wait_optimize_keyframes_ 容器
             */            
            void processData()
            {
                std::lock_guard<std::mutex> lock(base::keyframe_queue_mutex_); 
                // 对新加入的关键帧进行处理      
                // 如果没有新的关键帧  
                if (base::new_keyframe_queue_.empty()) {
                    return;
                }
                // 处理的数量  
                int num_processed = std::min<int>(base::new_keyframe_queue_.size(), 
                                                                                            max_keyframes_per_update_);
                // 遍历全部关键帧队列       
                for (int i = 0; i < num_processed; i++) 
                {
                    // 从keyframe_queue中取出关键帧
                    auto& keyframe = base::new_keyframe_queue_[i];
                    // 添加节点  
                    g2o::VertexSE3* v = new g2o::VertexSE3();
                    v->setId(vertexCnt_++);
                    v->setEstimate(keyframe.correct_pose_);      // 设置先验位姿
                    optimizer_.addVertex(v);     
                    vertexs_.push_back(v);    // 只有在外部保存起来  之后才能进行设置和修改  
                    // 处理优化的第一个节点  
                    if (vertexs_.size() == 1)
                    { 
                        v->setFixed(true);            // 直接fix 
                        //  添加到数据库中  
                        PoseGraphDataBase::GetInstance().AddKeyFrameData(keyframe);   
                        continue;  
                    }
                    // 为除了第一个节点外的其他结点 添加激光里程计的边  
                    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
                    edge->setId(edgeCnt_++);
                    edge->setVertex(0, vertexs_[vertexCnt_-2]);
                    edge->setVertex(1, vertexs_[vertexCnt_-1]);
                    // 计算相对位移        Tlw * Twc = Tlc 
                    // 里程计约束  
                    edge->setMeasurement(keyframe.between_constraint_);
                    // 信息矩阵  里程计模块需要给出一个值   
                    // prev_keyframe->cloud = Tlc * keyframe->cloud
                    // Eigen::MatrixXd information = inf_calclator->calc_information_matrix((sliding_windows.at(vertexCnt-2))->cloud, (*keyframe_it)->cloud,  relative_pose);
                    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) / 0.005; 
                    edge->setInformation(information);
                    optimizer_.addEdge(edge);

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
                    if (enable_planeConstraint_optimize_)
                    {
                        static int planeConstraint_freq_count = 0;
                        if (planeConstraint_freq_count <= 0) 
                        {
                            keyframe.planeConstraint_valid_ = true;  
                            planeConstraint_freq_count = planeConstraint_optimize_freq_;  
                        }
                        else 
                        {
                            planeConstraint_freq_count--;
                        }
                    }     
                    //  添加到数据库中  
                    PoseGraphDataBase::GetInstance().AddKeyFrameData(keyframe);   
                }

                base::new_keyframe_queue_.erase(base::new_keyframe_queue_.begin(), 
                    base::new_keyframe_queue_.begin() + num_processed);     //  [first,last) 
                
                return;
            }

            /**
             * @brief: 全局pose graph优化
             * @details: 
             * @return 是否进行了优化 
             */            
            bool optimize() override
            {
                static std::vector<g2o::VertexSE3*> temporary_fix_vertexs; 
                static bool do_optimize = false;  
                // 提取所有新添加的回环数据
                std::deque<Loop>  new_loops = loop_detect_.GetNewLoops();
                for (uint16_t i = 0; i < new_loops.size(); i++)
                {
                    // 添加回环边
                    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
                    edge->setId(edgeCnt_++);
                    edge->setVertex(0, vertexs_[new_loops[i].id_1_]);     // 相对早一点的
                    edge->setVertex(1, vertexs_[new_loops[i].id_2_]);     // 相对晚一点的
                    Eigen::Isometry3d relpose(new_loops[i].relative_pose_);    // relpose :  curr -> last  
                    edge->setMeasurement(relpose);
                    // 计算信息矩阵    维度与优化状态的维度相当   
                    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) / 0.001; 
                    edge->setInformation(information);
                    optimizer_.addEdge(edge);
                    vertexs_[new_loops[i].id_1_]->setFixed(true);            // 直接fix 
                    temporary_fix_vertexs.push_back(vertexs_[new_loops[i].id_1_]);  
                    // 回环数据记录到数据库中
                    PoseGraphDataBase::GetInstance().AddLoop(new_loops[i]); 
                    do_optimize = true;   // 只要有回环就触发优化
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
                    if (!do_optimize) return false;  
                    std::cout << std::endl;
                    std::cout << "--- global optimization ---" << std::endl;
                    std::cout << "nodes: " << optimizer_.vertices().size() << "   edges: " 
                                        << optimizer_.edges().size() << std::endl;
                    std::cout << "optimizing... " << std::flush;

                    std::cout << "init" << std::endl;
                    optimizer_.initializeOptimization(0);
                    optimizer_.setVerbose(true);

                    std::cout << "chi2" << std::endl;                   
                    double chi2 = optimizer_.chi2();

                    std::cout << "optimize!!" << std::endl;
                    int iterations = optimizer_.optimize(30);
                    std::cout << "done" << std::endl;
                    std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer_.chi2() << std::endl;
                    do_optimize = false;  
                    optimize_dormant = 10; 
                    for (auto &vertex : temporary_fix_vertexs)
                    {
                        vertex->setFixed(false);
                    }
                    temporary_fix_vertexs.clear();  
                } else {
                    optimize_dormant--;  
                 }

                return true;  
            }
    };
}
