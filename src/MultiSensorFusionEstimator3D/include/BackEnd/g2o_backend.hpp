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
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "backend_base.hpp"

namespace Slam3D {

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
            using KeyFramePtr = typename KeyFrame<_FeatureT>::Ptr; 
            using Block = g2o::BlockSolver<g2o::BlockSolverTraits<6,-1>>;     // 重命名 块求解器     
            
            uint32_t KF_index_ = 0;  
            // 局部优化每次处理的最大帧数  
            int max_keyframes_per_update_ = 10;
            int planeConstraint_optimize_freq_ = 5;
            int sliding_windows_size_ = 50;  
            std::vector<KeyFramePtr> wait_optimize_keyframes_;
            std::vector<KeyFramePtr> wait_loopDetect_keyframes_;
            std::deque<KeyFramePtr> keyframe_database_;                            // 保存关键帧信息
            // 局部优化的滑动窗口 
            std::deque<KeyFramePtr> sliding_windows_;

            bool enable_GNSS_optimize_ = false;  
            bool enable_planeConstraint_optimize_ = false;

        public:
            // BackEndOptimizationG2O()
            // {
            //     //backend_thread_ = std::thread(&BackEndOptimizationG2O::process, this); 
            // }

            /**
             * @brief: 添加激光里程计数据  
             * @details: 
             */            
            void AddKeyFrame(FeatureInfo<_FeatureT> const& lidar_data, 
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
                std::cout<<common::GREEN<<"Backend -- AddKeyFrame()"
                    <<common::RESET<<std::endl;
                last_keyframe_t = lidar_data.time_stamp_; 
                // 线程锁开启
                std::lock_guard<std::mutex> lock(base::keyframe_queue_mutex_); 
                // 把关键帧点云存储到硬盘里     不消耗内存
                // 如果未来维护关键帧包括关键帧的删除或替换的话 , 那么 KF_index_ 的也需要去维护 
                for (auto iter = lidar_data.pointcloud_data_.begin(); 
                            iter != lidar_data.pointcloud_data_.end(); ++iter) 
                {  // 存到临时数据文件架中
                    std::string file_path = base::keyframes_save_path_ + "/temp/key_frame_" 
                        + iter->first + std::to_string(KF_index_) + ".pcd";
                    pcl::io::savePCDFileBinary(file_path, *(iter->second));
                }
                // 通过点云与里程计和累计距离等来创建关键帧   实际的关键帧中就不包含点云数据了  
                KeyFramePtr keyframe(new KeyFrame<_FeatureT>(lidar_data.time_stamp_, odom, KF_index_));
                keyframe->between_constraint_ = between_constraint;      // 获取该关键帧与上一关键帧之间的相对约束 
                // 获取在MAP系下的坐标   注意  这里只是为了可视化   在关键帧处理函数中会用最新的优化矩阵去转换 
                keyframe->correct_pose_ = base::trans_odom2map_ * keyframe->odom_;  
                base::new_keyframe_queue_.push_back(keyframe);     // 加入处理队列
                // KF index 更新  
                KF_index_++;
            }

        protected:

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            void process() override
            {
                while(true)
                {
                    {
                        std::lock_guard<std::mutex> lock(base::keyframe_queue_mutex_); 
                        // 数据处理 
                        dataParse();
                    }
                    // 进行优化 
                    std::cout<<"optimization ------------------------ begin "<<std::endl;

                    if (base::wait_optimize_keyframes_.empty())
                    {
                        return;  
                    }

                    std::cout<<"slidingWindowLocalOptimize ----------------------- begin "<<std::endl;
                    // 执行局部优化
                    localOptimize();
                    std::cout<<"slidingWindowLocalOptimize ----------------------- down "<<std::endl;
                    // 计算优化矩阵
                    //base::trans_odom2map = sliding_windows.back()->Pose * sliding_windows.back()->odom.inverse();  
                    // 如果有订阅者  发布odom到map坐标系的变换  
                    // if(odom2map_pub.getNumSubscribers()) 
                    // {
                    //     ROS_INFO_STREAM("BackEnd_node - trans_odom2map: "<<std::endl<<trans_odom2map.matrix());   
                    //     // 构造 ROS Msg
                    //     geometry_msgs::TransformStamped ts = matrix2transform(ros::Time::now(), trans_odom2map.matrix().cast<float>(), map_frame_id, odom_frame_id);
                    //     odom2map_pub.publish(ts);
                    // }

                    // if (SaveBackEndPath)
                    // {
                    //     // 保存局部优化的结果 
                    //     Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();  
                    //     for(auto const& frame : wait_optimize_keyframes_)
                    //     { 
                    //         if(frame->GNSS_Valid)
                    //         { 
                    //             gt_pose.block<3,1>(0, 3) = frame->utm_coord; 
                    //             gt_pose.block<3,3>(0, 0) =  frame->orientation.matrix(); 
                    //             SaveTrajectory(gt_pose, frame->Pose.matrix(), "/slam_data/trajectory", "/slam_data/trajectory/backend_ground_truth.txt", 
                    //                         "/slam_data/trajectory/backend_est_path.txt");
                    //         }
                    //     }
                    // }

                    // // 如果存在回环 
                    // if(loop)
                    // {
                    //   // 执行全局优化
                    //   globalOptimize(loop);
                    //   Loops.push_back(loop);
                    // }
                    
                    // // KeyFrameSnapshot 用于建图的关键帧节点  只有位姿与点云信息  
                    // std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframe_database_.size());
                    // std::transform(keyframe_database_.begin(), keyframe_database_.end(), snapshot.begin(),
                    //   [=](const KeyFrame::Ptr& k) {     
                    //     return std::make_shared<KeyFrameSnapshot>(k);     // 用 KeyFrame 指针 k 构造  KeyFrameSnapshot 
                    // });
                    // keyframes_snapshot_mutex.lock();
                    // keyframes_snapshot.swap(snapshot);
                    // keyframes_snapshot_mutex.unlock();
                    std::cout<<"optimization ------------------------ down "<<std::endl;
                    // 500ms的延时  
                    std::chrono::milliseconds dura(500);
                    std::this_thread::sleep_for(dura);
                    // }
                }
            }    

            /**
             * @brief: 
             * @details: 
             * @param {*}
             * @return {*}
             */            
            void dataParse() override
            {
                // 对新加入的关键帧进行处理      
                // 如果没有新的关键帧  
                if (base::new_keyframe_queue_.empty()) 
                {
                    return;
                }
                // 处理的数量  
                int num_processed = std::min<int>(base::new_keyframe_queue_.size(), 
                                                                                            max_keyframes_per_update_);
                int num = 0;
                uint16_t local_constraint_num = 0; 
                // 遍历全部关键帧队列       
                for (int i = 0; i < num_processed; i++) 
                {
                    // 从keyframe_queue中取出关键帧
                    const auto& keyframe = base::new_keyframe_queue_[i];
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
                            keyframe->planeConstraint_valid_ = true;  
                            planeConstraint_freq_count = planeConstraint_optimize_freq_;  
                            local_constraint_num++;
                        }
                        else 
                        {
                            planeConstraint_freq_count--;
                        }
                    }
                    // 放置到待优化容器中     
                    wait_optimize_keyframes_.push_back(keyframe);   
                    // 放置到待回环检测容器中
                    wait_loopDetect_keyframes_.push_back(keyframe); 
                    num++;  
                }

                base::new_keyframe_queue_.erase(base::new_keyframe_queue_.begin(), 
                    base::new_keyframe_queue_.begin() + num);     //  [first,last) 
                // 如果没有其他约束   那么没有优化的必要  
                if (local_constraint_num == 0)
                {
                    for (int i = 0; i < wait_optimize_keyframes_.size(); i++)
                    { 
                        keyframe_database_.push_back(wait_optimize_keyframes_[i]);   
                    }
                    wait_optimize_keyframes_.clear();
                }

                return;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            bool updataSlidingWindows()
            {
                if (wait_optimize_keyframes_.empty())
                {
                    return false;  
                }

                for (int i = 0; i < wait_optimize_keyframes_.size(); i++)
                { 
                    // 判断滑窗满了没   满了就要滑动  
                    if (sliding_windows_.size() >= sliding_windows_size_)
                    {
                        keyframe_database_.push_back(sliding_windows_.front());  
                        sliding_windows_.pop_front();
                        sliding_windows_.push_back(wait_optimize_keyframes_[i]);   
                    }
                    else
                    {
                        sliding_windows_.push_back(wait_optimize_keyframes_[i]);   
                    }
                }

                wait_optimize_keyframes_.clear();
                return true; 
            }

            /**
             * @brief: 
             * @details: 
             * @param {*}
             * @return {*}
             */            
            void localOptimize() override
            {
                // 更新滑动窗口 
                if (!updataSlidingWindows())
                    return;

                g2o::SparseOptimizer optimizer;   // 定义稀疏求解器  
                // 创建线性求解器
                std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
                // 定义 块求解器  
                std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
                g2o::OptimizationAlgorithmLevenberg* solver = 
                    new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));     // 构造优化方法  LM  
                optimizer.setAlgorithm(solver);            // 设置优化方法到 稀疏求解器
                // 记录本次局部优化中  节点与边的序号   
                int vertexCnt=1, edgeCnt=0;
                // 记录本次优化的全部节点  用于边寻找关联 节点    
                vector<g2o::VertexSE3*> vertexs;      
                // 遍历本次优化的全部关键帧  
                for (typename deque<KeyFramePtr>::iterator keyframe_it = sliding_windows_.begin(); 
                        keyframe_it != sliding_windows_.end(); ++keyframe_it)
                {    
                    // 添加节点  
                    g2o::VertexSE3* v = new g2o::VertexSE3();
                    v->setId(vertexCnt++);
                    // ROS_INFO_STREAM("setEstimate() : "<<keyframe->odom.matrix());
                    v->setEstimate((*keyframe_it)->correct_pose_);      // 设置先验位姿
                    optimizer.addVertex(v);     
                    vertexs.push_back(v);
                    
                    if (enable_GNSS_optimize_)
                    {   
                        // 判断是否执行GNSS优化
                        // 添加先验的边    每一条GNSS的边  残差维度 3   单边   只与一个节点状态有关  Jt(6*3)*W(3*3)*J(3*6)  
                        if ((*keyframe_it)->GNSS_valid_)
                        {
                            // g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
                            // edge->setMeasurement((*keyframe_it)->utm_coord);      // 设置观测先验  
                            // edge->vertices()[0] = v;
                            // // 信息矩阵     3*3      JtWJ  
                            // Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
                            // information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy * gps_edge_stddev_xy;     // 1
                            // information_matrix(2, 2) /= gps_edge_stddev_z * gps_edge_stddev_z;                  // 2  
                            // edge->setInformation(information_matrix);
                            // optimizer.addEdge(edge);
                            // //  optimizer.add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));
                        }
                    }

                    // 可以添加全局平面约束   
                    if(enable_planeConstraint_optimize_)
                    {   
                        if((*keyframe_it)->planeConstraint_valid_)
                        {

                        }
                    }      
                    // 处理本次局部优化的第一个节点  
                    if(vertexs.size()==1)
                    { 
                        // 历史上第一个节点  
                        if(keyframe_database_.empty())
                        {
                            v->setFixed(true);            // 直接fix 
                        }
                        else
                        {    
                            // 用之前一个关键帧为当前关键帧提供约束   类似于边缘化后的先验信息
                            g2o::VertexSE3* org = new g2o::VertexSE3();
                            org->setId(0);    
                            org->setEstimate(keyframe_database_.back()->correct_pose_);
                            org->setFixed(true);
                            optimizer.addVertex(org);
                            // 激光里程计的边  
                            g2o::EdgeSE3* edge(new g2o::EdgeSE3());
                            edge->setId(edgeCnt++);
                            edge->setVertex(0, org);
                            edge->setVertex(1, v);
                            // Tlw * Twc = Tlc   
                            edge->setMeasurement((*keyframe_it)->between_constraint_);
                            // 计算信息矩阵    
                            // 通过kdtree检查点云通过变换后的匹配程度反映里程计是否准确   匹配程度越高  则信息矩阵各权重越大   则优化时  会更靠近里程计的结果   
                            // Eigen::MatrixXd information = inf_calclator->calc_information_matrix( keyframe_database_.back()->cloud, (*keyframe_it)->cloud, relative_pose);
                            Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) / 0.01; 
                            edge->setInformation(information);
                            optimizer.addEdge(edge);
                        }
                        continue;
                    }
                    // 为除了第一个节点外的其他结点 添加激光里程计的边  
                    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
                    edge->setId(edgeCnt++);
                    edge->setVertex(0, vertexs[vertexCnt-3]);
                    edge->setVertex(1, vertexs[vertexCnt-2]);
                    // add edge between consecutive keyframe_database_
                    // 计算相对位移        Tlw * Twc = Tlc 
                    // 里程计约束  
                    edge->setMeasurement((*keyframe_it)->between_constraint_);
                    // 信息矩阵  里程计模块需要给出一个值   
                    // prev_keyframe->cloud = Tlc * keyframe->cloud
                    // Eigen::MatrixXd information = inf_calclator->calc_information_matrix((sliding_windows.at(vertexCnt-2))->cloud, (*keyframe_it)->cloud,  relative_pose);
                    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) / 0.005; 
                    edge->setInformation(information);
                    optimizer.addEdge(edge);
                }

                // optimize the pose graph
                // 执行优化
                std::cout << std::endl;
                std::cout << "--- local optimization ---" << std::endl;
                std::cout << "nodes: " << optimizer.vertices().size() << "   edges: " << optimizer.edges().size() << std::endl;
                std::cout << "optimizing... " << std::flush;

                optimizer.initializeOptimization(0);
                optimizer.setVerbose(true);

                std::cout << "chi2" << std::endl;
                double chi2 = optimizer.chi2();

                std::cout << "optimize!!" << std::endl;
                auto t1 = ros::WallTime::now();
                optimizer.optimize(30);

                auto t2 = ros::WallTime::now();
                std::cout << "done" << std::endl;
                std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer.chi2() << std::endl;
                std::cout << "time: " << (t2 - t1).toSec() << "[sec]" << std::endl;

                int i = 0;
                for (typename deque<KeyFramePtr>::iterator keyframe_it = sliding_windows_.begin(); 
                        keyframe_it != sliding_windows_.end(); ++keyframe_it)
                {
                    (*keyframe_it)->correct_pose_ = vertexs[i]->estimate();    //  获取优化结果
                    i++;  
                }
            }

            void globalOptimize() override
            {

            }



    };
}
