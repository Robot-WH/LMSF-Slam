/**
 * @Author: lwh
 * @Version: 1.0
 * @Date: 
 * @Description: 保存位姿图的数据库管理模块 
 * @Others: 
 */

#pragma once 

#include <boost/thread/thread.hpp>
#include <boost/thread/shared_mutex.hpp> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "Common/keyframe.hpp"
#include "Common/type.h"
#include "Common/keyframe.hpp"
#include "graph.hpp"

namespace Slam3D
{
    /**
     * @brief: 维护 pose graph 的数据 
     */    
    class PoseGraphDataBase
    {
        public:
            /**
             * @brief: 单例的创建函数  
             */            
            static PoseGraphDataBase& GetInstance()
            {
                static PoseGraphDataBase PoseGraph_dataBase; 
                return PoseGraph_dataBase; 
            }
            /**
             * @brief: 保存数据
             */            
            bool Save()
            {   // 保存keyframe信息 
                assert(database_save_path_ != ""); 
                for (uint64_t i = 0; i < keyframe_database_.size(); i++) {
                    keyframe_database_[i].Save(database_save_path_);  
                }
                // 保存pose-graph
                // 节点
                for (uint64_t i = 0; i < vertex_container_.size(); i++) {
                    vertex_container_[i].Save(database_save_path_);  
                }
                // 边
                for (uint64_t i = 0; i < edge_container_.size(); i++) {
                    edge_container_[i].Save(database_save_path_);  
                }
                // 保存位姿点云
                std::string file_path = database_save_path_ + "/Position3D.pcd";
                pcl::io::savePCDFileBinary(file_path, *cloudKeyFramePosition3D_);
                file_path = database_save_path_ + "/Rot3D.pcd";
                pcl::io::savePCDFileBinary(file_path, *cloudKeyFrameRot3D_);
            }
            /**
             * @brief: 从指定路径中加载数据库  
             */            
            bool Load()
            {
                assert(database_save_path_ != ""); 
                if (!LoadKeyFrame()) {
                    return false;
                }
                if (!LoadPoseGraph()) 
                {
                    DataReset(); 
                    return false; 
                }
                // 加载姿态点云
                std::string file_path = database_save_path_ +"/Position3D.pcd";
                if (pcl::io::loadPCDFile(file_path, *cloudKeyFramePosition3D_) < 0) {
                    std::cout<<common::RED<<"Load Position3D ERROR"<<std::endl;
                    DataReset(); 
                    return false; 
                }
                file_path = database_save_path_ +"/Rot3D.pcd";
                if (pcl::io::loadPCDFile(file_path, *cloudKeyFrameRot3D_) < 0) {
                    std::cout<<common::RED<<"Load Rot3D ERROR"<<std::endl;
                    DataReset();
                    return false;
                } 
                return true; 
            }

            bool LoadKeyFrame()
            {
                uint64_t index = 0; 
                // 加载keyframe数据
                while(1) 
                {
                    KeyFrame keyframe;  
                    std::ifstream ifs(database_save_path_ + "/KeyFrameData/data_" + std::to_string(index));
                    if(!ifs) {
                        break;
                    }
                    index++;  
                    while(!ifs.eof()) 
                    {
                        std::string token;
                        ifs >> token;
                        if(token == "stamp") 
                        {
                            ifs >> keyframe.time_stamp_; 
                        } 
                        else if (token == "id")
                        {
                            ifs >> keyframe.id_; 
                        }        
                        else if (token == "odom")
                        {
                            Eigen::Matrix4d matrix; 
                            for(int i = 0; i < 4; i++) {
                                for(int j = 0; j < 4; j++) {
                                    ifs >> matrix(i, j);
                                }
                            }
                            keyframe.odom_.translation() = matrix.block<3, 1>(0, 3);
                            keyframe.odom_.linear() = matrix.block<3, 3>(0, 0);
                        }
                        else if (token == "utm_coord")
                        {
                        }
                        else if (token == "floor_coeffs")
                        {
                        }
                        else if (token == "acceleration")
                        {
                        }
                        else if (token == "orientation")
                        {
                        }
                    }
                    keyframe_database_.push_back(std::move(keyframe));
                }
                if (index == 0) return false;
                std::cout<<common::GREEN<<"Load KetFrame done, num: "<<
                keyframe_database_.size()<<std::endl;
                return true; 
            }

            bool LoadPoseGraph()
            {
                uint64_t index = 0; 
                // 加载vertex数据
                while(1) 
                {
                    Vertex vertex;  
                    std::ifstream ifs(database_save_path_ + "/PoseGraph/Vertex/id_" + std::to_string(index));
                    if(!ifs) {
                        break;
                    }
                    index++;  
                    while(!ifs.eof()) 
                    {
                        std::string token;
                        ifs >> token;
                        if (token == "id")
                        {
                            ifs >> vertex.id_; 
                            //std::cout<<"vertex.id: "<<vertex.id_<<std::endl;
                        }        
                        else if (token == "pose")
                        {
                            Eigen::Matrix4d matrix; 
                            for(int i = 0; i < 4; i++) {
                                for(int j = 0; j < 4; j++) {
                                    ifs >> matrix(i, j);
                                }
                            }
                            vertex.pose_.translation() = matrix.block<3, 1>(0, 3);
                            vertex.pose_.linear() = matrix.block<3, 3>(0, 0);
                            //std::cout<<"vertex.pose_: "<<vertex.pose_.matrix()<<std::endl;
                        }
                    }
                    vertex_container_.push_back(std::move(vertex));
                }
                if (vertex_container_.size() != keyframe_database_.size()) {
                    std::cout<<common::RED<<
                    "Load KetFrame ERROR: vertex num keyframe num not match,  vertex num: "
                    <<vertex_container_.size()<<", keyframe num: "<<
                    keyframe_database_.size()<<std::endl;
                    return false;  
                }
                index = 0;  
                // 加载edge数据
                while(1) 
                {
                    Edge edge;  
                    std::ifstream ifs(database_save_path_ + "/PoseGraph/Edge/id_" + std::to_string(index));
                    if(!ifs) {
                        break;
                    }
                    index++;  
                    while(!ifs.eof()) 
                    {
                        std::string token;
                        ifs >> token;
                        if (token == "id")
                        {
                            ifs >> edge.id_; 
                            //std::cout<<"edge.id: "<<edge.id_<<std::endl;
                        }        
                        else if (token == "link_head")
                        {
                            ifs >> edge.link_id_.first; 
                            //std::cout<<"link_head: "<<edge.link_id_.first<<std::endl;
                        }
                        else if (token == "link_tail")
                        {
                            ifs >> edge.link_id_.second; 
                            //std::cout<<"link_tail: "<<edge.link_id_.second<<std::endl;
                        }
                        else if (token == "constraint")
                        {
                            Eigen::Matrix4d matrix; 
                            for(int i = 0; i < 4; i++) {
                                for(int j = 0; j < 4; j++) {
                                    ifs >> matrix(i, j);
                                }
                            }
                            edge.constraint_.translation() = matrix.block<3, 1>(0, 3);
                            edge.constraint_.linear() = matrix.block<3, 3>(0, 0);
                            //std::cout<<"edge.constraint_: "<<edge.constraint_.matrix()<<std::endl;
                        }
                        else if (token == "noise")
                        {
                            for(int i = 0; i < 6; i++) {
                                    ifs >> edge.noise_(0, i);
                            }
                            //std::cout<<"edge.noise_: "<<edge.noise_.matrix()<<std::endl;
                        }
                    }
                    edge_container_.push_back(std::move(edge));
                }
                std::cout<<common::GREEN<<"Load Pose-Graph done"<<std::endl;
                return true; 
            }

            void SetSavePath(string const& database_save_path) {
                database_save_path_ = database_save_path; 
            }
            // 添加一个关键帧的数据
            void AddKeyFrameData(KeyFrame const& keyframe)
            {
                // 将关键帧数据保存 
                database_mt_.lock();  
                keyframe_database_.push_back(keyframe);  
                database_mt_.unlock();  
                has_new_keyframe_ = true; 
            }
            // 添加一个位姿图节点
            void AddVertex(uint64_t id, Eigen::Isometry3d const& pose) {
                vertex_container_.emplace_back(id, pose);
            }
            // 添加一个位姿到位姿点云 
            void AddPosePoint(Eigen::Isometry3d const& pose) 
            {
                pose_cloud_mt_.lock();  
                //  将位置数据保存到位置点云
                pcl::PointXYZ Position3D;
                Position3D.x = pose.translation().x();
                Position3D.y = pose.translation().y();
                Position3D.z = pose.translation().z();
                cloudKeyFramePosition3D_->push_back(Position3D);
                // 保存姿态数据到位姿点云
                pcl::PointXYZI Rot3D; 
                Eigen::Quaterniond q(pose.rotation());
                Rot3D.x = q.x();
                Rot3D.y = q.y();
                Rot3D.z = q.z();
                Rot3D.intensity = q.w();  
                cloudKeyFrameRot3D_->push_back(Rot3D);
                pose_cloud_mt_.unlock();  
            }

            // 添加一个位姿图边 
            inline void AddEdge(uint64_t head_id, uint64_t tail_id, Eigen::Isometry3d const& constraint, 
                                            Eigen::Matrix<double, 1, 6> const& noise)
            {
                uint64_t id = edge_container_.size();  // 边的标识 
                edge_container_.emplace_back(id, head_id, tail_id, constraint, noise); 
            }
            // 重载
            inline void AddEdge(Edge &edge)
            {
                edge.id_ = edge_container_.size(); 
                edge_container_.push_back(edge); 
            }

            // 读取当前posegraph中节点的数量 
            uint64_t ReadVertexNum() 
            {
                database_mt_.lock_shared();
                uint64_t num = vertex_container_.size();
                database_mt_.unlock_shared();
                return num;  
            }

            // 添加一个关键帧的点云数据 
            template<typename _PointT>
            void AddKeyFramePointCloud(std::string const& name, 
                                                                            uint32_t const& KF_index, 
                                                                            pcl::PointCloud<_PointT> const& pointcloud)
            {
                std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                    + name + std::to_string(KF_index) + ".pcd";
                pcl::io::savePCDFileBinary(file_path, pointcloud);
            }

            // 获取最后一个节点
            Vertex GetLastVertex() 
            {
                Vertex vertex;  
                boost::shared_lock<boost::shared_mutex> lock(database_mt_); 
                if (vertex_container_.empty()) return vertex;
                vertex = vertex_container_.back();  
                return vertex;  
            }

            // 获取最后一个关键帧数据
            KeyFrame GetLastKeyFrameData() 
            {
                KeyFrame keyframe;  
                boost::shared_lock<boost::shared_mutex> lock(database_mt_); 
                if (keyframe_database_.empty()) return keyframe;
                keyframe = keyframe_database_.back();  
                return keyframe;  
            }

            // 获取最新添加的keyframe 
            bool GetNewKeyFrame(KeyFrame &keyframe)
            {
                if (!has_new_keyframe_) return false;  
                boost::shared_lock<boost::shared_mutex> lock(database_mt_); 
                keyframe = keyframe_database_.back();  
                has_new_keyframe_ = false; 
                return true;  
            }

            std::deque<KeyFrame> const& GetKeyFrameDataBase() {
                return keyframe_database_;
            }

            std::deque<Vertex> const& GetAllVertex() {
                return vertex_container_;
            }

            std::deque<Edge> const& GetAllEdge() {
                return edge_container_;
            }
            // 获取位置点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr GetKeyFramePositionCloud() 
            {
                boost::shared_lock<boost::shared_mutex> lock(pose_cloud_mt_); 
                return pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>(*cloudKeyFramePosition3D_));
            }
            // 获取姿态点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr GetKeyFrameRotCloud()
            {
                boost::shared_lock<boost::shared_mutex> lock(pose_cloud_mt_); 
                return pcl::PointCloud<pcl::PointXYZI>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZI>(*cloudKeyFrameRot3D_));
            }

            /**
             * @brief: 获取以一个节点为中心，前后若干个连续相邻节点共同组成的local map 
             * @param center_id localmap的中心节点 id 
             * @param neighbors_num 中心节点前/后 邻居个数 
             * @param points_name 点云的标识名
             * @param[out] local_map 构造的local map 
             * @return 是否成功
             */            
            template<typename _PointT>
            bool GetAdjacentLinkNodeLocalMap(uint32_t const& center_id, uint16_t const& neighbors_num, 
                                                                                        std::string const& points_name, 
                                                                                        typename pcl::PointCloud<_PointT>::Ptr &local_map)
            {
                pcl::PointCloud<_PointT> origin_points;   // 激光坐标系下的点云
                pcl::PointCloud<_PointT> trans_points;   // 转换到世界坐标系下的点云 
                for (int16_t i = -neighbors_num; i <= neighbors_num; i++ )
                {
                    if ((int64_t)center_id + i < 0) continue;  
                    std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                        + points_name + std::to_string(center_id + i) + ".pcd";
                    if (pcl::io::loadPCDFile(file_path, origin_points) < 0) return false; 
                    Eigen::Isometry3d pose;
                    // 如果没有查到该帧的位姿  那么就失败 
                    if (!SearchVertexPose(center_id + i, pose)) {
                        return false;  
                    }
                    pcl::transformPointCloud (origin_points, trans_points, pose.matrix()); // 转到世界坐标  
                    *local_map += trans_points; 
                }
                return true;  
            }

            /**
             * @brief: 获取keyframe 的一个 名字为 name 的点云数据 
             * @param name 点云名称
             * @param id 关键帧在数据库中的id
             * @param[out] curr_points 读取的结果 
             * @return 
             */            
            template<typename _PointT>
            bool GetKeyFramePointCloud(std::string const& name, uint32_t const& id, 
                typename pcl::PointCloud<_PointT>::Ptr &curr_points)
            {
                std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                        + name + std::to_string(id) + ".pcd";
                if (pcl::io::loadPCDFile(file_path, *curr_points) < 0) return false;
                return true;  
            }

            /**
             * @brief: 根据index获取vertex的pose 
             * @param index
             */            
            inline bool SearchVertexPose(uint32_t const& index, Eigen::Isometry3d &pose) const
            {
                if (vertex_container_.size() <= index) return false;
                pose = vertex_container_[index].pose_;
                return true;  
            }

            inline void UpdateVertexPose(uint32_t id, Eigen::Isometry3d correct_pose)
            {
                boost::unique_lock<boost::shared_mutex> lock1(database_mt_);    // 写锁 
                boost::unique_lock<boost::shared_mutex> lock2(pose_cloud_mt_);    // 写锁 
                vertex_container_[id].SetPose(correct_pose);  // 节点 位姿更新  
                // 位姿点云更新   
                cloudKeyFramePosition3D_->points[id].x = correct_pose.translation().x();
                cloudKeyFramePosition3D_->points[id].y = correct_pose.translation().y();
                cloudKeyFramePosition3D_->points[id].y = correct_pose.translation().y();
                
                Eigen::Quaterniond q(correct_pose.rotation());
                cloudKeyFrameRot3D_->points[id].x = q.x();
                cloudKeyFrameRot3D_->points[id].y = q.y();
                cloudKeyFrameRot3D_->points[id].z = q.z();
                cloudKeyFrameRot3D_->points[id].intensity = q.w();   
            }

        protected:
            PoseGraphDataBase() 
            {
                cloudKeyFramePosition3D_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>()
                ); 
                cloudKeyFrameRot3D_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZI>()
                ); 
                database_save_path_ = ""; 
            }
            PoseGraphDataBase(PoseGraphDataBase const& object) {}
            PoseGraphDataBase(PoseGraphDataBase&& object) {}

            void DataReset()
            {
                cloudKeyFramePosition3D_->clear();
                cloudKeyFrameRot3D_->clear();
                keyframe_database_.clear();
                edge_container_.clear();
                vertex_container_.clear(); 
            }

        private:
            bool has_new_keyframe_ = false; 
            std::string database_save_path_;  
            boost::shared_mutex pose_cloud_mt_, database_mt_, loop_mt_;  
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyFramePosition3D_;  // 历史关键帧位置
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyFrameRot3D_; // 历史关键帧姿态   四元数形式  
            std::deque<KeyFrame> keyframe_database_; // 保存全部关键帧的观测信息
            std::deque<Edge> edge_container_; // 保存图节点
            std::deque<Vertex> vertex_container_; // 保存图的边
    }; // class 
} // namespace 
