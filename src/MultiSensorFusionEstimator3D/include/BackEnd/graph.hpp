
/*
 * @Copyright(C): 
 * @FileName: 文件名
 * @Author: lwh
 * @Description: 
 * @Others: 
 */

#pragma once
#include <Eigen/Dense>

namespace Slam3D {

/**
 * @brief: pose-graph 顶点
 */
struct Vertex
{
    Vertex() : id_(-1), pose_(Eigen::Isometry3d::Identity()) {}
    Vertex(uint64_t const& id, Eigen::Isometry3d const& pose) : id_(id), pose_(pose) {}
    void SetPose(Eigen::Isometry3d const& pose) {
        pose_ = pose;  
    }
    uint64_t id_; 
    Eigen::Isometry3d pose_;
}; 

/**
 * @brief: pose-graph 边
 * @details: 边连接的节点idx，约束 + 协方差  
 */
struct Edge
{
    // 边类型 
    enum Type{
    };  
    Edge() {}
    Edge(uint64_t head_id, uint64_t tail_id, Eigen::Isometry3d const& constraint, 
                Eigen::Matrix<double, 1, 6> const& noise) : link_id_(head_id, tail_id), 
                                                                                                    constraint_(constraint), noise_(noise) {}

    std::pair<uint64_t, uint64_t> link_id_{-1, -1};     // 该边 连接的 节点 id 
    Eigen::Isometry3d constraint_;
    Eigen::Matrix<double, 1, 6> noise_;     // 6 dof 约束的 噪声 向量  xyz  + rpy
}; 
}