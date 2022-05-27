/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-05-12 17:01:11
 * @Description: 
 * @Others: 
 */

#include "BackEnd/GraphOptimization/graph_optimization_gtsam.hpp"
// #include "tic_toc.h"
#include "Common/color.hpp"

namespace Slam3D
{
    using namespace gtsam;  

    GtsamGraphOptimizer::GtsamGraphOptimizer()
    {
        // ISM2参数
        ISAM2Params parameters;
         // Only relinearize variables whose linear delta magnitude is greater than this threshold (default: 0.1). 
        parameters.relinearizeThreshold = 0.1; 
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);
        // 首节点  先验噪声
        Eigen::Matrix<double, 1, 6> noise;
        noise << 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001; 
        prior_noise_ = noise;  
    }

    void GtsamGraphOptimizer::Rebuild(std::deque<Vertex> const& vertexs, std::deque<Edge> const& edges)
    {  
    }

    /**
     * @brief: 
     * @details: 
     * @param flag 0 表示普通优化   1表示回环优化 
     */    
    bool GtsamGraphOptimizer::Optimize(uint8_t flag) 
    {
        // 执行优化
        // gtSAMgraph 只是保存了当前新添加的因子 
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        if (flag == 1)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
        gtSAMgraph.resize(0);
        initialEstimate.clear();
        isamCurrentEstimate = isam->calculateEstimate();
    }

    // 输出数据
    bool GtsamGraphOptimizer::GetAllOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose)
    {
    }

    Eigen::Isometry3d GtsamGraphOptimizer::ReadOptimizedPose(uint64_t const& id) 
    {
        assert(id < isamCurrentEstimate.size());
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = isamCurrentEstimate.at<Pose3>(id).translation();
        pose.linear() = isamCurrentEstimate.at<Pose3>(id).rotation().matrix(); 
        return pose;  
    }

    uint64_t GtsamGraphOptimizer::GetNodeNum() 
    {
        return isamCurrentEstimate.size();  
    }

    // 添加节点、边、鲁棒核
    void GtsamGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name, 
                                                                                                                double robust_kernel_size)
    {}

    gtsam::Pose3 GtsamGraphOptimizer::trans2gtsamPose(Eigen::Isometry3d const& pose)
    {
        return gtsam::Pose3(gtsam::Rot3(pose.rotation()), gtsam::Point3(pose.translation()));
    }

    void GtsamGraphOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, uint64_t const& id, bool need_fix)
    {
        // 如果是第一个节点   设置先验约束
        if (id == 0)
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(prior_noise_); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(pose), priorNoise));
        }
        initialEstimate.insert(id, trans2gtsamPose(pose));
    }

    /**
     * @brief: 添加se3 节点间的约束
     * @details: 
     * @param relative_pose T vertex_index1<-vertex_index2           poseindex1.inverse() * poseindex2
     * @return {*}
     */    
    void GtsamGraphOptimizer::AddSe3Edge(uint64_t vertex_index1,
                    uint64_t vertex_index2,
                    const Eigen::Isometry3d &relative_pose,
                    const Eigen::VectorXd noise) 
    {
        std::cout<<common::GREEN<<"AddSe3Edge  ....... "<<common::RESET<<std::endl;
        noiseModel::Diagonal::shared_ptr Noise = noiseModel::Diagonal::Variances(noise);
        // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(BetweenFactor<Pose3>(vertex_index1, vertex_index2, 
                                                                                                trans2gtsamPose(relative_pose),   
                                                                                                Noise));
        std::cout<<common::GREEN<<"AddSe3Edge  done....... "<<common::RESET<<std::endl;
    }

    void GtsamGraphOptimizer::AddSe3PriorXYZEdge(uint64_t se3_vertex_index,
                            const Eigen::Vector3d &xyz,
                            Eigen::VectorXd noise) 
    {
    }

    void GtsamGraphOptimizer::AddSe3PriorQuaternionEdge(uint64_t se3_vertex_index,
                                const Eigen::Quaterniond &quat,
                                Eigen::VectorXd noise) 
    {
    }

}