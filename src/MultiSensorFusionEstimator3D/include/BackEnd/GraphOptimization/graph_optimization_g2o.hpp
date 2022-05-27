/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-29 12:48:25
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "graph_optimization.hpp"

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
} // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);

// LinearSolverDense  使用dense cholesky分解法 
// LinearSolverEigen 稀疏cholesky法   效果和CSparse相似    
// 因为SLAM的优化问题一般都具有稀疏性，所以一般不用Dense方法
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

// namespace g2o {
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
// } // namespace g2o

namespace Slam3D 
{
    class G2oGraphOptimizer: public GraphOptimizerInterface 
    {
    public:
        G2oGraphOptimizer(const std::string &solver_type = "lm_var");
        void Rebuild(std::deque<Vertex> const& vertexs, std::deque<Edge> const& edges) override; 
        // 优化
        bool Optimize(uint8_t flag = 0) override;
        // 输出数据
        bool GetAllOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) override;
        Eigen::Isometry3d ReadOptimizedPose(uint64_t const& id) override;
        uint64_t GetNodeNum() override;
        // 添加节点、边、鲁棒核
        void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
        void AddSe3Node(const Eigen::Isometry3d &pose, uint64_t const& id, bool need_fix = false) override;
        void AddSe3Edge(uint64_t vertex_index1,
                        uint64_t vertex_index2,
                        const Eigen::Isometry3d &relative_pose,
                        const Eigen::VectorXd noise) override;
        void AddSe3PriorXYZEdge(uint64_t se3_vertex_index,
                                const Eigen::Vector3d &xyz,
                                Eigen::VectorXd noise) override;
        void AddSe3PriorQuaternionEdge(uint64_t se3_vertex_index,
                                    const Eigen::Quaterniond &quat,
                                    Eigen::VectorXd noise) override;

    private:
        Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
        Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise);
        Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
        void AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size);

    private:
        g2o::RobustKernelFactory *robust_kernel_factory_;
        std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

        std::string robust_kernel_name_;
        double robust_kernel_size_;
        bool need_robust_kernel_ = false;
    }; // class 
} // namespace Slam3D