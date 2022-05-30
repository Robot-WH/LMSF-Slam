/*
 * @Copyright(C): 
 * @Author: lwh
 * @Description: 
 * @Others: 
 */
#include <glog/logging.h>
#include "BackEnd/GraphOptimization/graph_optimization_g2o.hpp"
#include "tic_toc.h"

namespace Slam3D
{
    G2oGraphOptimizer::G2oGraphOptimizer(const std::string &solver_type) 
    {
        graph_ptr_.reset(new g2o::SparseOptimizer());

        g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
        g2o::OptimizationAlgorithmProperty solver_property;
        g2o::OptimizationAlgorithm *solver = solver_factory->construct(solver_type, solver_property);
        graph_ptr_->setAlgorithm(solver);

        if (!graph_ptr_->solver()) {
            LOG(ERROR) << "G2O 优化器创建失败！";
            throw std::bad_alloc();  
        }
        LOG(INFO) << "G2O 优化器创建成功！";
        robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
    }

    /**
     * @brief: 根据节点信息和边的信息重建位姿图   
     */    
    void G2oGraphOptimizer::Rebuild(std::deque<Vertex> const& vertexs, std::deque<Edge> const& edges)
    {
        bool is_first = true;  
        bool need_fix = true; 
        for (auto const& vertex : vertexs)
        {
            AddSe3Node(vertex.pose_, vertex.id_, need_fix); 
            if (is_first) 
            {
                is_first = false; 
                need_fix = false;  
            }
        }
        for (auto const& edge : edges) {
            AddSe3Edge(edge.link_id_.first, edge.link_id_.second, edge.constraint_, edge.noise_);  
        }   
        Optimize();
    }

    bool G2oGraphOptimizer::Optimize(uint8_t flag) 
    {
        if (graph_ptr_->edges().size() < 10) {
            return -1;
        }

        std::cout << std::endl;
        std::cout << "--- pose graph optimization ---" << std::endl;
        std::cout << "nodes: " << graph_ptr_->vertices().size() << "   edges: " << graph_ptr_->edges().size() << std::endl;
        std::cout << "optimizing... " << std::flush;

        std::cout << "init" << std::endl;
        graph_ptr_->initializeOptimization();
        graph_ptr_->setVerbose(true);

        std::cout << "chi2" << std::endl;
        double chi2 = graph_ptr_->chi2();

        std::cout << "optimize!!" << std::endl;
        TicToc optimize_time;
        int iterations = graph_ptr_->optimize(max_iterations_num_);

        optimize_time.toc("optimize_time ");
        std::cout << "done" << std::endl;
        std::cout << "iterations: " << iterations << " / " << max_iterations_num_ << std::endl;
        std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph_ptr_->chi2() << std::endl;

        return true;
    }

    bool G2oGraphOptimizer::GetAllOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) 
    {
        optimized_pose.clear();
        int vertex_num = graph_ptr_->vertices().size();

        for (int i = 0; i < vertex_num; i++) {
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(i));
            Eigen::Isometry3d pose = v->estimate();
            optimized_pose.push_back(pose.matrix().cast<float>());
        }
        return true;
    }

    Eigen::Isometry3d G2oGraphOptimizer::ReadOptimizedPose(uint64_t const& id) 
    {
        assert(id < graph_ptr_->vertices().size());
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(id));
        Eigen::Isometry3d pose = v->estimate();
        return pose;
    }

    uint64_t G2oGraphOptimizer::GetNodeNum() 
    {
        return graph_ptr_->vertices().size();
    }

    void G2oGraphOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, uint64_t const& id, bool need_fix) 
    {
        g2o::VertexSE3 *vertex(new g2o::VertexSE3());

        vertex->setId(id);   // 图中 每个节点的id 必须是唯一的  若id 与 之前的重复  则无法添加id 到 图中 
        vertex->setEstimate(pose);
        if (need_fix) {
            vertex->setFixed(true);
        }

        graph_ptr_->addVertex(vertex);
    }

    void G2oGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name,
            double robust_kernel_size) 
    {
        robust_kernel_name_ = robust_kernel_name;
        robust_kernel_size_ = robust_kernel_size;
        need_robust_kernel_ = true;
    }

    void G2oGraphOptimizer::AddSe3Edge(
        uint64_t vertex_index1,
        uint64_t vertex_index2,
        const Eigen::Isometry3d &relative_pose,
        const Eigen::VectorXd noise) 
    {
        Eigen::MatrixXd information_matrix = CalculateSe3EdgeInformationMatrix(noise);

        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index1));
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index2));
        
        g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
        edge->setMeasurement(relative_pose);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v1;
        edge->vertices()[1] = v2;
        graph_ptr_->addEdge(edge);
        if (need_robust_kernel_) {
            AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
        }
    }

    Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise) 
    {
        Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
        information_matrix = CalculateDiagMatrix(noise);
        return information_matrix;
    }

    void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge *edge, 
                                                                                                const std::string &kernel_type, 
                                                                                                double kernel_size) 
    {
        if (kernel_type == "NONE") {
            return;
        }

        g2o::RobustKernel *kernel = robust_kernel_factory_->construct(kernel_type);
        if (kernel == nullptr) {
            std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
            return;
        }

        kernel->setDelta(kernel_size);
        edge->setRobustKernel(kernel);
    }

    Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise) 
    {
        Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
        for (int i = 0; i < noise.rows(); i++) {
            information_matrix(i, i) /= noise(i);
        }
        return information_matrix;
    }

    /**
     * @brief: 位置先验约束 
     * @details 比如GPS
     * @param se3_vertex_index 添加约束的节点index 
     */    
    void G2oGraphOptimizer::AddSe3PriorXYZEdge(
        uint64_t se3_vertex_index,
        const Eigen::Vector3d &xyz,
        Eigen::VectorXd noise) 
    {
        // Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
        // g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
        // g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
        // edge->setMeasurement(xyz);
        // edge->setInformation(information_matrix);
        // edge->vertices()[0] = v_se3;
        // graph_ptr_->addEdge(edge);
    }

    /**
     * @brief: 姿态先验约束 
     * @details 比如惯导的姿态观测，地面的观测
     * @param se3_vertex_index 添加约束的节点index 
     */    
    void G2oGraphOptimizer::AddSe3PriorQuaternionEdge(uint64_t se3_vertex_index,
            const Eigen::Quaterniond &quat,
            Eigen::VectorXd noise) 
    {
        // Eigen::MatrixXd information_matrix = CalculateSe3PriorQuaternionEdgeInformationMatrix(noise);
        // g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
        // g2o::EdgeSE3PriorRot* edge(new g2o::EdgeSE3PriorRot());
        // edge->setMeasurement(quat);
        // edge->setInformation(information_matrix);
        // edge->vertices()[0] = v_se3;
        // graph_ptr_->addEdge(edge);
    }

    // TODO: 姿态观测的信息矩阵尚未添加
    // 备注：各位使用时可只用位置观测，而不用姿态观测，影响不大
    // 我自己在别的地方尝试过增加姿态观测，但效果反而变差，如果感兴趣，可自己编写此处的信息矩阵，并在后端优化中添加相应的边进行验证
    Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise) 
    {
        Eigen::MatrixXd information_matrix;
        return information_matrix;
    }

}