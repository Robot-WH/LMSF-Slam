#ifndef EDGE_PLANE_PRIOR_HPP
#define EDGE_PLANE_PRIOR_HPP

#include <Eigen/Dense>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::ColMajor>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace g2o {
class EdgePlanePriorNormal : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlanePriorNormal() : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPlane>() {}

  void computeError() override {
    const g2o::VertexPlane* v1 = static_cast<const g2o::VertexPlane*>(_vertices[0]);
    Eigen::Vector3d normal = v1->estimate().normal();

    if(normal.dot(_measurement) < 0.0) {
      normal = -normal;
    }

    _error = normal - _measurement;
  }

  void setMeasurement(const Eigen::Vector3d& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);
    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};


// 全局平面约束 单边   主要用在 室内
// 误差为6维的向量      g2o::SE3Quat 表示观测值     g2o::VertexSE3Expmap 为单边连接节点类型  
class EdgeSE3PlanePrior:public g2o::BaseUnaryEdge<6, Eigen::Isometry3d, g2o::VertexSE3>
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PlanePrior() : g2o::BaseUnaryEdge<6, Eigen::Isometry3d, g2o::VertexSE3>() {}

  void computeError() override {
      const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
      Eigen::Isometry3d error_T = _measurement * v1->estimate().inverse();
      g2o::SE3Quat err(error_T.linear(), error_T.translation());      // SE3目标值*SE3当前值 
      _error = err.log();            // 求误差的李代数  残差量   前3个为旋转  后面为平移     
  }
  
  // 设置观测值  即全局平面存在时的位姿 
  void setMeasurement(const Eigen::Isometry3d& m) override {
    _measurement = m;
  }

    // 求se3向量的左jacobian
  Matrix6d invJl(Vector6d const& se3)
  {
     // 获得旋转向量   前3个
     Eigen::Vector3d rotate_vector = se3.head(3);
     double angle = rotate_vector.norm();      // 求旋转角
     double cos = std::cos(angle);
     double sin = std::sin(angle);
     
     double angle_2 = angle*angle;   
     double angle_4 = angle_2*angle_2;  
     double k[4];
     k[0] = (4 - angle*sin - 4*cos) / 2*angle_2;
     k[1] = (4*angle - 5*sin + angle*cos) / 2*angle_2*angle;  
     k[2] = (2 - angle*sin -2*cos) / 2*angle_4;
     k[3] = (2*angle - 3*sin +angle*cos) / 2*angle_4*angle;   

     Matrix6d adj_se3;
     adj_se3.block<3,3>(0,0) = skew(rotate_vector);   
     adj_se3.block<3,3>(0,3) = skew(se3.tail(3)); 
     adj_se3.block<3,3>(3,3) = skew(rotate_vector);

     Matrix6d adj_k = Matrix6d::Identity(); 
     Matrix6d Jl = Matrix6d::Identity();  

     for(int i=0; i<4; i++)
     {
       adj_k *= adj_se3;  
       Jl += k[i] * adj_k;   
     }
     std::cout<<" Jl ----------------------------"<<std::endl<<Jl.matrix()<<std::endl;
     
     return Jl.inverse();   
  }

  void linearizeOplus()
  {
    // 获得当前节点状态  
    const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
    // 误差向量
    Eigen::Isometry3d error_T = _measurement * v1->estimate().inverse();
    std::cout<<" error_T ----------------------------"<<std::endl<<error_T.matrix()<<std::endl;
    // 构造SE3  
    g2o::SE3Quat err(error_T.linear(), error_T.translation());      // SE3目标值*SE3当前值 
    // 求误差的李代数  残差量      
    Vector6d error = err.log();   
    std::cout<<" error ----------------------------"<<std::endl<<error.transpose()<<std::endl;
    // 误差关于状态的jacobian
    _jacobianOplusXi = -invJl(-error); 
  }
  
  virtual bool read(std::istream& is) override {
    /*
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);
    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
    */
  }

  virtual bool write(std::ostream& os) const override {
    /*
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
    */
  }

};

class EdgePlanePriorDistance : public g2o::BaseUnaryEdge<1, double, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlanePriorDistance() : g2o::BaseUnaryEdge<1, double, g2o::VertexPlane>() {}

  void computeError() override {
    const g2o::VertexPlane* v1 = static_cast<const g2o::VertexPlane*>(_vertices[0]);
    _error[0] = _measurement - v1->estimate().distance();
  }

  void setMeasurement(const double& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    is >> _measurement;
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    os << _measurement;
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};
}  // namespace g2o

#endif  // EDGE_SE3_PRIORXY_HPP
