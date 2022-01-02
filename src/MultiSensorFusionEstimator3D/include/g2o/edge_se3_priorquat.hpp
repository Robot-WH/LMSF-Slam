#ifndef KKL_G2O_EDGE_SE3_PRIORQUAT_HPP
#define KKL_G2O_EDGE_SE3_PRIORQUAT_HPP

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
  // 主要用于IMU 位姿观测  	
  //单边      观测值的类型为Eigen::Quaterniond        残差的维度为3      
class EdgeSE3PriorQuat : public g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSE3PriorQuat()
		: g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3>()
		{}

		void computeError() override {
		       //  获取该边连接的节点
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
               //  获取节点状态的旋转部分
			Eigen::Quaterniond estimate = Eigen::Quaterniond(v1->estimate().linear());
               // 这什么意思？？？？？？？？？？？？？？？？？
			if(_measurement.coeffs().dot(estimate.coeffs()) < 0.0) {
				estimate.coeffs() = -estimate.coeffs();
			}
			// 虚部相减     为啥？？？？？？？？？？？？？？？？？？？？？
			_error = estimate.vec() - _measurement.vec();
		}
		
        // 设定测量值    四元数   
		void setMeasurement(const Eigen::Quaterniond& m) override {
			_measurement = m;        
			// 什么意思？？？？？？？    保持w为正？？？？
			if(m.w() < 0.0) {
				_measurement.coeffs() = -m.coeffs();
			}
		}

		virtual bool read(std::istream& is) override {
			Eigen::Quaterniond q;
			is >> q.w() >> q.x() >> q.y() >> q.z();
			setMeasurement(q);
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j) {
					is >> information()(i, j);
					if (i != j)
						information()(j, i) = information()(i, j);
				}
			return true;
		}
		virtual bool write(std::ostream& os) const override {
			Eigen::Quaterniond q = _measurement;
			os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j)
					os << " " << information()(i, j);
			return os.good();
		}
	};
}

#endif
