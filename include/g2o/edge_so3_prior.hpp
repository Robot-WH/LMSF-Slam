#ifndef G2O_EDGE_SO3_PRIOR_HPP
#define G2O_EDGE_SO3_PRIOR_HPP

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
  // 主要用于IMU 位姿观测  	
  // 单边      观测值的类型为Eigen::Quaterniond        残差的维度为3      
class EdgeSO3Prior : public g2o::BaseUnaryEdge<3, Eigen::Matrix3d, g2o::VertexSE3> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSO3Prior()
		: g2o::BaseUnaryEdge<3, Eigen::Matrix3d, g2o::VertexSE3>()
		{}

		void computeError() override {
		    //  获取该边连接的节点
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
            //  获取节点状态的旋转部分
			Eigen::Matrix3d estimate_matrix = v1->estimate().linear();
            Eigen::Matrix3d error_matrix = _measurement * estimate_matrix.transpose();   
            // 转换为旋转向量
            Eigen::AngleAxisd error_vector(error_matrix);
			_error = error_vector.axis()*error_vector.angle();  
		}
		
        // 设定测量值    四元数   
		void setMeasurement(const Eigen::Matrix3d& m) override {
			_measurement = m;        
		}

        
        // 求误差关于状态的jacobian 
        void linearizeOplus()
        {
           //  获取该边连接的节点
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
            //  获取节点状态的旋转部分
			Eigen::Matrix3d estimate_matrix = v1->estimate().linear(); 
            std::cout<<"estimate_matrix--------------------------"<<std::endl<<estimate_matrix.matrix()<<std::endl;
            Eigen::Matrix3d error_matrix = _measurement * estimate_matrix.transpose();   
            std::cout<<"_measurement :-----------------------"<<std::endl<<_measurement.eulerAngles(2,1,0).transpose()<<std::endl;
            std::cout<<"error_matrix--------------------------"<<std::endl<<error_matrix<<std::endl;
            // 转换为旋转向量
            Eigen::AngleAxisd error_AngleAxisd(error_matrix); 
            std::cout<<"error_vector--------------------------"<<(error_AngleAxisd.angle() * error_AngleAxisd.axis()).transpose()<<std::endl;
            double angle = error_AngleAxisd.angle();     // 求旋转角度  
            double sin = std::sin(angle);
            double cos = std::cos(angle);
            // 求单位旋转轴
            Eigen::Vector3d axis = -error_AngleAxisd.axis(); 
            std::cout<<"error axis: "<<axis.transpose()<<" angle: "<<angle<<std::endl;
            Eigen::Matrix3d Jr = Eigen::Matrix3d::Zero();
            double k = sin / angle; 
            Jr(0,0) = k;
            Jr(1,1) = k;
            Jr(2,2) = k;
            Jr += (1-k)*axis*axis.transpose();  
            Jr -= (1-cos) * skew(axis) / angle; 
            // 求逆
            Eigen::Matrix3d invJr = Jr.inverse();  
            //std::cout<<"Jr--------------------------"<<std::endl<<Jr<<std::endl;
            
            _jacobianOplusXi(0,0) = -invJr(0,0);
            _jacobianOplusXi(0,1) = -invJr(0,1);
            _jacobianOplusXi(0,2) = -invJr(0,2);
            _jacobianOplusXi(0,3) = 0;
            _jacobianOplusXi(0,4) = 0;
            _jacobianOplusXi(0,5) = 0;

            _jacobianOplusXi(1,0) = -invJr(1,0);
            _jacobianOplusXi(1,1) = -invJr(1,1);
            _jacobianOplusXi(1,2) = -invJr(1,2);
            _jacobianOplusXi(1,3) = 0;
            _jacobianOplusXi(1,4) = 0;
            _jacobianOplusXi(1,5) = 0;

            _jacobianOplusXi(2,0) = -invJr(2,0);
            _jacobianOplusXi(2,1) = -invJr(2,1);
            _jacobianOplusXi(2,2) = -invJr(2,2);
            _jacobianOplusXi(2,3) = 0;
            _jacobianOplusXi(2,4) = 0;
            _jacobianOplusXi(2,5) = 0;
            
        }
        

		virtual bool read(std::istream& is) override {
            /*
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
            */
		}

		virtual bool write(std::ostream& os) const override {
            /*
			Eigen::Quaterniond q = _measurement;
			os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j)
					os << " " << information()(i, j);
			return os.good();
            */
		}
        
	};
}

#endif
