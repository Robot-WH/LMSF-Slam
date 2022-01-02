#ifndef KKL_G2O_EDGE_SE3_PLANE_HPP
#define KKL_G2O_EDGE_SE3_PLANE_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace g2o {
	class EdgeSE3Plane : public g2o::BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3, g2o::VertexPlane> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			EdgeSE3Plane()
			: BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3, g2o::VertexPlane>()
		{}
		
        // 残差的构建   
		void computeError() override {
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]); 
			const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);
            // 当前位姿  base_link -> world  
            Eigen::Isometry3d b2w = v1->estimate();
			Eigen::Matrix3d Rwb = b2w.linear();                      
			Eigen::Vector3d Twb = b2w.translation(); 
//			std::cout<<"w2n:"<<std::endl<<w2n.matrix()<<std::endl;     // .linear() 获取旋转部分  .translation()获取平移
             
            Plane3D globar_plane = v2->estimate(); 
			Eigen::Vector4d local_plane; 
            // 计算全局一致平面在雷达系下的表示
			local_plane.head<3>() = Rwb.transpose() * globar_plane.normal();       // 约束旋转
            local_plane[3] = - globar_plane.distance() + Twb.transpose()*globar_plane.normal();    //约束高度

            // 方向向量与坐标的夹角
			double alpha_l = std::atan2(local_plane[1], local_plane[0]);     // 绕z轴的旋转
			double beta_l = std::atan2(local_plane[2],local_plane.head<2>().norm());  // 绕y轴的旋转
			// 一个指向x正方向的向量  先绕y轴顺时针转beta_l   再绕Z轴逆时针转alpha_l  会得到现在的n向量
			Eigen::Matrix3d Rotation = (Eigen::AngleAxis<double>(alpha_l,Eigen::Vector3d::UnitZ())*Eigen::AngleAxis<double>(-beta_l, Eigen::Vector3d::UnitY())).toRotationMatrix();
			// 观测的地面旋转到x方向
			Eigen::Vector3d n = Rotation.transpose()*_measurement.normal();
			// 计算残差  
			_error = Eigen::Vector3d(std::atan2(n[1], n[0]), std::atan2(n[2],n.head<2>().norm()), _measurement.distance()+local_plane[3]);
		}  
		
			// local_plane法相量的夹角	

			/*
			double alpha_l = std::atan2(local_plane[1], local_plane[0]);
			double beta_l = std::atan2(local_plane[2],local_plane.head<2>().norm());
            // _measurement法相量的夹角
			double alpha_m = std::atan2(_measurement.normal()[1] ,_measurement.normal()[0]);
			double beta_m = std::atan2(_measurement.normal()[2],_measurement.normal().head<2>().norm());
			// 计算残差
			_error = Eigen::Vector3d(alpha_l - alpha_m, beta_l - beta_m, local_plane[3]+_measurement.distance());
            */ 
 
		/*
        	void computeError() override {
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
			const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);

			Eigen::Isometry3d w2n = v1->estimate().inverse();
			Plane3D local_plane = w2n * v2->estimate();
			_error = local_plane.ominus(_measurement);
		} */
		

		void setMeasurement(const g2o::Plane3D& m) override {
			_measurement = m;
		}

		virtual bool read(std::istream& is) override {
			Eigen::Vector4d v;
			is >> v(0) >> v(1) >> v(2) >> v(3);
			setMeasurement(Plane3D(v));
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j) {
					is >> information()(i, j);
					if (i != j)
						information()(j, i) = information()(i, j);
				}
			return true;
		}
		
		virtual bool write(std::ostream& os) const override {
			Eigen::Vector4d v = _measurement.toVector();
			os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j)
					os << " " << information()(i, j);
			return os.good();
		}
	};
}

#endif
