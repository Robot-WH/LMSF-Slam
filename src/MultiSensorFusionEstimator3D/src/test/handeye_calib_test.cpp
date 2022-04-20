/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-20 12:48:09
 * @Description: 
 * @Others: 
 */

#include "utility.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Algorithm/PointClouds/registration/ceres_edgeSurfFeatureRegistration.hpp"
#include "Algorithm/PointClouds/processing/Filter/voxel_grid.hpp"
#include "Common/pose.hpp"
#include "Math.hpp"

int main()
{
    // 下面证明了，不管两个传感器的外参是多少，以及如何运动，传感器坐标旋转转换为轴角后，角度变化都相等
    // 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
    Eigen::Vector3d ea(0, M_PI / 6, 0);
    // 欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "rotation matrix =\n" << rotation_matrix << endl;   
   // 外参 
    Eigen::Isometry3d extrinsic;
    extrinsic.linear() = rotation_matrix;
    extrinsic.translation() = Eigen::Vector3d(0, 0, 0);
    cout << "extrinsic =\n" << extrinsic.matrix() << endl;   

    Eigen::Isometry3d pose_ref; 
    pose_ref.setIdentity(); 
    Eigen::Isometry3d pose_curr, last_pose_curr; 
    pose_curr = pose_ref * extrinsic;  
    last_pose_curr = pose_curr;  
    std::cout<<"pose_ref: "<<pose_ref.matrix()<<" pose_curr: "<<pose_curr.matrix()<<std::endl;
    // 产生一个变换
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(M_PI / 7, Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(M_PI / 7, Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX());
    cout << "trans rotation matrix =\n" << rot << endl;   
    // 外参 
    Eigen::Isometry3d delta_trans;
    delta_trans.linear() = rot;
    delta_trans.translation() = Eigen::Vector3d(0, 0, 0);
    cout << "delta_trans =\n" << delta_trans.matrix() << endl;   
    Eigen::AngleAxisd rot_vector(delta_trans.rotation());
    cout << "delta_rot angle =\n" << rot_vector.angle() <<" ,axis: "<<rot_vector.axis().transpose() << endl;   
    // 作用给ref pose
    pose_ref = pose_ref * delta_trans;  
    pose_curr = pose_ref * extrinsic;  
    Eigen::Isometry3d delta_curr_trans = last_pose_curr.inverse() * pose_curr;  
    Eigen::AngleAxisd curr_rot_vector(delta_curr_trans.rotation());
    cout << "curr_rot_vector angle =\n" << curr_rot_vector.angle() <<" ,axis: "<<curr_rot_vector.axis().transpose() << endl;   

    // Eigen::Matrix4d Q_; 
    // Eigen::Quaterniond primary_q(delta_trans.rotation());    // 主传感器的旋转
    // Eigen::Quaterniond second_q(delta_curr_trans.rotation());  // 辅传感器的旋转  
    // // 求rubust核函数
    // Eigen::Matrix4d L = Math::QuanternionLeftProductMatrix(primary_q);  
    // Eigen::Matrix4d R = Math::QuanternionRightProductMatrix(second_q);  
    // Q_.block<4, 4>(0, 0) = (L - R);
    // // SVD 求解 AX = 0 
    // Eigen::JacobiSVD<Eigen::Matrix4d> svd(Q_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3); // [w, x, y, z]     // 最小奇异值对应的特征向量为解
    // if (x[0] < 0) x = -x; // use the standard quaternion
    // Eigen::Vector4d rot_cov = svd.singularValues(); // singular value
    // std::cout<<"rot_cov: "<<rot_cov.transpose()<<std::endl;
    // // 若没有退化  AQ = 0 零空间为1 ， 则A的秩 = 4 -1 = 3， 那么ATA秩也为3， 因此A的奇异值只有一个为0
    // //  因此 检查第二小奇异值， 看是否足够大，越大，解越稳定
    // Eigen::Quaterniond q(x);
    // std::cout<<"solve: "<<q.toRotationMatrix()<<std::endl;

        Eigen::Quaternionf q_w_l_;
        // 1.86373  2.90168 -2.91245
        float roll = -2.91245;
        float pitch = 2.90168;
        float yaw = 1.86373;

        std::cout<<"before: roll: "<<roll<<", pitch："
        <<pitch<<", yaw:"<<yaw<<std::endl;
        Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(roll,Eigen::Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(pitch,Eigen::Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ())); 
        q_w_l_= yawAngle * pitchAngle * rollAngle; 
        std::cout<<"q_w_l_: "<<q_w_l_.coeffs().transpose()<<std::endl;
        Eigen::Vector3f eulerAngle=q_w_l_.matrix().eulerAngles(2,1,0);  // 旋转方向  0 -> 1 ->2   输出结果：yaw, pitch ,roll  
        roll = eulerAngle[2];
        pitch = eulerAngle[1];
        yaw = eulerAngle[0];
        std::cout<<"after: roll: "<<roll<<", pitch："
        <<pitch<<", yaw:"<<yaw<<std::endl;
        Eigen::AngleAxisf rollAngle2(Eigen::AngleAxisf(roll,Eigen::Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle2(Eigen::AngleAxisf(pitch,Eigen::Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle2(Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ())); 
        q_w_l_= yawAngle2 * pitchAngle2 * rollAngle2; 
        // if (q_w_l_.w() < 0) q_w_l_.coeffs() = -q_w_l_.coeffs();  
        std::cout<<"q_w_l_: "<<q_w_l_.coeffs().transpose()<<std::endl;
        eulerAngle=q_w_l_.matrix().eulerAngles(2,1,0);  // 旋转方向  0 -> 1 ->2   输出结果：yaw, pitch ,roll  
        roll = eulerAngle[2];
        pitch = eulerAngle[1];
        yaw = eulerAngle[0];
        std::cout<<"after: roll: "<<roll<<", pitch："
        <<pitch<<", yaw:"<<yaw<<std::endl;

    return 1;
}
