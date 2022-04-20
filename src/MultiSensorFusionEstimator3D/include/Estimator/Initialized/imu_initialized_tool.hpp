
#ifndef _IMU_INITIALIZED_TOOL_HPP_
#define _IMU_INITIALIZED_TOOL_HPP_

#include "utility.hpp"
#include "Sensor/gnss_data_process.hpp"
#include "Sensor/sensor.hpp"

namespace Slam3D {

    /**
     * @brief: 初始化中IMU的相关操作 
     * @details: 
     */    
    class ImuInitializedTool 
    {
        public:
            ImuInitializedTool() : MeanAccLimit_(0.05),  minimum_dataSize_(10){}

            ImuInitializedTool(double MeanAccLimit, int minimum_dataSize) 
                : MeanAccLimit_(MeanAccLimit), minimum_dataSize_(minimum_dataSize) {}

            virtual ~ImuInitializedTool(){}
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 检查用于初始化的IMU数据数量
             */
            bool CheckInitializedImuDateNumber(std::vector<ImuDataConstPtr> const& imu_datas) const 
            {
                std::cout<<" imu_datas.size(): " << imu_datas.size() <<" minimum_dataSize_: "
                << minimum_dataSize_ <<std::endl;
                return (imu_datas.size() >= minimum_dataSize_);
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 获取初始化时刻的IMU数据   
             * @details 对静止初始化的IMU数据去平均值     
             * @param [in] acc_buf 加速度容器
             * @param [in] gyro_buf 角速度容器
             * @param [in] rot_buf 旋转四元数容器
             * @param[out] mean_acc 输出平滑后的加速度
             * @param[out] mean_gyro 输出平滑后的角速度 
             * @return 是否满足要求 
             */
            bool GetInfoFromImuData(std::vector<Eigen::Vector3d> const& acc_buf, 
                                                                    std::vector<Eigen::Vector3d> const& gyro_buf,
                                                                    std::vector<Eigen::Quaterniond> const& rot_buf, 
                                                                    Eigen::Vector3d &mean_acc, 
                                                                    Eigen::Vector3d &mean_gyro, 
                                                                    Eigen::Quaterniond &mean_rot) const 
            {
                // Compute mean and std of the imu buffer.
                Eigen::Vector3d sum_acc(0., 0., 0.);
                // 计算加速度均值 
                for  (auto const& acc_data : acc_buf) {
                    sum_acc += acc_data;
                }
                mean_acc = sum_acc / (double)acc_buf.size();
                Eigen::Vector3d sum_err2(0., 0., 0.);
                // 计算标准差
                for  (const auto acc_data : acc_buf) {
                    sum_err2 += (acc_data - mean_acc).cwiseAbs2();
                }
                const Eigen::Vector3d sigma_acc = (sum_err2 / (double)acc_buf.size()).cwiseSqrt();
                // 求模长 检查是否运动过于剧烈 
                if  (sigma_acc.norm() > MeanAccLimit_) 
                {
                    std::cout << "[CheckImuMotionForStaticInitialization]: Too sigma_acc: " 
                    << sigma_acc.transpose() << std::endl;
                    return false;
                }
                // 运动合理则继续计算
                // 计算陀螺仪均值 
                Eigen::Vector3d sum_gyro(0., 0., 0.);
                for (auto const& gyro_data : gyro_buf) {
                    sum_gyro += gyro_data;
                }
                mean_gyro = sum_gyro / (double)gyro_buf.size();
                if(!rot_buf.empty()) {
                    mean_rot = rot_buf.back();  
                } else {
                    mean_rot = Eigen::Quaterniond::Identity();
                }

                return true;  
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 通过IMU的重力数据恢复旋转  
             * @param Rwi imu->world的旋转  
             * @param smooth_gravity 平滑后的重力 
             * @return 是否计算成功 
             */
            Eigen::Matrix3d const ComputeRotationFromGravity(Eigen::Vector3d const& smooth_gravity) 
            {
                //  z-axis.   world系 即导航系z轴(重力方向) 在 载体系的表示 
                const Eigen::Vector3d& z_axis = smooth_gravity.normalized(); 
                //  x-axis.
                Eigen::Vector3d x_axis = 
                    Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
                x_axis.normalize();
                // y-axis.
                Eigen::Vector3d y_axis = z_axis.cross(x_axis);
                y_axis.normalize();
                // world -> body
                Eigen::Matrix3d R_i_n;
                R_i_n.block<3, 1>(0, 0) = x_axis;
                R_i_n.block<3, 1>(0, 1) = y_axis;
                R_i_n.block<3, 1>(0, 2) = z_axis;
                // 返回 imu->导航系的旋转 
                return R_i_n.transpose();
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 陀螺仪偏置  静态初始化 
             * @param gyro_buf 陀螺仪数据 
             */
            Eigen::Vector3d const StaticInitializeGyroBias(std::vector<Eigen::Vector3d> const& gyro_buf) 
            {   
                // 就是求平均值
                Eigen::Vector3d sum_value;
                for(auto const& gyro : gyro_buf) {
                    sum_value += gyro;
                }
                return sum_value / gyro_buf.size();   
            }        
            
        private:
            // 初始化加速度阈值   
            double MeanAccLimit_;  
            // IMU缓存数量阈值
            int minimum_dataSize_;  
    }; // class ImuInitializedTool
};   // namespace Estimator 

#endif  