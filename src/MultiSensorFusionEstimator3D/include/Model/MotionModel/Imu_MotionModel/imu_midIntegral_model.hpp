/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */

#ifndef _IMU_MIDINTEGRAL_MODEL_HPP_
#define _IMU_MIDINTEGRAL_MODEL_HPP_

#include <eigen3/Eigen/Dense>
#include "Sensor/sensor.hpp"
#include "imu_motion_model_interface.hpp"

namespace Slam3D {
    /**
     * @brief IMU运动模型的派生子类  实现中值积分
     */
    class ImuMidIntegralModel : public ImuMotionModelInterface 
    {
        public:
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ImuMidIntegralModel() {}

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            void ImuMotionModelInitialize(ImuData const& curr_imu) override 
            {
                last_imu_ = curr_imu;   
            }
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 中值积分预测PVQ
             * @param[in/out] states 上一时刻的状态/推导的下一时刻状态
             * @param curr_imu 当前的imu测量
             * @param acc_bias IMU加速度偏置
             * @param gyro_bias IMU角速度偏置 
             */
            void ImuPredictPVQ( CommonStates &states, 
                                                        ImuDataConstPtr const& curr_imu, 
                                                        Eigen::Vector3d const& acc_bias, 
                                                        Eigen::Vector3d const& gyro_bias) override 
            {
                // 时间间隔
                double const delta_t = curr_imu->timestamp - states.timestamp_;
                double const delta_t2 = delta_t * delta_t;
                // 中值积分  
                // Acc and gyro.
                Eigen::Vector3d const acc_0 = states.Q_ * (last_imu_.acc - acc_bias) + states.G_;
                Eigen::Vector3d const mid_gyro_unbias = 0.5 * (last_imu_.gyro + curr_imu->gyro) - gyro_bias;
                // 角增量向量   陀螺仪测出来的角速度向量就可以等价为旋转向量  
                Eigen::Vector3d const delta_angle_axis = mid_gyro_unbias * delta_t;
                // 更新旋转  
                if (delta_angle_axis.norm() > 1e-12) {
                    states.Q_ = states.Q_ * Eigen::Quaterniond(
                            1, delta_angle_axis[0] / 2, delta_angle_axis[1] / 2, delta_angle_axis[2] / 2);
                }
                Eigen::Vector3d const acc_1 = states.Q_ * (curr_imu->acc - acc_bias) + states.G_;
                Eigen::Vector3d const mid_acc_unbias = 0.5 * (acc_0 + acc_1);  
                // nominal state. 
                states.P_ += (states.V_ * delta_t + 0.5 * mid_acc_unbias * delta_t2);
                states.V_ += (mid_acc_unbias * delta_t);
                // 记录IMU数据 
                last_imu_ = *curr_imu; 
                return;  
            }
        private:
            ImuData last_imu_;
    }; // class ImuMidIntegralModel
} // namespace Model

#endif