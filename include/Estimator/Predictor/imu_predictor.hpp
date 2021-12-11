/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */

#ifndef _IMU_PREDICTOR_HPP_
#define _IMU_PREDICTOR_HPP_

#include "utility.hpp"
#include "Sensor/sensor.hpp"
#include "Estimator/states.hpp"
#include "Model/MotionModel/Imu_MotionModel/imu_midIntegral_model.hpp"
#include "predictor_interface.hpp"

namespace Estimator{
    /**
     * @brief 卡尔曼滤波的IMU预测类 
     * @details 变化主要有： 1、采用不同的状态（重载）      2、不同的IMU积分方式 (重载)
     * @param _StatesType 为预测器的状态
     */
    template<typename _StatesType>
    class ImuPredictor : public PredictorInterface<_StatesType, ImuDataConstPtr>   // ImuDataConstPtr 为当前预测器使用的传感器数据 
    {
        public:
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ImuPredictor() {}

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ImuPredictor( float const& acc_noise, float const& gyro_noise, 
                                        float const& acc_bias_noise, float const& gyro_bias_noise,
                                        std::shared_ptr<Model::ImuMotionModelInterface> imu_motion_model_ptr)
                                        : acc_noise_(acc_noise), gyro_noise_(gyro_noise), 
                                        acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
                                        imu_motion_model_ptr_(imu_motion_model_ptr) {
                last_imu_ = std::make_shared<ImuData>();
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ~ImuPredictor() {}

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ImuDataConstPtr const& GetLastData() const override {
                return last_imu_; 
            }
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            void SetLastData(ImuDataConstPtr const& data) override {
                last_imu_=data;  
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            void PredictInitialize(ImuDataConstPtr const& data) override {   
                SetLastData(data);
                imu_motion_model_ptr_->ImuMotionModelInitialize(*data);  
                is_initialized_ = true;  
            }
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
             * @brief 使用IMU的预测环节 
             * @param states 上一时刻的状态 
             * @param curr_imu 当前时刻的IMU测量  
             */ 
            void Predict(_StatesType &states, ImuDataConstPtr const& curr_imu) {
                // 如果没有初始化  或者上一时刻状态与上一时刻IMU时间戳不相等 则退出   
                if(!is_initialized_||states.common_states_.timestamp_ != last_imu_->timestamp) {   
                    std::cout<<"error !!!"<<std::endl;
                    return; 
                } 
                // 可以采用多种积分方式  RK4、中值
                // IMU运动积分只与状态与IMU测量值有关   因此  采用多态方式进行切换 
                std::cout<<"PVQ predict - acc_bias_: "<<states.acc_bias_.transpose()<<" ,gyro_bias_:"<<states.gyro_bias_.transpose()<<std::endl;
                imu_motion_model_ptr_->ImuPredictPvq(states.common_states_, curr_imu, states.acc_bias_, states.gyro_bias_);
                // 时间差 
                const double delta_t = curr_imu->timestamp - states.common_states_.timestamp_;
                const double delta_t2 = delta_t * delta_t;
                // Acc and gyro.
                const Eigen::Vector3d acc_unbias = 0.5 * (last_imu_->acc + curr_imu->acc) - states.acc_bias_;
                const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu_->gyro + curr_imu->gyro) - states.gyro_bias_;
                // Error-state. Not needed.
                // Fx
                Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
                Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
                Fx.block<3, 3>(3, 6)   = - states.common_states_.Q_.toRotationMatrix() * 
                                                                            GetSkewMatrix(acc_unbias) * delta_t;
                Fx.block<3, 3>(3, 9)   = - states.common_states_.Q_.toRotationMatrix() * delta_t;
                
                Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;  

                if (delta_angle_axis.norm() > 1e-12) {
                    Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), 
                                                                delta_angle_axis.normalized()).toRotationMatrix().transpose();
                } else {
                    Fx.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity();
                }

                Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;
                // Fi  IMU噪声转换矩阵   IMU噪声只是影响 速度、旋转、bias 
                Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
                Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();
                // IMU噪声协方差矩阵 
                Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
                Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
                Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
                Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
                Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

                states.cov_ = Fx * states.cov_ * Fx.transpose() + Fi * Qi * Fi.transpose();
                // 更新状态的时间戳 
                states.common_states_.timestamp_ = curr_imu->timestamp;
                // 保存为上一个imu数据 
                last_imu_ = curr_imu; 
            }

        private:
            template<typename __StatesType>
            void calculateF(__StatesType const& states, Eigen::MatrixXd Fx, Eigen::MatrixXd Fi, Eigen::MatrixXd &Qi);

        private:
            // 上一时刻IMU数据  
            ImuDataConstPtr last_imu_; 
            // IMU运动模型
            std::shared_ptr<Model::ImuMotionModelInterface> imu_motion_model_ptr_;  
            // IMU参数
            float acc_noise_;
            float gyro_noise_;
            float acc_bias_noise_; 
            float gyro_bias_noise_; 
            bool is_initialized_ = false; 
    }; // class ImuPredictor

    // // 模板类函数特化
    // template<>
    // ImuPredictor<StatesWithImu>::calculateF(StatesWithImu const& states, Eigen::MatrixXd &Fx, 
    //                                                                                         Eigen::MatrixXd &Fi, Eigen::MatrixXd &Qi) {
    //     // Fx
    //     Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    //     Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
    //     Fx.block<3, 3>(3, 6)   = - states.common_states_.Q_.toRotationMatrix() * 
    //                                                                 GetSkewMatrix(acc_unbias) * delta_t;
    //     Fx.block<3, 3>(3, 9)   = - states.common_states_.Q_.toRotationMatrix() * delta_t;
        
    //     Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;  

    //     if (delta_angle_axis.norm() > 1e-12) {
    //         Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), 
    //                                                     delta_angle_axis.normalized()).toRotationMatrix().transpose();
    //     } else {
    //         Fx.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    //     }

    //     Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;
    //     // Fi  IMU噪声转换矩阵   IMU噪声只是影响 速度、旋转、bias 
    //     Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    //     Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();
    //     // IMU噪声协方差矩阵 
    //     Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    //     Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    //     Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    //     Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    //     Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();
    // }


} // namespace 

#endif  
