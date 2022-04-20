/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-20 15:26:34
 * @Description: 
 * @Others: 
 */

#ifndef _FILTER_LIO_ESTIMATOR_HPP_
#define _FILTER_LIO_ESTIMATOR_HPP_

#include "utility.hpp"
#include "states.hpp"
#include "Estimator/filter_estimator_base.hpp"
#include "Estimator/ins_filter_estimator.hpp"
#include "Estimator/Predictor/imu_predictor.hpp"
#include "Estimator/Correction/corrector_interface.hpp"
#include "Estimator/Correction/GNSS/position_correction.hpp"
#include "Sensor/sensor.hpp"
#include "Sensor/gnss_data_process.hpp"
#include "Sensor/sensor_process_interface.hpp"
#include "Initialized/imu_initialized_tool.hpp"
#include "LidarTracker/LidarTrackerBase.hpp"

namespace Slam3D{

    template<typename _StateType, int _ErrorStateDim>
    using LidarCorrectorPtr = std::unique_ptr<CorrectorInterface<_StateType, Eigen::Vector3d, _ErrorStateDim>>;

    template<typename _StateType, int _ErrorStateDim>
    using XYZCorrectorPtr = std::unique_ptr<CorrectorInterface<_StateType, Eigen::Vector3d, _ErrorStateDim>>;
    //  激光里程计类型  
    using LidarTrackerPtr = std::unique_ptr<Module::LidarTrackerBase<Sensor::PCLConstPtr>>;

    /**
     * @brief: LIO估计器   
     * @details:  激光雷达融合IMU的里程计， IMU用来预测，当没有IMU时，采用运动模型预测， 
     *                       GPS用于轨迹对比以及外参标定并不进行融合，否则就不是里程计了 
     * @param _StateType 采用的状态类型  - 是否在线优化IMU与激光的外参
     * @param _ErrorStateDim 误差状态维度，考虑是否优化外参  
     *                    对于参数的变化 采用模板特化进行覆盖 
     */    
    template<typename _StateType, int _ErrorStateDim>
    class LioEstimator  : public FilterEstimatorBase<_StateType, _ErrorStateDim, 
                                                LidarDataConstPtr, ImuDataConstPtr, GnssDataConstPtr> 
    {
            using BaseType = FilterEstimatorBase<_StateType, _ErrorStateDim, 
                                                    LidarDataConstPtr, ImuDataConstPtr, GnssDataConstPtr>;
            public:
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                LioEstimator(float const& acc_noise, float const& gyro_noise, float const& acc_bias_noise, 
                                            float const& gyro_bias_noise, LidarTrackerPtr lidar_odometry = nullptr,      // 用于初始外参标定的激光里程计
                                            //XYZCorrectorPtr<_StateType, _ErrorStateDim> corrector_ptr,
                                            std::unique_ptr<Model::ImuMotionModelInterface> imu_motion_model_ptr
                                                = std::make_unique<Model::ImuMidIntegralModel>()  // 默认中值积分
                                          ) : imu_predictor_(acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, 
                                                std::move(imu_motion_model_ptr)), lidar_tracker_(std::move(lidar_odometry)) 
                {
                            // 初始化 用于INS组合导航的估计器  用于外参估计
                            ins_estimator_.reset(new InsEstimator<StatesWithImu, 15>(
                                                                        acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, 
                                                                        XYZCorrectorPtr<StatesWithImu, 15>(
                                                                        new PositionCorrection<Estimator::StatesWithImu, 16, 15>()
                                                                    )));
                            
                }

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 估计器初始化  
                 * @details: 
                 * @param[in] timestamp 初始化时候的时间戳
                 * @return void 
                 */                
                virtual bool Initialize(double const& timestamp) override 
                {
                    // // 判断IMU数据是否足够   必须要有IMU才能初始化成功(一种自检，保证安全)
                    // if(!imu_initialized_tool_.CheckInitializedImuDateNumber(prepared_imus_)) {
                    //     std::cout<<"imu data is not enough!!!"<<std::endl;
                    //     return false;  
                    // }
                    // // IMU数据中是否有旋转信息 
                    // bool rotation_active = false;  
                    // // 如果没有旋转信息   采用加速度进行初始化  
                    // if((*prepared_imus_.begin())->rot.w()==0&&
                    //     (*prepared_imus_.begin())->rot.x()==0&&
                    //     (*prepared_imus_.begin())->rot.y()==0&&
                    //     (*prepared_imus_.begin())->rot.z()==0) {
                    //     std::cout<<"FilterEstimatorCentreRobot::Initialize ----- no rotation message !" << std::endl;
                    // } else {
                    //     //rotation_active = true;
                    //     std::cout<<"FilterEstimatorCentreRobot::Initialize ----- use rotation message !" << std::endl;
                    // }
                    // std::vector<Eigen::Vector3d> acc_buf;   // 加速度buf
                    // std::vector<Eigen::Quaterniond> rot_buf; // 旋转buf
                    // std::vector<Eigen::Vector3d> gyro_buf;  // 陀螺仪buf
                    // // 提取数据 
                    // for(auto const& imu : prepared_imus_) {
                    //     acc_buf.push_back(std::move(imu->acc)); 
                    //     gyro_buf.push_back(std::move(imu->gyro)); 
                    //     if(!rotation_active) {
                    //         continue;
                    //     }
                    //     rot_buf.push_back(std::move(imu->rot));  
                    // }
                    // // 平滑后的等效IMU   用于初始化 
                    // ImuDataPtr initialized_imu = std::make_shared<ImuData>(); 
                    // initialized_imu->timestamp = timestamp;     // 时间戳和GNSS的相等  
                    // // 检查加速度是否满足静止初始化条件并求取平均IMU数据
                    // if(!imu_initialized_tool_.GetInfoFromImuData(acc_buf, gyro_buf, rot_buf, initialized_imu->acc, 
                    //                                         initialized_imu->gyro, initialized_imu->rot)) {
                    //     std::cout<<"motion excessive !! "<< std::endl;  
                    //     prepared_imus_.clear();  
                    //     return false; 
                    // }
                    // // 状态初始化
                    // BaseType::estimated_state_.Reset();
                    // BaseType::estimated_state_.common_states_.timestamp_ = timestamp; 
                    // // 检查IMU是否可以获取旋转数据  
                    // Eigen::Matrix3d R_w_b;  
                    // if(!rotation_active) { // 靠重力初始化
                    //     R_w_b = imu_initialized_tool_.ComputeRotationFromGravity(initialized_imu->acc);
                    //     BaseType::estimated_state_.common_states_.Q_ = Eigen::Quaterniond(R_w_b).normalized();   
                    //     std::cout<<"imu rotation initialize ! R: "<< std::endl << R_w_b << std::endl;
                    // } else {                   // 直接用旋转初始化 
                    // }
                    // // 陀螺仪偏执初始化 
                    // Eigen::Vector3d const& gyro_bias = imu_initialized_tool_.StaticInitializeGyroBias(gyro_buf); 
                    // BaseType::estimated_state_.gyro_bias_ = gyro_bias;  
                    // // 估计器状态协方差矩阵初始化 
                    // statesCovarianceInitialize();
                    // // 预测器初始化
                    // imu_predictor_.PredictInitialize(initialized_imu);  
                    // return true;   
                }
                
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 对于Lidar数据的处理 
                 * @param data
                 * @return {*}
                 */                
                virtual void Process(LidarDataConstPtr const& data) override 
                {
                    PCLPtr pointcloud(new PCLType(data->point_clouds_)); 
                    // 如果需要估计外参
                    if (ESTIMATE_EXTRINSIC_) 
                    {
                        // 计算纯激光里程计
                        if (lidar_tracker_ != nullptr) 
                        {
                            // lidar_tracker_->Solve(pointcloud);  
                        }
                        // 估计出激光与IMU的外参后进行初始化 
                        ESTIMATE_EXTRINSIC_ = false;  
                    } 
                    else 
                    {
                        // 外参已经获取, 接下来进行初始化 (将激光对齐到ENU坐标系下)+观测更新(可选择是否在线外参优化)
                        if (BaseType::initialize_done_) 
                        {
                        } 
                        else 
                        {
                            // 进行初始化   IMU静止初始化 

                        }
                    }
                }



                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 对于IMU数据的处理 
                 * @param data
                 * @return {*}
                 */                
                virtual void Process(ImuDataConstPtr const& data) override 
                {
                    // 初始化完成后就进行滤波器的预测环节
                    if (BaseType::initialize_done_) 
                    {
                        std::cout<<"Imu predict !" <<std::endl;
                        imu_predictor_.Predict(BaseType::estimated_state_, data, BaseType::cov_);
                    } 
                    else 
                    { 
                        if (ESTIMATE_EXTRINSIC_) 
                        {
                            ins_estimator_->Process(data);       // 如果需要估计外参  则将当前IMU数据交给 INS 估计器 
                        } 
                        else 
                        {
                            prepared_imus_.push_back(std::move(data));       // 缓存数据用于初始化  
                        }
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 对于GNSS数据的处理 
                 * @param data
                 * @return {*}
                 */                
                virtual void Process(GnssDataConstPtr const& data) override 
                {
                    // 不需要外参标定 那么直接
                    if (!ESTIMATE_EXTRINSIC_) 
                    {

                    } 
                    else 
                    {  // 如果需要外参初始化，那么应该单独构造INS估计器
                        ins_estimator_->Process(data);  
                    }

                }

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 获取上一个IMU数据  
                 * @details: 
                 */                
                ImuDataConstPtr const& GetLastIMU() const 
                {
                    return imu_predictor_.GetLastData();  
                }

            private:

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 协方差矩阵初始化   
                 */                
                void statesCovarianceInitialize(); 
                
            private:
                std::unique_ptr<InsEstimator<StatesWithImu, 15>> ins_estimator_; // INS估计器  用于GNSS&Lidar外参标定 
                LidarTrackerPtr lidar_tracker_;   // 激光里程计  用于外参标定时激光运动的计算  
                ImuPredictor imu_predictor_;                 // Imu 预测器  
                std::unique_ptr<CorrectorInterface<_StateType, Eigen::Vector3d, _ErrorStateDim>> corrector_ptr_;  // 位置校正器
                ImuInitializedTool imu_initialized_tool_;    // IMU 初始化工具  
                std::vector<Sensor::ImuDataConstPtr> prepared_imus_;   // 用于初始化的imu数据 
                bool ESTIMATE_EXTRINSIC_ = false;  // 是否估计外参 
    }; // namespace LioEstimator
}; // namespace Estimator
#endif