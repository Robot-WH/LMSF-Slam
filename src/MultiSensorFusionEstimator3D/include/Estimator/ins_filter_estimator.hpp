/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-20 15:26:34
 * @Description: 
 * @Others: 
 */

#ifndef _INS_FILTER_ESTIMATOR_HPP_
#define _INS_FILTER_ESTIMATOR_HPP_

#include "utility.hpp"
#include "states.hpp"
#include "Estimator/filter_estimator_base.hpp"
#include "Estimator/Predictor/imu_predictor.hpp"
#include "Estimator/Correction/corrector_interface.hpp"
#include "Sensor/sensor.hpp"
#include "Sensor/gnss_data_process.hpp"
#include "Sensor/sensor_process_interface.hpp"
#include "Initialized/imu_initialized_tool.hpp"

namespace Estimator{

    using Sensor::ImuDataConstPtr;
    using Sensor::GnssDataConstPtr;  
    using Sensor::SensorProcessInterface;  

    template<typename _StateType, int _StateDim>
    using XYZCorrectorPtr = std::unique_ptr<CorrectorInterface<_StateType, Eigen::Vector3d, _StateDim>>;

    /**
     * @brief: INS 组合导航估计器 
     * @details:  融合IMU与GNSS，有IMU时融合IMU，没有IMU数据时，采用 运动学模型  
     * @param _StateType 不优化内参/优化内参 
     * @param _StateDim 状态维度，还得考虑是否优化重力  
     */    
    template<typename _StateType, int _StateDim>
    class InsEstimator  
    : public FilterEstimatorBase<_StateType, _StateDim, ImuDataConstPtr, GnssDataConstPtr> {
            using BaseType = 
                FilterEstimatorBase<_StateType, _StateDim, ImuDataConstPtr, GnssDataConstPtr>;
            public:
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                InsEstimator(float const& acc_noise, 
                                          float const& gyro_noise, 
                                          float const& acc_bias_noise, 
                                          float const& gyro_bias_noise,
                                          XYZCorrectorPtr<_StateType, _StateDim> corrector_ptr,
                                          std::unique_ptr<Model::ImuMotionModelInterface> imu_motion_model_ptr
                                            = std::make_unique<Model::ImuMidIntegralModel>()  // 默认中值积分
                                          ) : imu_predictor_(acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, 
                                                std::move(imu_motion_model_ptr)), corrector_ptr_(std::move(corrector_ptr)) {}

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 估计器初始化  
                 * @details: 
                 * @param[in] timestamp 初始化时候的时间戳
                 * @return void 
                 */                
                virtual bool Initialize(double const& timestamp) override {
                    // 判断IMU数据是否足够   必须要有IMU才能初始化成功(一种自检，保证安全)
                    if(!imu_initialized_tool_.CheckInitializedImuDateNumber(prepared_imus_)) {
                        std::cout<<"imu data is not enough!!!"<<std::endl;
                        return false;  
                    }
                    // IMU数据中是否有旋转信息 
                    bool rotation_active = false;  
                    // 如果没有旋转信息   采用加速度进行初始化  
                    if((*prepared_imus_.begin())->rot.w()==0&&
                        (*prepared_imus_.begin())->rot.x()==0&&
                        (*prepared_imus_.begin())->rot.y()==0&&
                        (*prepared_imus_.begin())->rot.z()==0) {
                        std::cout<<"FilterEstimatorCentreRobot::Initialize ----- no rotation message !" << std::endl;
                    } else {
                        //rotation_active = true;
                        std::cout<<"FilterEstimatorCentreRobot::Initialize ----- use rotation message !" << std::endl;
                    }
                    std::vector<Eigen::Vector3d> acc_buf;   // 加速度buf
                    std::vector<Eigen::Quaterniond> rot_buf; // 旋转buf
                    std::vector<Eigen::Vector3d> gyro_buf;  // 陀螺仪buf
                    // 提取数据 
                    for(auto const& imu : prepared_imus_) {
                        acc_buf.push_back(std::move(imu->acc)); 
                        gyro_buf.push_back(std::move(imu->gyro)); 
                        if(!rotation_active) {
                            continue;
                        }
                        rot_buf.push_back(std::move(imu->rot));  
                    }
                    // 平滑后的等效IMU   用于初始化 
                    ImuDataPtr initialized_imu = std::make_shared<ImuData>(); 
                    initialized_imu->timestamp = timestamp;     // 时间戳和GNSS的相等  
                    // 检查加速度是否满足静止初始化条件并求取平均IMU数据
                    if(!imu_initialized_tool_.GetInfoFromImuData(acc_buf, gyro_buf, rot_buf, initialized_imu->acc, 
                                                            initialized_imu->gyro, initialized_imu->rot)) {
                        std::cout<<"motion excessive !! "<< std::endl;  
                        prepared_imus_.clear();  
                        return false; 
                    }
                    // 状态初始化
                    BaseType::estimated_state_.Reset();
                    BaseType::estimated_state_.common_states_.timestamp_ = timestamp; 
                    // 检查IMU是否可以获取旋转数据  
                    Eigen::Matrix3d R_w_b;  
                    if(!rotation_active) { // 靠重力初始化
                        R_w_b = imu_initialized_tool_.ComputeRotationFromGravity(initialized_imu->acc);
                        BaseType::estimated_state_.common_states_.Q_ = Eigen::Quaterniond(R_w_b).normalized();   
                        std::cout<<"imu rotation initialize ! R: "<< std::endl << R_w_b << std::endl;
                    } else {                   // 直接用旋转初始化 
                    }
                    // 陀螺仪偏执初始化 
                    Eigen::Vector3d const& gyro_bias = imu_initialized_tool_.StaticInitializeGyroBias(gyro_buf); 
                    BaseType::estimated_state_.gyro_bias_ = gyro_bias;  
                    // 估计器状态协方差矩阵初始化 
                    statesCovarianceInitialize();
                    // 预测器初始化
                    imu_predictor_.PredictInitialize(initialized_imu);  
                    return true;   
                }
                
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 对于IMU数据的处理 
                 * @param data
                 * @return {*}
                 */                
                virtual void Process(ImuDataConstPtr const& data) override {
                    // 只要gnss与lidar有一个初始化了  那么就进行滤波器的预测环节
                    if(BaseType::initialize_done_) {
                        std::cout<<"Imu predict !" <<std::endl;
                        imu_predictor_.Predict(BaseType::estimated_state_, data, BaseType::cov_);
                    } else { 
                        prepared_imus_.push_back(std::move(data));       // 缓存数据用于初始化  
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 对于GNSS数据的处理 
                 * @param data
                 * @return {*}
                 */                
                virtual void Process(GnssDataConstPtr const& data) override {
                    // 如果gnss没有初始化 
                    if(!BaseType::initialize_done_) {  
                        // 估计器初始化 
                        if(Initialize(data->timestamp)) { 
                            std::cout<<"estimator Initialize success!" <<std::endl;
                            BaseType::initialize_done_ = true;  
                            // GNSS初始化 
                            Sensor::GnssDataProcess &gnss_data_process = Sensor::GnssDataProcess::GetInstance(); 
                            gnss_data_process.InitOriginPosition(data->lla);  
                        } else {
                            std::cout<<"estimator Initialize failure!" <<std::endl;
                        } 
                    } else {  // 如果初始化了 执行滤波器校正  
                        // 首先将GNSS数据转换到ENU系下
                        Sensor::GnssDataProcess &gnss_processor = Sensor::GnssDataProcess::GetInstance();  
                        if(gnss_processor.UpdateXYZ(data->lla)) {
                            // Eigen::Vector3d position_obs = tig_ + (gnss_processor_ptr->GetEnuPosition()  
                            //                                             - BaseType::estimated_state__.common_states_.Q_*tig_);
                            Eigen::Vector3d position_obs = gnss_processor.GetEnuPosition(); 
                            // 计算GNSS观测协方差矩阵
                            Eigen::Matrix3d cov;
                            cov << 0.5, 0, 0,
                                        0, 0.5, 0,
                                         0, 0, 0.5;  
                            std::cout<<"gnss cov: "<<std::endl<<data->cov<<std::endl;
                            corrector_ptr_->Correct(position_obs, data->timestamp, data->cov, BaseType::estimated_state_, BaseType::cov_);  
                            std::cout<<"estimator correct!, GPS observe: "<<position_obs.transpose()<<std::endl;
                        } 
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 获取上一个IMU数据  
                 * @details: 
                 */                
                ImuDataConstPtr const& GetLastIMU() const {
                    return imu_predictor_.GetLastData();  
                }

            private:
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief: 协方差矩阵初始化   
                 */                
                void statesCovarianceInitialize(); 
                
            private:
                ImuPredictor imu_predictor_;   // Imu 预测器  
                std::unique_ptr<CorrectorInterface<_StateType, Eigen::Vector3d, _StateDim>> corrector_ptr_;  // 位置校正器
                ImuInitializedTool imu_initialized_tool_;    // IMU 初始化工具  
                std::vector<Sensor::ImuDataConstPtr> prepared_imus_;   // 用于初始化的imu数据 
    }; // namespace InsEstimator

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief:  初始化状态协方差矩阵
     * @details 状态为 StatesWithImu, 维度为 15 的特化  
     */    
    template<>
    void InsEstimator<StatesWithImu, 15>::statesCovarianceInitialize() {
        BaseType::cov_.setZero();
        // 状态初始协方差设置   
        BaseType::cov_.block<3, 3>(0, 0) = 0.01 * Eigen::Matrix3d::Identity(); // position std: 0.1 m
        BaseType::cov_.block<3, 3>(3, 3) = 0.01 * Eigen::Matrix3d::Identity(); // velocity std: 0.1 m/s
        // roll pitch std 10 degree.
        BaseType::cov_.block<2, 2>(6, 6) = 10. * CoefDegreeToRadian * 10. * CoefDegreeToRadian * Eigen::Matrix2d::Identity();
        BaseType::cov_(8, 8)             = 100. * CoefDegreeToRadian * 100. * CoefDegreeToRadian;     // yaw std: 100 degree.
        // Acc bias.
        BaseType::cov_.block<3, 3>(9, 9) = 0.01 * 0.01 * Eigen::Matrix3d::Identity();   
        // Gyro bias.
        BaseType::cov_.block<3, 3>(12, 12) = 0.0001 * 0.0001 * Eigen::Matrix3d::Identity();
    }


}; // namespace Estimator
#endif