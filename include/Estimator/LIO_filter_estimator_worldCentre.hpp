/**
 * @brief Lidar-IMU-GNSS 滤波器状态估计器  
 * @details 主要用于定位，建图时不使用  
 * @author wh.l
 * @date 2021/4/20
 */

#ifndef _LIDARIMUGNSS_FILTER_ESTIMATOR_WORLDCENTRE_HPP_
#define _LIDARIMUGNSS_FILTER_ESTIMATOR_WORLDCENTRE_HPP_

#include "utility.hpp"
#include "Estimator/LIO_filter_estimator_interface.hpp"
#include "Estimator/Predictor/imu_predictor.hpp"
#include "Estimator/Correction/GNSS/gnss_correction.hpp"
#include "Estimator/Correction/GNSS/gnss_correction_with_prior_constraint.hpp"
#include "initialize.hpp"
#include "Sensor/Gnss_data.h"

namespace Estimator{

  /**
   * @brief 使用滤波器融合IMU-LIDAR的里程计      
   * @details 估计的方式是以世界坐标系为中心
   *          变化主要有： 1. 滤波器  eskf/ieskf(主要是观测更新的区别，采取多态)    2. 估计的状态变量设置 (可以复用 ，采取模板)
   * @param EstimatedStates 估计器估计的状态
   * @param _dim_states 状态的维度   
   */
  template<typename EstimatedStates, int _dim_states >
  class LidarImuFilterEstimatorWorldCentre : public LidarImuFilterEstimatorInterFace
  {
    public: 
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        LidarImuFilterEstimatorWorldCentre(float const& imu_acc_noise, float const& imu_gyro_noise, 
                         float const& imu_acc_bias_noise, float const& imu_gyro_bias_noise,
                         std::shared_ptr<Model::ImuMotionModelInterface> imu_motion_model_ptr,
                         Eigen::Matrix3d Rlb, Eigen::Vector3d tlb, Eigen::Matrix3d Ril, Eigen::Vector3d til,
                         Eigen::Matrix3d Rig, Eigen::Vector3d tig);

        ~LidarImuFilterEstimatorWorldCentre(){}

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Sensor::ImuDataConstPtr const& GetLastImuData() const override
        {
          return imu_predict_.GetLastImuData();  
        }
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        CommonStates const& GetCommonStates() const
        {
          return estimated_states_.common_states_;
        }
                     
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 处理IMU数据  
         */
        void ProcessSensorData(Sensor::ImuDataConstPtr &imuData) override;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 处理GNSS数据  
         */
        void ProcessSensorData(Sensor::GnssDataConstPtr const& gnssData) override;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 处理Lidar数据  
         * @details 处理Lidar观测的位姿
         */
        void ProcessSensorData() override;
    private:
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
        /**
         * @brief 初始化
         * @param timestamp 初始化的时间戳 
         */
        bool estimatorInitialize(double const& timestamp);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 估计器状态协方差矩阵初始化 
         * @param states StatesWithImu类型 初始化的状态
         */
        void estimatorStatesCovarianceInitialize(StatesWithImu &states);

    private:
        // 初始化器
        Initializer initializer_;  
        // 估计的状态
        EstimatedStates estimated_states_;  
        // IMU处理
        ImuPredictor imu_predict_;  
        // Lidar抽象接口 
        
        // GNSS抽象接口  
        PositionCorrection<EstimatedStates, _dim_states> gnss_correction_; 
        // 用于初始化的imu数据 
        std::vector<Sensor::ImuDataConstPtr> prepared_imus_;  
        // 外参   
        Eigen::Matrix3d Rlb_;     // body->lidar的旋转   
        Eigen::Vector3d tlb_;     // body->lidar的平移
        Eigen::Matrix3d Ril_;     // lidar->imu的旋转   
        Eigen::Vector3d til_;     // lidar->imu的平移  
        Eigen::Matrix3d Rig_;     // gnss->imu的旋转   
        Eigen::Vector3d tig_;     // gnss->imu的平移 
  };  // class LidarImuFilterEstimatorWorldCentre

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename EstimatedStates, int _dim_states >
  LidarImuFilterEstimatorWorldCentre<EstimatedStates, _dim_states>::LidarImuFilterEstimatorWorldCentre( float const& imu_acc_noise, 
                                                                                                                                                                    float const& imu_gyro_noise, 
                                                                                                                                                                    float const& imu_acc_bias_noise, 
                                                                                                                                                                    float const& imu_gyro_bias_noise,
                                                                                                                                                                    std::shared_ptr<Model::ImuMotionModelInterface> imu_motion_model_ptr,
                                                                                                                                                                    Eigen::Matrix3d Rlb, 
                                                                                                                                                                    Eigen::Vector3d tlb, 
                                                                                                                                                                    Eigen::Matrix3d Ril, 
                                                                                                                                                                    Eigen::Vector3d til,
                                                                                                                                                                    Eigen::Matrix3d Rig, 
                                                                                                                                                                    Eigen::Vector3d tig) : Rlb_(Rlb), tlb_(tlb), Ril_(Ril), til_(til), Rig_(Rig), tig_(tig)
  {   
      // imu预测器初始化  
      imu_predict_ = ImuPredictor(imu_acc_noise, imu_gyro_noise, imu_acc_bias_noise, 
                                                                  imu_gyro_bias_noise, imu_motion_model_ptr);  
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename EstimatedStates, int _dim_states >
  bool LidarImuFilterEstimatorWorldCentre<EstimatedStates, _dim_states>::estimatorInitialize(double const& timestamp)
  {  
    // 判断IMU数据是否足够
    if(!initializer_.CheckInitializedImuDateNumber(prepared_imus_))  
    {
      std::cout<<"imu data is not enough!!!"<<std::endl;
      return false;  
    }
    
    // IMU数据中是否有旋转信息 
    bool rotation_active = true;  
    // 如果没有旋转信息   采用加速度进行初始化  
    if((*prepared_imus_.begin())->rot.w()==0&&
        (*prepared_imus_.begin())->rot.x()==0&&
        (*prepared_imus_.begin())->rot.y()==0&&
        (*prepared_imus_.begin())->rot.z()==0)
    {
      rotation_active = false;    
      std::cout<<"FilterEstimatorCentreRobot::Initialize ----- no rotation message !" << std::endl;
    }
    // 加速度buf
    std::vector<Eigen::Vector3d> acc_buf;  
    // 旋转buf
    std::vector<Eigen::Quaterniond> rot_buf;
    // 陀螺仪buf
    std::vector<Eigen::Vector3d> gyro_buf; 
    // 提取数据 
    for(auto const& imu : prepared_imus_)
    {
      acc_buf.push_back(std::move(imu->acc)); 
      gyro_buf.push_back(std::move(imu->gyro)); 
      if(!rotation_active)   
          continue;
      rot_buf.push_back(std::move(imu->rot));  
    }
    // 平滑后的等效IMU   用于初始化 
    ImuDataPtr initialized_imu = std::make_shared<ImuData>(); 
    initialized_imu->timestamp = timestamp;
    // 检查加速度是否满足静止初始化条件并求取平均IMU数据
    if(!initializer_.InitializeFristImuData(acc_buf, gyro_buf, rot_buf, initialized_imu->acc, 
                                              initialized_imu->gyro, initialized_imu->rot))
    {
      std::cout<<"motion excessive !! "<< std::endl;  
      return false; 
    }
    
    /***************************************************** 状态初始化 *****************************************************************/ 
    estimated_states_.Reset();
    estimated_states_.common_states_.timestamp_ = timestamp; 
    // 检查IMU是否可以获取旋转数据  
    Eigen::Matrix3d R_w_b;  
    if(!rotation_active)   // 靠重力初始化  
    {
      R_w_b = initializer_.ComputeRotationFromGravity(initialized_imu->acc);
      estimated_states_.common_states_.Q_ = Eigen::Quaterniond(R_w_b).normalized();   
      std::cout<<"imu rotation initialize ! R: "<< std::endl << R_w_b << std::endl;
    }
    else                   // 直接用旋转初始化 
    {         
    }
    // 陀螺仪偏执初始化 
    Eigen::Vector3d const& gyro_bias = initializer_.StaticInitializeGyroBias(gyro_buf); 
    estimated_states_.gyro_bias_ = gyro_bias;  
    // 估计器状态协方差矩阵初始化 
    estimatorStatesCovarianceInitialize(estimated_states_);
    // 预测器初始化
    imu_predict_.ImuPredictInitialize(initialized_imu);  

    return true;   
  }  

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename EstimatedStates, int _dim_states >
  void LidarImuFilterEstimatorWorldCentre<EstimatedStates, _dim_states>::estimatorStatesCovarianceInitialize(StatesWithImu &states)
  {
    states.cov_.setZero();
    // 状态初始协方差设置   
    states.cov_.block<3, 3>(0, 0) = 0.01 * Eigen::Matrix3d::Identity(); // position std: 0.1 m
    states.cov_.block<3, 3>(3, 3) = 0.01 * Eigen::Matrix3d::Identity(); // velocity std: 0.1 m/s
    // roll pitch std 10 degree.
    states.cov_.block<2, 2>(6, 6) = 10. * CoefDegreeToRadian * 10. * CoefDegreeToRadian * Eigen::Matrix2d::Identity();
    states.cov_(8, 8)             = 100. * CoefDegreeToRadian * 100. * CoefDegreeToRadian;     // yaw std: 100 degree.
    // Acc bias.
    states.cov_.block<3, 3>(9, 9) = 0.01 * 0.01 * Eigen::Matrix3d::Identity();   
    // Gyro bias.
    states.cov_.block<3, 3>(12, 12) = 0.0001 * 0.0001 * Eigen::Matrix3d::Identity();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename EstimatedStates, int _dim_states >
  void LidarImuFilterEstimatorWorldCentre<EstimatedStates, _dim_states>::ProcessSensorData(Sensor::ImuDataConstPtr &imuData) 
  { 
    // 只要gnss与lidar有一个初始化了  那么就进行滤波器的预测环节
    if(gnss_initialize_||lidar_initialize_)
    {
      std::cout<<"Imu predict !" <<std::endl;
      imu_predict_.ImuPredict(estimated_states_, imuData);
    }
    else
    { 
      prepared_imus_.push_back(std::move(imuData));       // 缓存数据用于初始化  
    }
  } 

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename EstimatedStates, int _dim_states >
  void LidarImuFilterEstimatorWorldCentre<EstimatedStates, _dim_states>::ProcessSensorData(Sensor::GnssDataConstPtr const& gnssData) 
  {
    // 如果gnss没有初始化 
    if(!gnss_initialize_)
    {  
      // 如果LIO没有初始化   那么先初始化INS 运行 gps与imu的组合导航 
      if(!lidar_initialize_)
      {    
        // 初始化GNSS 
        // 估计器初始化 
        if(estimatorInitialize(gnssData->timestamp))
        { 
          std::cout<<"estimator Initialize success!" <<std::endl;
          gnss_initialize_ = true;  
          // GNSS初始化 
          Sensor::GnssDataProcess* gnss_data_process = Sensor::GnssDataProcess::GetInstance(); 
          gnss_data_process->InitOriginPosition(gnssData->lla);  
        }  
      }
      else  // lidar初始化后的GNSS初始化   
      {   
      }
    }
    else   // 如果初始化了 执行滤波器校正  
    {
      // 首先将GNSS数据转换到ENU系下
      GnssDataProcess* gnss_processor_ptr = GnssDataProcess::GetInstance();  
      if(gnss_processor_ptr->UpdateXYZ(gnssData->lla))
      {
        Eigen::Vector3d position_obs = tig_ + (gnss_processor_ptr->GetEnuPosition()  
                                                    - estimated_states_.common_states_.Q_*tig_);
        gnss_correction_.Correct(position_obs, estimated_states_, gnssData->timestamp, gnssData->cov);  
        std::cout<<"estimator correct!" <<std::endl;
      } 
    }
  } 

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename EstimatedStates, int _dim_states >
  void LidarImuFilterEstimatorWorldCentre<EstimatedStates, _dim_states>::ProcessSensorData() 
  {
      if(!lidar_initialize_)    // 没初始化那么进行初始化 
      { 
        // 如果gnss没有初始化  则先初始化lidar&imu   运行lio
        if(!gnss_initialize_)
        {
            
        }
        else   // 进行gnss已经运行后的 LIO初始化    
        {
            
        } 
      }
  }  

} // namespace Estimator  
#endif