/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-18 18:21:53
 * @Description: 
 * @Others: 
 */

#ifndef  _LIDARODOMETRYDIRECTMETHOD_HPP_
#define  _LIDARODOMETRYDIRECTMETHOD_HPP_

#include "utility.hpp"
#include "LidarOdometry/LidarOdometryBase.hpp"
#include "Algorithm/PointClouds/registration/registration_adapter.hpp"
#include "Algorithm/PointClouds/processing/common_processing.hpp"

using Sensor::PointType;
using Sensor::LidarDataConstPtr;  

using PCLConstPtr = typename pcl::PointCloud<PointType>::ConstPtr;  
using PCLPtr = typename pcl::PointCloud<PointType>::Ptr;  
using PCLType = typename pcl::PointCloud<PointType>;  

/**
 * @brief:  基于局部子图的直接法激光里程计 
 * @details:  即不提取特征, 采用 NDT, GICP等方法 
 *                       不进行畸变去除，即传入的点云就默认是去除了畸变的
 *                       变化：  2、匹配算法 NDT、GICP等  3、匹配策略 - LOAM、sliding-window map   
 * @param PointType 仅仅考虑 对pcl::PointCloud<PointType> 类型的点云进行处理 
 */
class LidarOdometryDirectMethodLocalMap : public LidarOdometryBase<LidarDataConstPtr, Eigen::Matrix4d> 
{
    public:
        using BaseType = LidarOdometryBase<LidarDataConstPtr, Eigen::Matrix4d>;  
        using InputType = std::pair<PCLPtr, double>;
        /**
         * @brief: 
         * @param registration_ptr 匹配算法   
         * @param processing_ptr 点云处理模块 
         */        
        template<typename _RegistrationType>
        LidarOdometryDirectMethodLocalMap(std::unique_ptr<_RegistrationType> registration_ptr,
                        std::unique_ptr<Algorithm::PclCommonProcessing<PointType>> processing_ptr = nullptr)
        : processing_ptr_(std::move(processing_ptr)) 
        {
            registration_ptr_.reset(
                new Algorithm::RegistrationAdapterImpl<PCLConstPtr, _RegistrationType>(
                    std::move(registration_ptr))
            );  
            BaseType::calculate_thread_ = std::thread(&LidarOdometryDirectMethodLocalMap::calculate, this); 
        }

        /**
         * @brief:  将数据输入   
         * @details 
         */        
        void Input(LidarDataConstPtr const& cloud) override 
        {
            // 进行点云滤波   
            PCLPtr filtered;   
            processing_ptr_->Processing(cloud->point_clouds_, filtered);  
            // 存放到待处理队列中
            m_pcl_.lock();  
            input_cache_.push(std::make_pair(filtered, cloud->timestamp));   
            m_pcl_.unlock();  
        }

    protected:
        /**
         * @brief:  里程计运算线程  
         */        
        void calculate() 
        {
            while(true) 
            {
                std::unique_lock<std::mutex> u_lock(m_pcl_);
                if (!input_cache_.empty()) 
                {
                    PCLPtr cloud = input_cache_.front().first;
                    double timestamp = input_cache_.front().second;  
                    input_cache_.pop();   
                    u_lock.unlock();  
                    TicToc tt;  
                    // 第一帧时   进行初始化  
                    if(submap_ == nullptr) 
                    {
                        submap_.reset(new pcl::PointCloud<PointType>());  
                        prev_trans_.setIdentity();                         // 上一帧变换设置为单位阵
                        predict_trans_.setIdentity();                    // 预测位姿
                        motion_increment_.setIdentity();         // 运动增量  
                        // 当前帧加入滑窗 
                        *submap_ = *cloud;
                        FramesWin_.push_back(cloud);  
                        registration_ptr_->SetInputTarget(submap_);      // 将关键帧设置为匹配对象
                        DataCache(std::make_pair(Eigen::Matrix4d::Identity(), timestamp));  
                        continue;
                    }
                    tt.tic();  
                    // 每次匹配都与参考关键帧进行匹配
                    registration_ptr_->SetInputSource(cloud);
                    // 根据匀速运动假设进行预测    
                    predict_trans_ = BaseType::odom_*motion_increment_;         // Twb1*Tb1b2 = Twb2
                    registration_ptr_->Registration(predict_trans_);                   // 进行配准     predict_trans_ = setInputSource -> setInputTarget
                    float score = registration_ptr_->GetFitnessScore();               // 获得得分
                    // 如果迭代没收敛   则该帧忽略    匹配的协方差应该设置很大 
                    if (!registration_ptr_->HasConverged()) 
                    {
                        DataCache(std::make_pair(predict_trans_, timestamp));       // 结果缓存  
                        continue;        // 返回预测位姿
                    } 
                    else 
                    {
                        //  检查得分或者匹配的协方差矩阵
                    }
                    // 收敛的话   注意PCL中的T 是B -> W
                    BaseType::odom_ = registration_ptr_->GetFinalTransformation();          // trans为当前帧->参考关键帧的变换矩阵
                    DataCache(std::make_pair(BaseType::odom_, timestamp));  
                    // std::cout<<" odom_ : "<<std::endl<<trans<<std::endl;
                    // Tb1w*Twb2 = Tb1b2
                    motion_increment_ = prev_trans_.inverse()*BaseType::odom_;                               // 前两帧位姿的增量   用于预测下一帧位姿
                    // 判断是否重新设置关键帧
                    double delta_trans = motion_increment_.block<3, 1>(0, 3).norm();         
                    // 旋转矩阵对应 u*theta  对应四元数  e^u*theta/2  = [cos(theta/2), usin(theta/2)]
                    Eigen::Quaterniond q_trans(motion_increment_.block<3, 3>(0, 0));
                    q_trans.normalize();   
                    double delta_angle = std::acos(q_trans.w())*2;     // 获得弧度    45度 约等于 0.8  
                    // 满足关键帧条件
                    if (delta_trans > THRESHOLD_TRANS || delta_angle > THRESHOLD_ROT) 
                    {
                        // 变换点云到odom坐标 
                        // 执行变换，并将结果保存在新创建的‎‎ transformed_cloud ‎‎中
                        typename pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType>());
                        pcl::transformPointCloud (*cloud, *transformed_cloud, BaseType::odom_);
                        // 更新滑动窗口  
                        if (FramesWin_.size()>=WINDOW_SIZE) 
                        {  
                            tt.tic();  
                            FramesWin_.pop_front();
                            FramesWin_.push_back(std::move(transformed_cloud));
                            submap_->clear();   
                            // 更新submap  
                            for (typename std::deque<PCLConstPtr>::const_iterator it = FramesWin_.begin(); 
                                        it != FramesWin_.end(); it++) 
                            {
                                *submap_ += **it;   
                            }
                            //tt.toc("map slidingwindow: ");  
                        } 
                        else  
                        {//滑窗没有满
                            *submap_ += *transformed_cloud;      
                            FramesWin_.push_back(std::move(transformed_cloud));   
                        }
                        // 降采样 
                        if(FramesWin_.size() >= 5) 
                        {
                        }
                        tt.tic();  
                        registration_ptr_->SetInputTarget(submap_);   // 设定为匹配目标
                        //tt.toc("map construct: ");  
                        // 记录
                        prev_trans_ = BaseType::odom_;   
                    }
                }
                // 10us的延时  
                std::chrono::microseconds dura(10);
                std::this_thread::sleep_for(dura);
            }
        }

    private:
        std::queue<InputType> input_cache_;     // 输入的缓存队列   
        // odometry calculation
        Eigen::Matrix4d prev_trans_;                  // previous estimated transform from keyframe  上一帧相对于参考关键帧的位姿
        Eigen::Matrix4d predict_trans_;               // 预测位姿
        Eigen::Matrix4d motion_increment_;     // 运动增量 
        Eigen::Matrix4d keyframe_pose_;               // keyframe pose      参考关键帧的位姿
        bool start_ = true;
        typename pcl::PointCloud<PointType>::Ptr submap_ = nullptr;  // keyframe point cloud  用来匹配的参考子图 
        std::deque<PCLConstPtr> FramesWin_;// 子图滑窗
        std::unique_ptr<Algorithm::RegistrationAdapterBase<PCLConstPtr>> registration_ptr_;  // 匹配算法 
        std::unique_ptr<Algorithm::PclCommonProcessing<PointType>> processing_ptr_;      // 点云预处理算法
        uint16_t WINDOW_SIZE = 1;    // 滑窗size  
        float THRESHOLD_TRANS = 1;   // 1m
        float THRESHOLD_ROT = 0.5;    // 30度 
        std::mutex m_pcl_;  
}; // class LidarOdometryDirectMethodLocalMap

/**
 * @brief:  基于LOAM策略的直接法激光里程计 
 * @details:  即不提取特征, 采用 NDT, GICP等方法 
 *                       变化： 1、是否进行畸变去除  2、匹配算法 NDT、GICP等  3、匹配策略 - LOAM、sliding-window map   
 */


#endif   

