/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-17 12:40:18
 * @Description:  匹配算法的适配器 
 * @Others: 
 */

#ifndef _REGISTRATION_ADAPTER_HPP_
#define _REGISTRATION_ADAPTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <eigen3/Eigen/Dense>
#include "Sensor/lidar_data_type.h"

namespace Algorithm {

    using Slam3D::FeaturePointCloudContainer;
    
    /**
     * @brief: 3D点云匹配的基类
     * @details:  外部通过此类的接口调用配准算法
     * @param _PointCloudType 传入的点云的类型  
     */    
    template<typename _SourceType, typename _TargetType>
    class RegistrationAdapterBase 
    {
        public:
            virtual ~RegistrationAdapterBase() {}
            virtual bool Registration(Eigen::Isometry3d &predict) {};  
            virtual bool HasConverged() = 0;   
            virtual Eigen::Matrix4d GetFinalTransformation() = 0;  
            virtual void SetInputSource(_SourceType const& source) {};
            virtual void SetInputTarget(_TargetType const& target) {};  
            virtual float GetFitnessScore() {}
            virtual Eigen::MatrixXd GetCovariance() {}  
            virtual std::vector<std::string> GetUsedPointsName() = 0;  
    }; // class RegistrationImpl

    /**
     * @brief: 通用RegistrationImpl 模板类 
     * @details: 相比于基类 RegistrationAdapterBase ，
     *                      该子类可以通过模板_RegistrationType设置任意的匹配算法
     * @return {*}
     */    
    template<typename _SourceType, typename _TargetType, typename _RegistrationType>
    class RegistrationAdapterImpl : public RegistrationAdapterBase<_SourceType, _TargetType> 
    {
        public:
            using RegistrationPtr = std::unique_ptr<_RegistrationType>;  
            RegistrationAdapterImpl(RegistrationPtr registration_ptr) 
                : registration_ptr_(std::move(registration_ptr)) 
            {
                std::cout<<"construct un-specialization RegistrationImpl object "<<std::endl;
            }
            virtual ~RegistrationAdapterImpl() {}
            virtual void SetInputSource(_SourceType const&source) override 
            {
                registration_ptr_->SetInputSource(source); 
            }
            virtual void SetInputTarget(_TargetType const& target) override 
            {
                registration_ptr_->SetInputTarget(target);
            }  
            virtual bool Registration(Eigen::Isometry3d &predict) override 
            {
                registration_ptr_->Solve(predict); 
                return true;  
            }
            virtual bool HasConverged() override 
            {
            }
            virtual Eigen::Matrix4d GetFinalTransformation() override 
            {
            }
            virtual float GetFitnessScore() override {
            }
            virtual Eigen::MatrixXd GetCovariance() override {
            }
            std::vector<std::string> GetUsedPointsName() override {

            }
        private:
            RegistrationPtr  registration_ptr_;  
    }; // class RegistrationAdapterImpl

    /**
     * @brief: 针对pcl::Registration的特化 
     * @details:  当使用的点云为 PCL ， 以及PCL 匹配库时, 
     *                       如NDT，GICP  等直接法
     */    
    template<typename _SourceType, typename _PointType>
    class RegistrationAdapterImpl<_SourceType, FeaturePointCloudContainer<_PointType>, 
                                                                        pcl::Registration<_PointType, _PointType>> 
    : public RegistrationAdapterBase<_SourceType, FeaturePointCloudContainer<_PointType>> 
    {
        public:
            using  RegistrationPtr = std::unique_ptr<pcl::Registration<_PointType, _PointType>>;  
            RegistrationAdapterImpl(RegistrationPtr registration_ptr, 
                                                                std::string const& specified_points_name = "") 
            : registration_ptr_(std::move(registration_ptr)), specified_points_name_(specified_points_name)
            {
                std::cout<<"construct RegistrationImpl object - pcl::Registration"<<std::endl;
            }
            virtual ~RegistrationAdapterImpl() {}
            // specified_points_name_ 没有限制 ，因为 对于ndt或gicp，因该任何点云都要能使用
            virtual void SetInputSource(_SourceType const& source) 
            {
                // 如果没有指定点云    那么设置默认点云名   本次就与该点云进行匹配
                if (specified_points_name_ == "") {
                    default_points_name_ = source.first;
                } else if (specified_points_name_ != source.first) return;  
                // 在 pcl 里 target 和 source 的概念和这个代码里相反 
                registration_ptr_->setInputTarget(source.second);
                // std::cout<<common::YELLOW<<"setInputSource, size: "<<source.second->size()
                // <<"specified_points_name_: "<<specified_points_name_<<common::RESET<<std::endl;
            }

            virtual void SetInputTarget(FeaturePointCloudContainer<_PointType> const& target) 
            {   
                // 从特征点云中 找到想要的点云
                typename FeaturePointCloudContainer<_PointType>::const_iterator iter;
                if (specified_points_name_ == "") {
                    iter = target.find(default_points_name_); 
                } else {
                    iter = target.find(specified_points_name_); 
                }
                if (iter != target.end())
                {
                    //std::cout<<"setInputTarget, size: "<<iter->second->size()<<std::endl;
                    registration_ptr_->setInputSource(iter->second); 
                }
            }

            virtual bool Registration(Eigen::Isometry3d &predict) override 
            {
                typename pcl::PointCloud<_PointType>::Ptr aligned(new pcl::PointCloud<_PointType>());
                registration_ptr_->align(*aligned, predict.matrix().cast<float>());        // 进行配准     predict_trans = setInputSource -> setInputTarget
                //std::cout<<"GetFitnessScore :"<<GetFitnessScore()<<std::endl;
                if (!HasConverged()) return false;
                Eigen::Matrix4f T = registration_ptr_->getFinalTransformation();  
                predict.translation() =T.cast<double>().block<3, 1>(0, 3);  
                predict.linear() = T.cast<double>().block<3, 3>(0, 0); 
                return true;  
            }

            virtual bool HasConverged() override 
            {
                return registration_ptr_->hasConverged();  
            }

            virtual Eigen::Matrix4d GetFinalTransformation() override 
            {
                Eigen::Matrix4f matrix_f = registration_ptr_->getFinalTransformation();  
                return matrix_f.cast<double>();  
            }

            virtual float GetFitnessScore() override 
            {
                return registration_ptr_->getFitnessScore();  
            }

            virtual Eigen::MatrixXd GetCovariance() override 
            {
            }

            std::vector<std::string> GetUsedPointsName() override 
            {
                return std::vector<std::string>{specified_points_name_};
            }
        private:
            string specified_points_name_;   // 指定使用的点云名
            string default_points_name_;
            //string prefer_name_;
            RegistrationPtr  registration_ptr_;  
    }; // class RegistrationAdapterImpl
} // namespace Algorithm 

#endif 
