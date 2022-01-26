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

#include "utility.hpp"

namespace Algorithm {

    /**
     * @brief: 3D点云匹配的基类
     * @details:  外部通过此类的接口调用配准算法
     * @param {*}
     */    
    template<typename _PointCloudType>
    class RegistrationAdapterBase {
        public:
            virtual ~RegistrationAdapterBase() {}
            virtual void Registration(Eigen::Matrix4d const& predict) {};  
            virtual void Registration(_PointCloudType const& target, _PointCloudType const& source, 
                                                                Eigen::Matrix4d const& predict) {};  
            virtual bool HasConverged() = 0;   
            virtual Eigen::Matrix4d GetFinalTransformation() = 0;  
            virtual void SetInputSource(_PointCloudType const& source) {};
            virtual void SetInputTarget(_PointCloudType const& target) {};  
            virtual float GetFitnessScore() {}
            virtual Eigen::MatrixXd GetCovariance() {}  
    }; // class RegistrationImpl

    // 通用RegistrationImpl 模板类 
    template<typename _PointCloudType, typename _RegistrationType>
    class RegistrationAdapterImpl : public RegistrationAdapterBase<_PointCloudType> {
        public:
            using RegistrationPtr = std::unique_ptr<_RegistrationType>;  
            RegistrationAdapterImpl(RegistrationPtr registration_ptr) : registration_ptr_(std::move(registration_ptr)) {
                std::cout<<"construct un-specialization RegistrationImpl object "<<std::endl;
            }
            virtual ~RegistrationAdapterImpl() {}
            virtual void SetInputSource(_PointCloudType const& source) override {
            }
            virtual void SetInputTarget(_PointCloudType const& target) override {
            }  
            virtual void Registration(Eigen::Matrix4d const& predict) override {
            }
            virtual void Registration(_PointCloudType const& target, _PointCloudType const& source, 
                                                                Eigen::Matrix4d const& predict) override {
            }
            virtual bool HasConverged() override {
            }
            virtual Eigen::Matrix4d GetFinalTransformation() override {

            }
            virtual float GetFitnessScore() override {
            }
            virtual Eigen::MatrixXd GetCovariance() override {
            }
        private:
            RegistrationPtr  registration_ptr_;  
    }; // class RegistrationAdapterImpl

    template<typename _PointType>
    using PclConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;  
    // 针对pcl::Registration的特化 
    template<typename _PointType>
    class RegistrationAdapterImpl<PclConstPtr<_PointType>, pcl::Registration<_PointType, _PointType>> 
    : public RegistrationAdapterBase<PclConstPtr<_PointType>> {
        public:
            using  RegistrationPtr = std::unique_ptr<pcl::Registration<_PointType, _PointType>>;  
            RegistrationAdapterImpl(RegistrationPtr registration_ptr) : registration_ptr_(std::move(registration_ptr)) {
                std::cout<<"construct RegistrationImpl object - pcl::Registration"<<std::endl;
            }
            virtual ~RegistrationAdapterImpl() {}

            virtual void SetInputSource(PclConstPtr<_PointType> const& source) {
                registration_ptr_->setInputSource(source);
            }

            virtual void SetInputTarget(PclConstPtr<_PointType> const& target) {
                registration_ptr_->setInputTarget(target); 
            }

            virtual void Registration(Eigen::Matrix4d const& predict) override {
                typename pcl::PointCloud<_PointType>::Ptr aligned(new pcl::PointCloud<_PointType>());
                registration_ptr_->align(*aligned, predict.cast<float>());        // 进行配准     predict_trans = setInputSource -> setInputTarget
            }

            virtual void Registration(PclConstPtr<_PointType> const& target, PclConstPtr<_PointType> const& source, 
                                                                Eigen::Matrix4d const& predict) override {
                registration_ptr_->setInputTarget(target);
                registration_ptr_->setInputSource(source);                                                    
                typename pcl::PointCloud<_PointType>::Ptr aligned(new pcl::PointCloud<_PointType>());
                registration_ptr_->align(*aligned, predict.cast<float>());        // 进行配准     predict_trans = setInputSource -> setInputTarget
            }

            virtual bool HasConverged() override {
                return registration_ptr_->hasConverged();  
            }

            virtual Eigen::Matrix4d GetFinalTransformation() override {
                Eigen::Matrix4f matrix_f = registration_ptr_->getFinalTransformation();  
                return matrix_f.cast<double>();  
            }

            virtual float GetFitnessScore() override {
                return registration_ptr_->getFitnessScore();  
            }

            virtual Eigen::MatrixXd GetCovariance() override {
            }
        private:
            RegistrationPtr  registration_ptr_;  
    }; // class RegistrationAdapterImpl
}; // namespace Algorithm 

#endif 
