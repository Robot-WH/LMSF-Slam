/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: lwh
 * @Version: 版本
 * @Date: 2022-02-17 00:22:22
 * @Description: 管理全部数据    全局&单例 
 * @Others: 
 */

#pragma once 

#include <typeindex>
#include "tool.hpp"

namespace common {

/**
 * @todo:  目前仅仅支持一对一，而不支持一对多，目前是每读取一次队列就丢掉读取的数据，
 *                    这样多个对象去读取的话，每个对象读取的数据都会有所缺失。 
 *                  因此，应该改为，保存每个对象的读取位置指针，位置指针记录该对象当前读取的位置，
 *                  每个对象进行读取的时候，对位置指针进行移动，而不是直接将读取到的数据从缓存队列中
 *                   丢掉。
 */    

    /**
     * @brief: 数据容器对外接口类  
     * @details:  非模板的万能类  
     */    
    class DataContainer
    {
        public:
            DataContainer(uint16_t const& capacity) : capacity_(capacity) {}
            virtual ~DataContainer() {}
            virtual std::type_index GetDataType() const = 0; 
        protected:
            uint16_t capacity_;
    };
    /**
     * @brief: 实际存储数据的类   保存的类型由模板参数指定  
     */    
    template<typename _DataT>
    class DataContainerImpl : public DataContainer
    {
        public:
            DataContainerImpl(uint16_t const& capacity) : DataContainer(capacity), 
                                                                                                              type_info_(typeid(_DataT))
            {}
            // 获取数据类型  
            std::type_index GetDataType() const override
            {
                return type_info_;  
            }
            /**
             * @brief: 在队列最末尾添加数据 
             */             
            template<typename _DataType>
            bool AddData(_DataType&& data)
            {
                // 检查类型是否一致
                if (type_info_ != std::type_index(typeid(_DataType))) 
                {
                    throw std::bad_cast();  
                }
                data_m_.lock();  
                if (data_buffer_.size() >= capacity_)
                {
                    data_buffer_.pop_front();   
                }
                data_buffer_.push_back(std::forward<_DataType>(data));
                data_m_.unlock();  
                return true;  
            }
            /**
             * @brief:  在队列头部读取数据 
             */            
            bool GetData(_DataT &data)
            {
                data_m_.lock();  
                if (data_buffer_.empty())
                {
                    //std::cout<<"data_buffer_.empty() !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                    data_m_.unlock();  
                    return false;
                }
                data = std::move(data_buffer_.front()); 
                data_buffer_.pop_front();  
                data_m_.unlock();  
                return true;  
            }

            auto GetBufferSize() const
            {
                return data_buffer_.size();  
            }
        private:
            std::mutex data_m_;   // 加了mutex后变成了只移型别   (禁止了拷贝构造)
            std::deque<_DataT> data_buffer_;    
            std::type_index type_info_;  // 数据类型信息 
    };

    /**
     * @brief: 数据管理器  
     * @details:  负责系统各种类型数据的保存，读取   
     */    
    class DataManager
    {
        public:
            /**
             * @brief: 单例的创建函数  
             */            
            static DataManager& GetInstance()
            {
                static DataManager data_manager;
                return data_manager; 
            }

            /**
             * @brief: 注册一个指定类型与标识名的数据容器 
             * @details: 
             * @param name 数据标识名
             * @param capacity 缓存容量   
             */            
           template<typename _DataT>
            bool Registration(std::string const& name, uint16_t const& capacity)
            {   
                // 如果重复则插入失败 
                if (data_container_store_.find(name) != data_container_store_.end())
                    return false;  
                data_container_store_.insert(std::make_pair(name, 
                    std::unique_ptr<DataContainer>(new DataContainerImpl<_DataT>(capacity))));
            }

            /**
             * @brief: 向当前数据管理器添加数据
             * @param[in] name 数据的标识名
             * @param[in] data 加入的数据 
             * @return {*}
             */            
            template<typename _T>
            bool AddData(std::string const& name, _T&& data)
            {
                if (data_container_store_.find(name) == data_container_store_.end()) {
                    //std::cerr<<"AddData ERROR! name:"<<name<<"  not match "<<std::endl;
                    return false;  
                }
                // 首先判断类型是否一致
                if (data_container_store_[name]->GetDataType() != std::type_index(typeid(_T)))
                {
                    //std::cerr<<"DataManager AddData() Type ERROR !!!"<<std::endl;
                    throw std::bad_cast();  
                }

                DataContainerImpl<remove_reference_t<_T>>* data_container_ptr =  
                    dynamic_cast<DataContainerImpl<remove_reference_t<_T>>*>(
                        data_container_store_.at(name).get());
                data_container_ptr->AddData(std::forward<_T>(data)); 
                //std::cout<<"Add Data success! name:"<<name<<" size: "<<
                //data_container_ptr->GetBufferSize()<<std::endl;
                return true;
            }

            /**
             * @brief: 数据获取
             * @details: 
             * @param name 数据容器的标识名
             * @param[out] data 获取的数据   必须是左值   
             * @return {*}
             */            
            template<typename _T>
            bool GetData(std::string const& name, _T &data) const
            {
                // 检查有没有这个数据
                if (data_container_store_.find(name) == data_container_store_.end()) {
                    //std::cerr<<"data_container not find name: "<<name<<std::endl;
                    return false;  
                }
                // 判断类型是否一致
                if (data_container_store_.at(name)->GetDataType() != std::type_index(typeid(_T)))
                {
                    //std::cerr<<"DataManagerAddData Type ERROR !!!"<<std::endl;
                    throw std::bad_cast();  
                }

                DataContainerImpl<_T>* data_container_ptr =  
                    dynamic_cast<DataContainerImpl<_T>*>(data_container_store_.at(name).get());
                return data_container_ptr->GetData(data);  
            }

        protected:
            DataManager() {}
            DataManager(DataManager const& object) {}
            DataManager(DataManager&& object) {}
        private:
            std::unordered_map<std::string, 
                std::unique_ptr<DataContainer>> data_container_store_;  
    }; // class 
} // namespace 