/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-24 12:21:42
 * @Description: 点云匹配前的预处理基类 
 * @Others: 
 */

#ifndef _PROCESSING_BASE_HPP_
#define _PROCESSING_BASE_HPP_

namespace Algorithm {
    /**
     * @brief: 点云处理基类  
     */    
    template<typename _InputType, typename _OutputType>
    class PointCloudProcessingBase {
        public:
            virtual ~PointCloudProcessingBase() {}

            virtual void Processing(_InputType const& data_in, _OutputType &data_out) = 0;  
    };
};
#endif
