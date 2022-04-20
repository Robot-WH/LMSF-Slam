/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-20 16:25:49
 * @Description: 
 * @Others: 
 */

#ifndef _SENSOR_PROCESS_INTERFACE_HPP_
#define _SENSOR_PROCESS_INTERFACE_HPP_

namespace Slam3D {

    /**
     * @brief:  提供对于各种传感器数据处理的接口
     */
    template<typename... _DataTypes>
    class SensorProcessInterface {};   
    /**
     * @brief:  特化  
     */
    template<typename _CurrentDataType, typename... _OtherDataTypes>
    class SensorProcessInterface<_CurrentDataType, _OtherDataTypes...> 
    : public SensorProcessInterface<_OtherDataTypes...> {
        public:
            /**
             * @brief 传感器数据处理接口  
             */
            virtual void Process(_CurrentDataType const& data) = 0;     
    }; // class SensorProcessInterface 

    /**
     * @brief:  特化  
     */
    template<typename _CurrentDataType>
    class SensorProcessInterface<_CurrentDataType> {
        public:
            virtual ~SensorProcessInterface() {}
            /**
             * @brief 传感器数据处理接口  
             */
            virtual void Process(_CurrentDataType const& data) = 0;     
    }; // class SensorProcessInterface 
}; // namespace Sensor 

#endif
