/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-18 18:21:53
 * @Description: 
 * @Others: 
 */

#ifndef  _LIDARODOMETRYBASE_HPP_
#define  _LIDARODOMETRYBASE_HPP_

#include "utility.hpp"

template<typename _InputDataType, typename _OdomType>
class LidarOdometryBase {
    public:
        using ResultType = std::pair<_OdomType, double>;    //  结果类型   数据 + 时间戳  
        virtual ~LidarOdometryBase() {}
        virtual void Input(_InputDataType const& cloud) = 0;

        bool GetResult(ResultType &result) const {
            if (cache_.empty()) return false;
            result = cache_.front();
            cache_.pop();  
            return true;  
        }

    protected:
        void  calculate() {}    

        /**
         * @brief:  将当前的里程计数据进行缓存
         * @param {uint16_t} max_n
         */        
        void DataCache(ResultType const& result, uint16_t max_n = 10) {
            // 要缓存的数据应该之前没有被缓存
            if (result.second == cache_.back().second)  return;  
            if (cache_.size()>= max_n) {
                    cache_.pop();    
            }
            cache_.push(result);  
        }
    
    protected:
        _OdomType odom_;  
        std::thread calculate_thread_;
    private:
        std::queue<ResultType> cache_;  
}; // class LidarOdometryBase 

#endif   

