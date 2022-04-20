/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-28 12:46:49
 * @Description: 
 * @Others: 
 */

#pragma once 
#include "filter_base.hpp"
#include "factory/processing/pointcloud/filter/filter_factory.hpp"
namespace Algorithm
{
    template<typename _PointType>
    class OutLierRemovalFilter : public FilterBase<_PointType>
    {
        public:
            OutLierRemovalFilter() {}
            
            template<typename... _ParamType>
            void Reset(string const &name, _ParamType... params)
            {
                if (name == "radiusOutlierRemoval")
                {
                    FilterBase<_PointType>::SetFilter(
                        make_radiusOutlierRemoval<_PointType>(params...)); 
                }
                else if (name == "statisticalOutlierRemoval")
                {
                    FilterBase<_PointType>::SetFilter(
                            make_statisticalOutlierRemoval<_PointType>(params...)); 
                }
            }
    };
}
