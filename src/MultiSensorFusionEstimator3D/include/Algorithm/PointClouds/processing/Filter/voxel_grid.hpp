/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-02-28 12:08:07
 * @Description: 
 * @Others: 
 */

#pragma once 

#include "filter_base.hpp"
#include "factory/processing/pointcloud/filter/filter_factory.hpp"

namespace Algorithm
{
    template<typename _PointType>
    class VoxelGridFilter : public FilterBase<_PointType>
    {
        public:
            VoxelGridFilter() {}
            
            template<typename... _ParamType>
            void Reset(string const &name, _ParamType... params)
            {
                if (name == "VoxelGrid")
                {
                    FilterBase<_PointType>::SetFilter(make_voxelGrid<_PointType>(params...)); 
                }
                else if (name == "ApproximateVoxelGrid")
                {
                }
            }
    };
}

