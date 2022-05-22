#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

#include "tic_toc.h"


namespace Algorithm
{
    using namespace Eigen;
    using namespace nanoflann;

    template<typename _PointT>
    class ScanContext
    {
        protected:
            using KeyMat = std::vector<std::vector<float> >;
            using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;
        public:
            /**
             * @todo 应该不需要这个参数  
             */            
            const double LIDAR_HEIGHT_ = 2.0; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.
            const int    PC_NUM_RING_ = 20; // 20 in the original paper (IROS 18)
            const int    PC_NUM_SECTOR_ = 60; // 60 in the original paper (IROS 18)
            const double PC_MAX_RADIUS_ = 80.0; // 80 meter max in the original paper (IROS 18)
            const double PC_UNIT_SECTORANGLE_ = 360.0 / double(PC_NUM_SECTOR_);
            const double PC_UNIT_RINGGAP_ = PC_MAX_RADIUS_ / double(PC_NUM_RING_);
             // for fast comparison, no Brute-force, but search 10 % is okay. 
             // not was in the original conf paper, but improved ver.
            const double ROT_SEARCH_RATIO_ = 0.1;  // 旋转细化搜索率
            
        public:
            ScanContext() = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.

            Eigen::MatrixXd MakeScanContext(pcl::PointCloud<_PointT> const& _scan_down)
            {
                TicToc t_making_desc;
                int num_pts_scan_down = _scan_down.points.size();
                // main
                const int NO_POINT = -1000;     // 矩阵每一个的初值  
                MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING_, PC_NUM_SECTOR_);      // 构造一个二维矩阵 即sc描述子  行为ring的数量  列为sector数量 

                _PointT pt;    // pcl::PointXYZI 
                float azim_angle, azim_range; // wihtin 2d plane
                int ring_idx, sector_idx;
                // 遍历每一个点  
                for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
                {
                    // 获取每一个点的坐标  
                    pt.x = _scan_down.points[pt_idx].x; 
                    pt.y = _scan_down.points[pt_idx].y;
                    pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT_; // naive adding is ok (all points should be > 0).
                    // xyz to ring, sector
                    azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);     // R的值  决定ring的值
                    azim_angle = xy2theta(pt.x, pt.y);                // 决定 sector 
                    // if range is out of roi, pass
                    if( azim_range > PC_MAX_RADIUS_ )
                        continue;
                    // 求ring和sector的idx       ceil(x) 大于 x 的 最小整数 
                    ring_idx = std::max( std::min( PC_NUM_RING_, int(ceil( (azim_range / PC_MAX_RADIUS_) * PC_NUM_RING_ )) ), 1);   // 范围  [1, PC_NUM_RING_]
                    sector_idx = std::max( std::min( PC_NUM_SECTOR_, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR_ )) ), 1);     // 范围  [1, PC_NUM_SECTOR_]
                    // sc描述子 保存对应位置的最大z轴高度值   赋值  给sc描述符 
                    // 减1 是因为 行和列的起始位置是0 
                    if ( desc(ring_idx-1, sector_idx-1) < pt.z ) // -1 means cpp starts from 0
                        desc(ring_idx-1, sector_idx-1) = pt.z; // update for taking maximum value at that bin
                }
                // reset no points to zero (for cosine dist later)   如果有的desc的部分没有赋值的话   设置为 0   
                // 这里不能在初始化的时候直接赋值为0   因为 sc描述子在取值的时候  会去最大值作为当前值   如果点云高度为负数的话  初始化为0 就会得到错误的结果  
                for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
                {
                    for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
                    {
                        if( desc(row_idx, col_idx) == NO_POINT )
                            desc(row_idx, col_idx) = 0;
                    }
                }

                t_making_desc.toc("PolarContext making");
                return desc;
            }

            /**
         * @brief 构造ring key
         * @param[in] _desc SC描述符
         * @details 对每个SC描述符的每一行求均值  
         * @return ringkey 描述符   列向量  
         **/
            Eigen::MatrixXd MakeRingkeyFromScanContext(Eigen::MatrixXd const& _desc)
            {
                /* 
                * summary: rowwise mean vector
                */
                Eigen::MatrixXd invariant_key(_desc.rows(), 1);           // 一个列向量    列数为ring 的个数  
                // 遍历SC描述子每一行  
                for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
                {
                    Eigen::MatrixXd curr_row = _desc.row(row_idx);        // 获得第 row_idx 行 
                    invariant_key(row_idx, 0) = curr_row.mean();          // 求均值 
                }

                return invariant_key;
            }
            
            /**
             * @brief 对两个sc描述子求取相似性
             * @details 对两个sc描述子进行旋转   并求出距离最相似的得分与旋转量   
             * @return (最小相似得分, 旋转距离)
             **/
            std::pair<double, int> DistanceBtnScanContext(MatrixXd const& _sc1, MatrixXd const& _sc2)
            {
                // 1. fast align using variant key (not in original IROS18)
                // 先求出 Sectorkey    用来计算描述子之间的平移量   
                MatrixXd vkey_sc1 = makeSectorkeyFromScanContext( _sc1 );
                MatrixXd vkey_sc2 = makeSectorkeyFromScanContext( _sc2 );
                int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );   // 快速求出平移量    粗略值   
                // 在 argmin_vkey_shift 附近进行校准 
                const int SEARCH_RADIUS = round( 0.5 * ROT_SEARCH_RATIO_ * _sc1.cols() ); // a half of search range    ROT_SEARCH_RATIO_ = 0.1  
                std::vector<int> shift_idx_search_space { argmin_vkey_shift };
                // 将平移的范围求出  
                for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
                {
                    /**
                     * @todo  ??????? 不需要 + _sc1.cols() ？？？     
                     */                    
                    shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());  
                    shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
                }
                /**
                 * @todo  有啥用 ？？？？？？？？？？？？？/
                 */                
                std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());    
                // 2. fast columnwise diff 
                int argmin_shift = 0;
                double min_sc_dist = 10000000;
                // 遍历 平移量 shift_idx_search_space 
                for ( int num_shift: shift_idx_search_space )
                {
                    MatrixXd sc2_shifted = circshift(_sc2, num_shift);        // 平移
                    double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );   // 按照论文求距离 
                    if ( cur_sc_dist < min_sc_dist )
                    {
                        argmin_shift = num_shift;
                        min_sc_dist = cur_sc_dist;
                    }
                }

                return std::make_pair(min_sc_dist, argmin_shift);
            }

            uint16_t GetRingNum() const
            {
                return PC_NUM_RING_; 
            }

            uint16_t GetSectorKeyNum() const
            {
                return PC_NUM_SECTOR_; 
            }

        protected:

            /**
             * @brief 构造sector key 与原文不同 , 主要用于求取 旋转平移量 
             * @details 用于估计旋转的偏移  
             * @param[in] desc SC描述符
             * @details sector key 即将SC描述子每一列的向量求均值   
             **/
            Eigen::MatrixXd makeSectorkeyFromScanContext( Eigen::MatrixXd const& desc )
            {
                /* 
                * summary: columnwise mean vector
                */
                Eigen::MatrixXd variant_key(1, desc.cols());
                for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
                {
                    Eigen::MatrixXd curr_col = desc.col(col_idx);
                    variant_key(0, col_idx) = curr_col.mean();
                }

                return variant_key;
            }

            /**
             * @brief 计算两个sc描述子的距离   参考论文 (5) 式
             * @details 计算每一列的余弦距离  越小越接近 , 然后将所有列的余弦距离加起来得到得分
             *                    在 distanceBtnScanContext 中使用  
             * @return 接近程度  越小越接近
             **/ 
            double distDirectSC (MatrixXd const& _sc1, MatrixXd const& _sc2 ) // "d" (eq 5) in the original paper (IROS 18)
            {
                int num_eff_cols = 0;         // 有效的列数  
                double sum_sector_similarity = 0;
                // 遍历每一列  单独对每一列求余弦距离              
                for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
                {
                    VectorXd col_sc1 = _sc1.col(col_idx);
                    VectorXd col_sc2 = _sc2.col(col_idx);
                    // | == ||    有一个为0  则continue!  
                    if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
                        continue; // don't count this sector pair. 
                    // 相似度计算   余弦距离 
                    double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

                    sum_sector_similarity = sum_sector_similarity + sector_similarity;
                    num_eff_cols++;
                }
                
                double sc_sim = sum_sector_similarity / num_eff_cols;
                return 1.0 - sc_sim;      // 越小越接近  
            } 

            /**
             * @brief: 根据两个sector key 快速求解旋转的差异  
             * @details  distanceBtnScanContext() 中使用  
             * @param {MatrixXd &} _vkey1
             * @param {MatrixXd &} _vkey2
             * @return 平移量   大于 0 
             */            
            uint16_t fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 )
            {
                uint16_t argmin_vkey_shift = 0;
                double min_veky_diff_norm = 10000000;
                // 对这个一维向量进行平移 找出让这两个向量最接近的平移量   
                for ( uint16_t shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
                {   
                    // 对向量进行平移  
                    MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
                    // 求平移后与目标的误差  找出误差最小的
                    MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
                    double cur_diff_norm = vkey_diff.norm();
                    if( cur_diff_norm < min_veky_diff_norm )
                    {   
                        argmin_vkey_shift = shift_idx;     // 记录误差最小的那个平移  
                        min_veky_diff_norm = cur_diff_norm;
                    }
                }
            
                return argmin_vkey_shift;
            }

            /**
             * @brief 对 _mat 的列   进行 _num_shift的平移
             * @param[in] _num_shift 平移量
             **/
            MatrixXd circshift( MatrixXd const& _mat, int _num_shift )
            {
                // shift columns to right direction 
                assert(_num_shift >= 0);

                if( _num_shift == 0 )
                {
                    MatrixXd shifted_mat( _mat );
                    return shifted_mat; // Early return 
                }
                // 先构造一个临时的矩阵  用来保存平移后的量  
                MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
                // 对每一列的数据进行平移   
                for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
                {
                    int new_location = (col_idx + _num_shift) % _mat.cols();    // 求出平移这么多后处于什么位置 
                    shifted_mat.col(new_location) = _mat.col(col_idx);
                }

                return shifted_mat;
            } 

            /**
             * @brief: x轴正方向为轴 0度  逆时针增大  从 0 - 360
             *                         ^
             *                          |
             *                          |
             *      90-180      |         0 - 90
             *                          |
             *  -------------------------------------> x
             *                          |
             *     180-270     |        270-360
             *                          |
             *                          
             */            
            float xy2theta( const float & _x, const float & _y )
            { 
                // atan 范围为[-pi/2,+pi/2]  
                if ( _x >= 0 & _y >= 0) 
                    return (180 / M_PI) * atan(_y / _x);  

                if ( _x < 0 & _y >= 0) 
                    return 180 - ( (180 / M_PI) * atan(_y / (-_x)) ); 

                if ( _x < 0 & _y < 0) 
                    return 180 + ( (180 / M_PI) * atan(_y / _x) ); 

                if ( _x >= 0 & _y < 0)
                    return 360 - ( (180 / M_PI) * atan((-_y) / _x) );
            } // xy2theta

    }; // scanContext
} // namespace 
