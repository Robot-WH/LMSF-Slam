#include "loop_detect/scanContext/Scancontext.h"

// 通过全局描述子 - SC描述子 , 描述两帧之间的相似度
// 能否保证相似度得分随着平移的增加而缓慢下降 ??  这个将决定是否可以远距离回环 !  以及重定位 !  

void coreImportTest (void)
{
    cout << "scancontext lib is successfully imported." << endl;
} // coreImportTest


float rad2deg(float radians)
{
    return radians * 180.0 / M_PI;
}

float deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
}

// x轴正方向为轴 0度   逆时针增大
float xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
} // xy2theta

/**
 * @brief 对 _mat 的列   进行 _num_shift的平移
 * @param[in] _num_shift 平移量
 **/
MatrixXd circshift( MatrixXd &_mat, int _num_shift )
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

} // circshift


// @brief eigen 类型数据转换为 vector 数据 
std::vector<float> eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} // eig2stdvec


/**
 * @brief 计算两个sc描述子的距离   参考论文 (5) 式
 * @return 接近程度  越小越接近
 **/ 
double SCManager::distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    int num_eff_cols = 0;                     // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    // 遍历每一列  单独对每一列求距离              
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);
        // | == ||    有一个为0  则continue!  
        if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
            continue; // don't count this sector pair. 
      
        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols++;
    }
    
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;      // 越小越接近  

} // distDirectSC

/**
 * @brief 用sectorkey 快速求出向量的平移量  
 * 
 **/
int SCManager::fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    // 对这个一维向量进行平移 找出让这两个向量最接近的平移量   
    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
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
    
}   // fastAlignUsingVkey

/**
 * @brief 对两个sc描述子求取相似性
 * @return (最小相似得分, 旋转距离)
 **/
std::pair<double, int> SCManager::distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    // 1. fast align using variant key (not in original IROS18)
    // 先求出 Sectorkey 
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );    // 快速求出平移量    粗略值   
    // 在 argmin_vkey_shift 附近进行 进行平移 
    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range    SEARCH_RATIO = 0.1  
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    // 将平移的范围求出  
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );      // 
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    // 遍历 平移量 shift_idx_search_space 
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);        // 平移
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );   // 按照论文求距离 
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext

/**
 * @brief 构造scan context描述符
 * @param[in]  _scan_down 输入点云
 * @return desc 
 **/
MatrixXd SCManager::makeScancontext( pcl::PointCloud<SCPointType> & _scan_down )
{
    TicToc t_making_desc;
    
    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;     // 矩阵每一个的初值  
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);      // 构造一个二维矩阵 即sc描述子  行为ring的数量  列为sector数量 

    SCPointType pt;    // pcl::PointXYZI 
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sector_idx;
    // 遍历每一个点  
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        // 获取每一个点的坐标  
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);     // R的值  决定ring的值
        azim_angle = xy2theta(pt.x, pt.y);                // 决定 sector 

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;
        // 求ring和sector的idx       ceil(x) 大于 x 的 最小整数 
        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1);   // 范围  [1, PC_NUM_RING]
        sector_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1);     // 范围  [1, PC_NUM_SECTOR]

        // sc描述子 保存对应位置的最大z轴高度值   赋值  给sc描述符 
        // 减1 是因为 行和列的起始位置是0 
        if ( desc(ring_idx-1, sector_idx-1) < pt.z ) // -1 means cpp starts from 0
            desc(ring_idx-1, sector_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)   如果有的desc的部分没有赋值的话   设置为 0   
    // 这里不能在初始化的时候直接赋值为0   因为 sc描述子在取值的时候  会去最大值作为当前值   如果点云高度为负数的话  初始化为0 就会得到错误的结果  
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext making");

    return desc;
} // SCManager::makeScancontext

/**
 * @brief 构造ring key
 * @param[in] _desc SC描述符
 * @details 对每个SC描述符的每一行求均值  
 * @return ringkey 描述符   列向量  
 **/
MatrixXd SCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
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
} // SCManager::makeRingkeyFromScancontext

/**
 * @brief 构造sector key 与原文不同 , 主要用于求取 旋转平移量 
 * @param[in] _desc SC描述符
 * @details sector key 即将SC描述子每一列的向量求均值   
 **/
MatrixXd SCManager::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
} // SCManager::makeSectorkeyFromScancontext

/**
 * @brief 每一个帧都用这个函数生成全局描述子  
 * @param[in] _scan_down 输入点云   SCPointType = pcl::PointXYZI
 * @details 提取sc描述子的全部流程  
 **/
void SCManager::makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down )
{
    Eigen::MatrixXd sc = makeScancontext(_scan_down);                  // v1 提取sc全局特征描述符
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );        // 求 ring key 特征  
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );    // 提取sector key 与论文不同    粗略确定平移范围  
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );

    polarcontexts_.push_back( sc );                       // 当前帧点云检测完毕后   SC描述子 存放于 polarcontexts_
    polarcontext_invkeys_.push_back( ringkey );           
    polarcontext_vkeys_.push_back( sectorkey );           
    polarcontext_invkeys_mat_.push_back( polarcontext_invkey_vec );

    // cout <<polarcontext_vkeys_.size() << endl;

} // SCManager::makeAndSaveScancontextAndKeys


// 闭环检测
std::pair<int, float> SCManager::detectLoopClosureID ( void )
{
    int loop_id { -1 };                               // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")
    // 首先将最新的帧的sc描述子提取出来  进行回环检测 
    auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)      提取最新一帧ring-key
    auto curr_desc = polarcontexts_.back();           // current observation (query)      提取最新的sc描述子  

    /* 
     * step 1: candidates from ringkey tree_  首先查找ring key  获得候选关键帧
     */
    // 如果数量不够  
    if( polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result {loop_id, 0.0};
        return result; // Early return 
    }

    // tree_ reconstruction (not mandatory to make everytime)   使用kdtree对ring key进行查找  
    // 把历史关键帧的ringkey构造kdtree   
    if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0)         // to save computation cost    频率控制  
    {
        TicToc t_tree_construction;
        // std::vector<std::vector<float> > 类型
        polarcontext_invkeys_to_search_.clear();
        // 构造用于搜索的ringkey 集合    assign()  将区间[first,last)的元素赋值到当前的vector容器中     这里减去 NUM_EXCLUDE_RECENT 也就是 不考虑最近的若干帧 
        polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT ) ;
        // KDTreeVectorOfVectorsAdaptor<>的 unique_ptr 
        polarcontext_tree_.reset(new InvKeyTree(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */) ); 
        // TODO: 构建kdtree的细节 ?????????????????????????????????????????
        //polarcontext_tree_ = std::make_unique<InvKeyTree>( new InvKeyTree(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */) );
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;
        
    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // knn search   NUM_CANDIDATES_FROM_TREE = 10  , 10个候选关键帧    
    std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE );   // 保存候选帧在 数据容器中的序号 
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );        // 保存候选关键帧的ringkey的距离  

    TicToc t_tree_search;
    // 找 NUM_CANDIDATES_FROM_TREE  个最近关键帧 
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    // 初始化    用 candidate_indexes 和  out_dists_sqr 数组的数组名地址初始化          设置搜索结果存放的数组  
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );    
    // 查找与当前待搜索的ringkey 距离最近的10帧
    polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
    t_tree_search.toc("Tree search");

    /* 
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)    挑选最佳的候选关键帧   旋转检查
     */
    TicToc t_calc_dist;   
    // 遍历所有候选关键帧         
    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    {   // 获取候选关键帧 的sc描述子  
        MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
        // 计算描述子的距离  
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
        double candidate_dist = sc_dist_result.first;       // 距离   
        int candidate_align = sc_dist_result.second;        // 平移量 
        // 如果距离最小  
        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;
            // 获取index  
            nn_idx = candidate_indexes[candidate_iter_idx];
        }
    }
    t_calc_dist.toc("Distance calc");

    /* 
     * loop threshold check    即判断是否小于阈值  
     */
    if( min_dist < SC_DIST_THRES )
    {
        loop_id = nn_idx; 
    
        // std::cout.precision(3); 
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }
    else
    {
        std::cout.precision(3); 
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);      // 返回yaw角的差  
    std::pair<int, float> result {loop_id, yaw_diff_rad};      

    return result;

} // SCManager::detectLoopClosureID

