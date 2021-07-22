/**
 * @brief 2D栅格的建图算法
 * @author wenhaoli, 3062451557@qq.com 
 * @date 2020/8/22
 **/


#include "grid_map/2d_grid_mapping.h"

GridMapping::GridMapping ( GridMap* map, Pose2d& T_r_l, double& P_occ, double& P_free, double& P_prior):
map_(map), T_r_l_(T_r_l), P_occ_(P_occ), P_free_(P_free), P_prior_(P_prior)
{
    InvModel_Ps[P_free_] = log( P_free_ / (1.0 - P_free_) );
    InvModel_Ps[P_occ_] = log( P_occ_ / (1.0 - P_occ_) );
    InvModel_Ps[P_prior_] = log( P_prior_ / (1.0 - P_prior_) );
}

// 根据激光和机器人的位姿来更新地图 
void GridMapping::updateMap ( const sensor_msgs::LaserScanConstPtr& scan,  Pose2d& robot_pose )
{
    /* 获取激光的信息 */
    const double& ang_min = scan->angle_min;       // 激光的起始角 
    const double& ang_max = scan->angle_max;       // 激光的终止角
    const double& ang_inc = scan->angle_increment;   // 激光点的角度增量
    const double& range_max = scan->range_max;      // 范围
    const double& range_min = scan->range_min;
    
    /* 设置遍历的步长，沿着一条激光线遍历 */
    const double& cell_size = map_->getCellSize();       // 每个cell的尺寸  
    const double inc_step = 1.0 * cell_size;             // 步进的长度   

    /* for every laser beam  遍历一个激光束的全部激光点  */   
    for(float i = 0; i < scan->ranges.size(); i++)       // 一个激光束储存的是所有点的range  
    {
        /* 获取当前beam的距离 */
        double R = scan->ranges.at(i); 
        if(R < range_min||R>range_max)
            continue;
        
        /* 沿着激光射线以inc_step步进，更新地图 */
        double angle = ang_inc * i + ang_min;                         
        Eigen::Vector2d last_grid(Eigen::Infinity, Eigen::Infinity);            //上一步更新的grid位置，防止重复更新
        double step_x = inc_step * cos(angle);
        double step_y = inc_step * sin(angle);  
        double x = 0;
        double y = 0;  

        // 沿着激光束前进   步进 cell_size  
        for(double r = 0; r < R + cell_size; r += inc_step)
        {
            // 激光点在激光雷达坐标系的Pos   
            Eigen::Vector2d p_l(x, y);   //在激光雷达坐标系下的坐标
            x += step_x;
            y += step_y;  
            
            /* 转换到世界坐标系下 */
            Pose2d laser_pose = robot_pose * T_r_l_;                 // Tw_r*Tr_l = Tw_l        robot_pose 应该是 里程计的Pose   T_r_l 因该为 里程计到 
            Eigen::Vector2d p_w = laser_pose * p_l;                  // 转到世界坐标系map中   Tw_l * Pl

            /* 更新这个grid */
            if(p_w == last_grid) //避免重复更新
                continue;
            // 更新栅格
            updateGrid(p_w, laserInvModel(r, R, cell_size, range_max));
            // 记录更新的位置   	    
            last_grid = p_w;
        }//for each step
    }// for each beam
}


/**
 * @brief 更新栅格的概率    
 * @details pmzx 是   P(mi | zt, xt)   在当前处于xt 位置, 观测为zt的条件下, mi被占用的概率  
 * @param grid_pos 世界坐标系下位姿
 * @param log_pmzx 反演观测概率 
 **/
void GridMapping::updateGrid ( const Eigen::Vector2d& grid_pos, const double& log_pmzx )
{
    /* TODO 这个过程写的太低效了 */
    double log_bel;

    if( !map_->getGridLogBel( grid_pos(0), grid_pos(1), log_bel ) )     //获取上一时刻 log的bel
        return;

    log_bel += log_pmzx;                         // 更新    加上 反演观测模型        不用加 先验模型?    因为先验模型 为 0   
    map_->setGridBelByLogBel( grid_pos(0), grid_pos(1), log_bel  );         // 将log_bel转换为概率后   设置回地图 bel_data_  数组 
}

// 求 P(mi | zt, xt)的对数概率   在当前处于xt 位置, 观测为zt的条件下, mi被占用的概率  
double GridMapping::laserInvModel ( const double& r, const double& R, const double& cell_size , const int& range_max)
{   
    // 说明没有击中  
    if(R == range_max)
       return InvModel_Ps[P_free_];  
    // 小于当前击中点的部分  为空闲的   0.4
    if(r < ( R - 0.5*cell_size) )
        return InvModel_Ps[P_free_];
    // 后面的不知道被占据没有  设置为先验    0.5   
    if(r > ( R + 0.5*cell_size) )
        return InvModel_Ps[P_prior_];
    // 占据点   0.6   
    return InvModel_Ps[P_occ_];
}

