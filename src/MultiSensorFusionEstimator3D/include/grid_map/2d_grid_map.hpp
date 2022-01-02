
/**
 * @brief 栅格地图库   Copyright (C) 2020 Wenhao Li <3062451557@qq.com>
 * @author wenhao li
 * @date 2020.8.22
 **/

#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>

#include <nav_msgs/OccupancyGrid.h>

using namespace std; 

// 2d 占据栅格地图类    

class GridMap{
public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 构造函数   进行初始化    设置地图size    分辨率   
    GridMap ( const int& size_width, const int& size_height, const int& init_x, const int& init_y, const double& cell_size ) :
    size_width_ ( size_width ), size_height_ ( size_height ), init_x_ ( init_x ), init_y_ ( init_y ), cell_size_ ( cell_size )
    {
        // 初始值设定 
        map_bel_data_.resize ( size_height_, vector<double>(size_width_, 0.5) );                   // 保存所有栅格的概率    全部设为0.5的初始概率       
        map_logbel_data_.resize( size_height_, vector<double>(size_width_, 0) );  
        /* 为opencv图片显示相关  ???????????????????????????????????? */
        m_one_.resize( size_height_, vector<double>(size_width_, 1) );                             // 设置为 1  
        m_show_.resize( size_height_, vector<double>(size_width_, 0.5) );
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 设置x,y处栅格的概率   将概率设置到 bel_data_  
    bool setGridBel ( const double& x, const double& y, const double& bel )
    {
        Eigen::Vector2i idx;
        // 根据栅格坐标即 x,y 换算出栅格的序号   
        if(!getIdx(x, y, idx))
            return false;
        // map_bel_data_是一个 二维矩阵  保存所有栅格地图占用的概率   
        map_bel_data_[idx(1)][idx(0)] = bel;          // (row/y, col/x)
        return true;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 设置x,y处栅格的概率的Log  将概率设置到 bel_data_  
    bool setGridLogBel ( const double& x, const double& y, const double& Log_bel )
    {
        Eigen::Vector2i idx;
        // 根据栅格坐标即 x,y 换算出栅格的序号   
        if(!getIdx(x, y, idx))
            return false;
        // map_bel_data_是一个 二维矩阵  保存所有栅格地图占用的概率 
        map_logbel_data_[idx(1)][idx(0)] = Log_bel;     // (row/y, col/x)       
        return true;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 设置x,y处栅格的概率的Log  将概率设置到 bel_data_  
    bool setGridBelAndLogBel ( const double& x, const double& y, const double& bel, const double& Log_bel)
    {
        Eigen::Vector2i idx;
        // 根据栅格坐标即 x,y 换算出栅格的序号   
        if(!getIdx(x, y, idx))
            return false;
        // map_bel_data_是一个 二维矩阵  保存所有栅格地图占用的概率       
        map_bel_data_[idx(1)][idx(0)] = bel;                    
        map_logbel_data_[idx(1)][idx(0)] = Log_bel;
        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 获取x,y位置处栅格的概率  
    bool getGridBel ( const double& x, const double& y, double& bel)
    {
        Eigen::Vector2i idx;
        if(!getIdx(x, y, idx))
            return false;
        bel = map_bel_data_[idx(1)][idx(0)];
        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 求取x,y位置处的对数置信度   
    bool getGridLogBel ( const double& x, const double& y, double& log_bel )
    {
        Eigen::Vector2i idx;
        if(!getIdx(x, y, idx))
            return false;
        log_bel = map_logbel_data_[idx(1)][idx(0)];     // 行 列        
        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 通过对数概率求出栅格占用概率   然后设置   
    bool setGridBelByLogBel ( const double& x, const double& y, const double& log_bel )
    {   // 求栅格的占用置信度  
        double bel = 1.0 - 1.0 / (1 + exp(log_bel));
        // 设置x,y处栅格的概率   将概率设置到 bel_data_  
        if( !setGridBelAndLogBel(x, y, bel, log_bel) )
            return false;
        return true;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double getCellSize()
    {
       return cell_size_;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 转换到Ros栅格地图的消息
    void toRosOccGridMap(const std::string& frame_id, nav_msgs::OccupancyGrid& occ_grid) 
    {
        occ_grid.header.frame_id = frame_id;
        occ_grid.header.stamp = ros::Time::now();
        // 设置栅格的size    
        occ_grid.info.width = size_width_;
        occ_grid.info.height = size_height_;
        occ_grid.info.resolution = cell_size_;
        // 设置原点坐标   原点坐标相对于Map坐标系的偏移  
        occ_grid.info.origin.position.x = -init_x_*cell_size_;
        occ_grid.info.origin.position.y = -init_y_*cell_size_;
        // 遍历所有的栅格   
        const int N = size_width_ * size_height_;
        // 从最底层开始     由于 OccupancyGrid 是先从底层开始的 (行优先)
        for (int row = size_height_ - 1; row>=0; row--)
        {
            for(int col = 0; col < size_width_; col++)
            {
                double& value = map_bel_data_[row][col];     
                // 栅格地图在ROS消息下的表示为 occ_grid 为一个数组  
                if(value == 0.5)
                    occ_grid.data.push_back( -1);                // 表明未定义      
                else 
                    occ_grid.data.push_back( value * 100);       // 概率值 * 100      OccupancyGrid 每一栅格 概率范围为 0-100 
            }
        }

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 将栅格地图转为Mat   
    cv::Mat toCvMat()
    {
        // 构造opencv mat 
        cv::Mat map(size_height_, size_width_, CV_64FC1);

        for (int row = 0; row < size_height_; row++)
        {
            for (int col = 0; col < size_width_; col++)
            {   // 这样占据概率越高  map的值就越低  
                map.at<double>(row, col) = m_one_[row][col] - map_bel_data_[row][col];
            }
        }

        return map;
    }   
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 保存地图，图片加配置文件的形式
    void saveMap ( const std::string& img_dir, const std::string& cfg_dir )
    {
        // 保存图片 
        cv::Mat img = toCvMat();
        img = img * 255;              // 转换为颜色   也就是 占据的位置是0  黑色  
        cv::imwrite(img_dir, img);
        
        // 保存配置 
        std::ofstream  file;
        file.open(cfg_dir); 
        file << "map:"<< std::endl
        << "  size_width: " << size_width_ << std::endl
        << "  size_height: " << size_height_ << std::endl
        << "  init_x: " << init_x_ << std::endl
        << "  init_y: " << init_y_ << std::endl
        << "  cell_size: " << cell_size_ << std::endl;
    }     
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void loadMap(const std::string& img_dir, const std::string& cfg_dir); // TODO 加载地图
    
protected:

    /**
     * @brief 获得在 x,y处的栅格idx 
     * @param[in] x map坐标下的x坐标
     * @param[in] y map坐标下的y坐标
     * @param[out] idx 放置在栅格地图的二维数组中的位置      (x, y)
     **/
    bool getIdx(const double& x, const double& y, Eigen::Vector2i& idx)
    {   
        // 从这里也能看出来   当x,y的值为0时   也就是点云的map坐标原点的位置, 栅格图会建在中间的位置 
        // 栅格图的原点建立在 左下角  
        int xidx = cvFloor( x / cell_size_ ) + init_x_;                     // 求栅格地图的x坐标     
        int yidx  = size_height_ - cvFloor( y /cell_size_ ) - init_y_;      // 求栅格地图的y坐标 
         
        // TODO 动态扩张地图
        if((xidx < 0) || (yidx < 0) || (xidx >= size_width_) || (yidx >= size_height_))
            return false;
        idx << xidx , yidx;
        return true;
    }
    
private:

    int size_width_, size_height_;                          // 地图初始尺寸     
    // 地图中心相对与地图原点的偏移  点云地图的原点位于栅格地图的中心
    int init_x_, init_y_;                          
    double cell_size_;                            // 分辨率
    // 用vector 容器储存地图数据  
    vector<vector<double>> map_bel_data_;                // 保存所有栅格的概率  
    vector<vector<double>> map_logbel_data_;             // 保存栅格对数概率 
    vector<vector<double>> m_one_, m_show_;               
    
};// class GridMap


#endif