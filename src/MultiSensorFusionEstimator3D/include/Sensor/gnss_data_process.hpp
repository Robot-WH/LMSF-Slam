/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-18 18:21:53
 * @Description: 
 * @Others: 
 */

#ifndef _GNSS_DATA_PROCESS_HPP_
#define _GNSS_DATA_PROCESS_HPP_

#include <eigen3/Eigen/Dense>
#include "Sensor/sensor.hpp"
#include "Geocentric/LocalCartesian.hpp"

namespace Slam3D { 

  /**
   * @brief: GNSS的数据处理 
   * @details:  使用 Meyers' Singleton
   */  
  class GnssDataProcess 
  {
    public:
      ~GnssDataProcess(){}
      GnssDataProcess(GnssDataProcess const& object) = delete;   // 拒绝拷贝构造
      GnssDataProcess& operator=(GnssDataProcess const& object) = delete;  // 拒绝赋值
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      static GnssDataProcess& GetInstance() {
        // 使用 C++ 11 的magic static 特性， 如果当变量在初始化的时候，并发同时进入声明语句，
        // 并发线程将会阻塞等待初始化结束
        static GnssDataProcess gnss_data_process;
        return gnss_data_process;  
      }

      ////////// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /**
       * @brief: 初始化局部ENU坐标系原点 
       * @param llt 经纬高
       * @return void
       */
      void InitOriginPosition(Eigen::Vector3d const& llt) {
          geo_converter_.Reset(llt[0], llt[1], llt[2]);
          enu_xyz_ = {0,0,0};
          origin_position_inited_ = true;
      }
      
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /**
       * @brief: 初始化后  将LLT转局部ENU系坐标 
       * @details: 
       * @param[in] llt 经维高
       * @param[out] enu 输出的ENU坐标   
       * @return 是否成功
       */      
      bool UpdateXYZ(Eigen::Vector3d const& llt, Eigen::Vector3d &enu) {
          if (!origin_position_inited_) {
              std::cout<< "GeoConverter has not set origin position"<<std::endl;
              return false; 
          }
          geo_converter_.Forward(llt[0], llt[1], llt[2], enu[0], enu[1], enu[2]);
          enu_xyz_[0] = enu[0];
          enu_xyz_[1] = enu[1];
          enu_xyz_[2] = enu[2];
          return true; 
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /**
       * @brief: 初始化后  将LLT转局部ENU系坐标 
       * @details: 重载，结果输出在  enu_xyz_ 
       * @param[in] llt 经维高
       * @return 是否成功
       */      
      bool UpdateXYZ(Eigen::Vector3d const& llt) {
          if (!origin_position_inited_) {
              std::cout<< "GeoConverter has not set origin position"<<std::endl;
              return false; 
          }
          geo_converter_.Forward(llt[0], llt[1], llt[2], enu_xyz_[0], enu_xyz_[1], enu_xyz_[2]);
          return true;
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /**
       * @brief: 获取ENU坐标  
       * @return value 
       */
      Eigen::Vector3d const& GetEnuPosition() const {
          return enu_xyz_;  
      }

    private:
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // 私有的构造
      GnssDataProcess(){}
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      GeographicLib::LocalCartesian geo_converter_;
      bool origin_position_inited_;
      // ENU系坐标  
      Eigen::Vector3d enu_xyz_; 
  };  // class GnssDataProcess 
}  // namespace Sensor 
#endif