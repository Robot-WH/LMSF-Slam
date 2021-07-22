
#ifndef _GNSS_DATA_HPP_
#define _GNSS_DATA_HPP_

#include "utility.hpp"
#include "ros_utils.hpp"
#include "Sensor/sensor.hpp"
#include "Geocentric/LocalCartesian.hpp"


namespace Sensor { 

  class GnssDataProcess {
    public:
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      static GnssDataProcess* GetInstance();

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // 进行初始化  
      void InitOriginPosition(Eigen::Vector3d const& llt);
      
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // 初始化后  将LLT转局部ENU系坐标 
      bool UpdateXYZ(Eigen::Vector3d const& llt, Eigen::Vector3d &enu);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // 初始化后  将LLT转局部ENU系坐标 
      bool UpdateXYZ(Eigen::Vector3d const& llt);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // 获取ENU坐标  
      Eigen::Vector3d const& GetEnuPosition() const;

    private:
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // 私有的构造
      GnssDataProcess(){}
      ~GnssDataProcess(){}
      // 拷贝构造与拷贝赋值
      GnssDataProcess(GnssDataProcess const& gnss_data_process);
      const GnssDataProcess &operator=(GnssDataProcess const& gnss_data_process);
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      static GnssDataProcess* gnss_data_process_ptr;
      static std::mutex m_gnss_data_process; 
      GeographicLib::LocalCartesian geo_converter;
      bool origin_position_inited;
      // ENU系坐标  
      Eigen::Vector3d enu_xyz; 
  };  


  class GNSSData {
    public:
      ros::Time time;
      // WGS84系 
      double longitude = 0.0;
      double latitude = 0.0;
      double altitude = 0.0;
      // UTM系 
      double local_E = 0.0;
      double local_N = 0.0;
      double local_U = 0.0;
      // MAP系 下Lidar的坐标 
      Eigen::Vector3d Lidar_Map_coords = {0., 0., 0.};
      // MAP系下Lidar的旋转  
      Eigen::Quaterniond Lidar_Map_orientation;
      // IMU的旋转   初始化前使用
      Eigen::Quaterniond IMU_orientation;
      sensor_msgs::ImuConstPtr imu;
      int status = 0;
      int service = 0;

    private:
      // 静态成员必须在类外初始化  
      static GeographicLib::LocalCartesian geo_converter;
      static bool origin_position_inited ;

    public: 
    
      // 进行初始化  
      void InitOriginPosition()
      {
        geo_converter.Reset(latitude, longitude, altitude);
        origin_position_inited = true;
      }

      void UpdateXYZ()
      {
          if (!origin_position_inited) {
            std::cout<< "GeoConverter has not set origin position"<<std::endl;
          }
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
      }


  };


}  // namespace Sensor 

#endif