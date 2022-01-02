
#ifndef GNSSdata_HPP_
#define GNSSdata_HPP_

#include <deque>
#include "Geocentric/LocalCartesian.hpp"
#include <sensor_msgs/Imu.h>
 
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

    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};

#endif