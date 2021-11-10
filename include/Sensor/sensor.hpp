#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include "Geocentric/LocalCartesian.hpp"
#include "utility.hpp"


namespace Sensor{

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    enum sensor_id
    {
        lidar = 0,
        imu,
        gnss,
        wheel,      // 轮速计  
    };  
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief IMU数据结构 
     */
    struct ImuData 
    {
        double timestamp = 0;      // In second.
        Eigen::Vector3d acc = {0,0,0};   // Acceleration in m/s^2
        Eigen::Vector3d gyro = {0,0,0};  // Angular velocity in radian/s.
        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity(); // 旋转  
    };

    using ImuDataPtr = std::shared_ptr<ImuData>; 
    using ImuDataConstPtr = std::shared_ptr<const ImuData>; 

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief GNSS数据结构 
     */
    struct GnssData
    {
       double timestamp;      // In second.
       // WGS84系 
       Eigen::Vector3d lla; 
       Eigen::Matrix3d cov;  // Covariance in m^2. 
    };

    using GnssDataPtr = std::shared_ptr<GnssData>;
    using GnssDataConstPtr = std::shared_ptr<const GnssData>;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 轮速 数据结构 
     */
    struct WheelsData
    {
        
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief  激光点云数据结构 
     */
    struct LidarData
    {
       double timestamp;      // In second.
       // 点云
       pcl::PointCloud<PointType> point_clouds; 
    };

    using LidarDataPtr = std::shared_ptr<LidarData>;
    using LidarDataConstPtr = std::shared_ptr<const LidarData>;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    static ImuDataConstPtr ImuDataLinearInterpolation(ImuDataConstPtr const& last_imu, ImuDataConstPtr const& curr_imu, 
                                           double const& timestamp)
    {
       double t1 = timestamp - last_imu->timestamp;
       double t2 = curr_imu->timestamp - timestamp;
       // 计算插值系数     
       double front_scale = t2 / (t2+t1);
       double back_scale = t1 / (t2+t1);
       ImuDataPtr interpolation_imu{new ImuData{}}; 
       interpolation_imu->acc = front_scale * last_imu->acc + back_scale * curr_imu->acc; 
       interpolation_imu->gyro = front_scale * last_imu->gyro + back_scale * curr_imu->gyro; 
       interpolation_imu->rot = Eigen::Quaterniond::Identity(); // 旋转  
       // 时间
       interpolation_imu->timestamp = timestamp;

       return interpolation_imu;
    }
    
} // namespace Sensor 


#endif