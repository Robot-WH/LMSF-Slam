/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-10-27 00:13:12
 * @Description: 
 * @Others: 
 */
#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Geocentric/LocalCartesian.hpp"


namespace Slam3D {

    using PointType = pcl::PointXYZI;
    using PCLConstPtr = pcl::PointCloud<PointType>::ConstPtr;  
    using PCLPtr = pcl::PointCloud<PointType>::Ptr;  
    using PCLType = pcl::PointCloud<PointType>;  

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
    * Velodyne点云结构，变量名XYZIRT是每个变量的首字母
   */
   // struct VelodynePointXYZIRT
   // {
   //    PCL_ADD_POINT4D     // 位置
   //    PCL_ADD_INTENSITY;  // 激光点反射强度，也可以存点的索引
   //    uint16_t ring;      // 扫描线
   //    float time;         // 时间戳，记录相对于当前帧第一个激光点的时差，第一个点time=0
   //    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   // } EIGEN_ALIGN16;        // 内存16字节对齐，EIGEN SSE优化要求
   // // 注册为PCL点云格式
   // POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
   //    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
   //    (uint16_t, ring, ring) (float, time, time)
   // )




   //  template<class LidarDataType>
   //  using MultiLidarDataPtr = std::shared_ptr<MultiLidarData<LidarDataType>>;
   //  template<class LidarDataType>
   //  using MultiLidarDataConstPtr = std::shared_ptr<const MultiLidarData<LidarDataType>>;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    static ImuDataConstPtr ImuDataLinearInterpolation(ImuDataConstPtr const& last_imu, 
                                                                                                                ImuDataConstPtr const& curr_imu, 
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