/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-01-19 11:28:57
 * @Description:  匹配算法的工厂类  
 * @Others: 
 */
#ifndef _LIDARTRACKER_FACTORY_HPP_
#define _LIDARTRACKER_FACTORY_HPP_

#include "Common/parameters.h"
#include "Common/read_param.hpp"
#include "Algorithm/PointClouds/registration/registration_adapter.hpp"
#include "factory/registration/registration_factory.hpp"


namespace Slam3D {

  // using TrackerDirectMethodPtr = std::unique_ptr<Module::LidarTrackerDirectMethodBase>;  

  /**
   * @brief: 构建直接法激光tracker  
   * @details 变化 - 1、匹配策略  2、使用的匹配算法  
   * @param config_file 配置文件的
   */  
  // TrackerDirectMethodPtr make_lidarTrackerDirectMethod(std::string const& config_file) 
  // {
    // ParametersRead& param_read = ParametersRead::GetInstance(config_file);   // 参数读取器 
    // std::string LIDAR_TRACKING_MODE;  
    // param_read.ReadParam("lidar_tracking_mode", LIDAR_TRACKING_MODE);
    // std::unique_ptr<Algorithm::RegistrationAdapterBase<PCLConstPtr>> registration_ptr;   // 直接法使用的匹配器
    // // 构建激光tracker 
    // if (LIDAR_TRACKING_MODE == "scan_map") // scan-map的匹配策略  
    // {
    //   std::string REGISTRATION_METHOD;
    //   param_read.ReadParam("tracker.scan_map.method", REGISTRATION_METHOD);
    //   // 使用NDT匹配 
    //   if (REGISTRATION_METHOD == "ndt_omp")
    //   {
    //     double ndt_resolution;
    //     double transformation_epsilon;
    //     float step_size;
    //     int maximum_iterations; 
    //     int num_threads;
    //     std::string nn_search_method;
    //     param_read.ReadParam("tracker.scan_map.registration.ndt.resolution", ndt_resolution);
    //     param_read.ReadParam("tracker.scan_map.registration.ndt.epsilon", transformation_epsilon);
    //     param_read.ReadParam("tracker.scan_map.registration.ndt.step_size", step_size);
    //     param_read.ReadParam("tracker.scan_map.registration.ndt.maximum_iterations",
    //                                                           maximum_iterations);
    //     param_read.ReadParam("tracker.scan_map.registration.ndt.num_threads", num_threads);
    //     param_read.ReadParam("tracker.scan_map.registration.ndt.search_method", nn_search_method);
    //     NDTOMPPtr ndt_omp = make_ndtOmp(ndt_resolution, transformation_epsilon, step_size,
    //                                                                                         maximum_iterations, num_threads, nn_search_method);  
    //     registration_ptr.reset( new Algorithm::RegistrationAdapterImpl<
    //                                                     PCLConstPtr, pcl::Registration<PointType, PointType>
    //                                                     >(std::move(ndt_omp)));                                         
    //     return TrackerDirectMethodPtr(new Module::LidarTrackerDirectMethodLocalMap(std::move(registration_ptr), 
    //                                                                       scan_map_windows_size_));  
    //   }
    //}
  //   if (LIDAR_TRACKING_MODE == "loam") // loam的匹配策略 
  //   {
  //   }
  //   return nullptr;  
  // }

}
#endif