cmake_minimum_required(VERSION 2.8.3)
project(liv_slam)
# CMAKE_MODULE_PATH是以分号分隔的列表，供include()或 find_package()使用
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g")
# set(CMAKE_BUILD_TYPE Release)   # Debug

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  pcl_conversions
  cv_bridge
  pcl_ros
  std_msgs
  pcl_msgs
  sensor_msgs
  geographic_msgs
  nmea_msgs
  nav_msgs
  message_generation
  ndt_omp
)

find_package(catkin REQUIRED )
find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(glog  REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(FILES
  SaveMap.srv
  SaveData.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  geographic_msgs
  nmea_msgs
)

# 如果没有这句话 那么生成的可执行文件就会到build文件下  正确的是应该到devel下
catkin_package(
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
  nmea_msgs
  message_runtime
  message_generation
)

include_directories(
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  # ${G2O_INCLUDE_DIR} 
  # ${G2O_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

include(cmake/Macros.cmake)
include(cmake/geographic.cmake)

add_subdirectory(src)



