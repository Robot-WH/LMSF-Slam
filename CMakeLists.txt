cmake_minimum_required(VERSION 2.8.3)
project(liv_slam)

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
find_package(G2O REQUIRED)
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
  INCLUDE_DIRS include
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
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR} 
  ${G2O_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

include(cmake/Macros.cmake)
include(cmake/geographic.cmake)

# file(GLOB_RECURSE ALL_SRCS "*.cpp")     # 所有的.cpp文件  放置到 ALL_SRCS 
# file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")     # 所有的node cpp 文件
# file(GLOB_RECURSE other_SRCS "src/apps/feature_based_FrontEnd/*.cpp")     # 所有的node cpp 文件
# file(GLOB_RECURSE THIRD_PARTY_SRCS "third_part/*.cpp") # 所有第三方库 cpp文件
# list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})                          # ALL_SRCS 中剔除node cpp文件
# list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})         # ALL_SRCS 中剔除库cpp文件 
# list(REMOVE_ITEM ALL_SRCS ${other_SRCS}) 


###########
## Build ##
###########

# message(WARNING ${Boost_INCLUDE_DIR})
# message(WARNING ${Boost_LIBRARY_DIRS})
# message(WARNING ${Boost_LIBRARIES})

# add_executable( FilterFusionOdometry
#    src/apps/LIO/FilterLIO/FilterFusionOdometry_node.cpp
#    src/lib/ros_bridge/FilterFusionLIO_bridge.cpp
# )
# add_dependencies(FilterFusionOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(FilterFusionOdometry
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   MultisensorFusionLib
# )

# add_executable( 2d_GridMapping
#   src/apps/2D_mapping_node.cpp
#   src/lib/2d_grid_mapping.cpp
# )
# target_link_libraries(2d_GridMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS})


# add_executable(BackEnd_node
#   src/apps/Backend/BackEnd_node.cpp
#   src/lib/keyframe.cpp
#   src/lib/information_matrix_calculator.cpp
#   src/lib/robust_kernel_io.cpp
#   src/lib/map_cloud_generator.cpp
#   ${ALL_SRCS}
# )

# add_dependencies(BackEnd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(BackEnd_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${G2O_TYPES_DATA}
#   ${G2O_CORE_LIBRARY}
#   ${G2O_STUFF_LIBRARY}
#   ${G2O_SOLVER_PCG}
#   ${G2O_SOLVER_CSPARSE}
#   ${G2O_SOLVER_CHOLMOD}
#   ${G2O_TYPES_SLAM3D}
#   ${G2O_TYPES_SLAM3D_ADDONS}
#   MultisensorFusionLib
#   glog
# )

add_subdirectory(src)



