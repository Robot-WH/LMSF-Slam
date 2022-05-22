cmake_minimum_required(VERSION 3.0.2)
project(app)

# set(CMAKE_BUILD_TYPE "Release" )
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories(
  include
  ${SLAM_INCLUDE_DIR}
  )

link_libraries(${SLAM_LIBRARIES})

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(ins_node src/localization/ins_node.cpp)
target_link_libraries(ins_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

# add_executable(filter_lio_node src/LIO/FilterLIO/simple_lio_node.cpp)
# target_link_libraries(filter_lio_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

# add_executable(lidar_preprocessing_node src/DataPreProcessing/lidar_preprocessing.cpp)
# target_link_libraries(lidar_preprocessing_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

# add_executable(lidar_lidar_odometry_node src/MultiLidarOdometry/lidar_lidar_odometry.cpp)
# target_link_libraries(lidar_lidar_odometry_node 
#                                             ${catkin_LIBRARIES} 
#                                             ${ALL_TARGET_LIBRARIES}
#                                             # ${G2O_LIBRARIES}
#                                             ${SLAM_LIBRARIES}
#                                             )

add_executable(MultiSensorFilterSLAM_node src/MultiSensorFilterSLAM_node.cpp)
target_link_libraries(MultiSensorFilterSLAM_node 
                                            ${catkin_LIBRARIES} 
                                            ${ALL_TARGET_LIBRARIES}
                                            ${SLAM_LIBRARIES}
)

# add_executable(MultiSensorFilterSLAM_node_temp src/MultiSensorFilterSLAM_node_temp.cpp)
# target_link_libraries(MultiSensorFilterSLAM_node_temp 
#                                             ${catkin_LIBRARIES} 
#                                             ${ALL_TARGET_LIBRARIES}
#                                             ${SLAM_LIBRARIES}
# )
                                            