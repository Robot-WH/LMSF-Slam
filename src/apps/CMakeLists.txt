include_directories(
  include
  ${SLAM_INCLUDE_DIR}
  )

add_executable(OldPointCloudProcess_node
    src/LidarPreprocessing_node.cpp
)
# add_dependencies(OldPointCloudProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OldPointCloudProcess_node
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)

add_executable(PointCloudProcess_node
    src/DataPreProcessing/LidarPreProcessing_node.cpp
)
#add_dependencies(PointCloudProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PointCloudProcess_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(FeatureExtraction_node
    src/DataPreProcessing/FeatureExtraction_node.cpp
)
#add_dependencies(FeatureExtraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FeatureExtraction_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(imuPreintegration_node
    src/LIO/FactorGraphLIO/imuPreintegration_node.cpp
)
#add_dependencies(imuPreintegration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imuPreintegration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # ${Boost_LIBRARIES}
  # /usr/lib/x86_64-linux-gnu/libboost_timer.so
  gtsam
)

add_executable(mapOptmization_node
    src/LIO/FactorGraphLIO/mapOptmization_node.cpp
)
# add_dependencies(mapOptmization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapOptmization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS} 
  gtsam
)

add_executable(Segmentation_node
    src/Segmentation_node.cpp
)
# add_dependencies(Segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Segmentation_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable( basicLidarOdometry_node
    src/basic_FrontEnd/lidarOdometry_node.cpp
)
# add_dependencies(basic_lidar_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(basicLidarOdometry_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable( MapOptimator_node
    src/basic_FrontEnd/MapOptimator_node.cpp
)
# add_dependencies(MapOptimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MapOptimator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(Pointcloud2laserscan_node src/Pointcloud2laserscan_node.cpp)
# add_dependencies(Pointcloud2laserscan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Pointcloud2laserscan_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ascanRegistration_node src/feature_based_FrontEnd/scanRegistration.cpp)
#add_dependencies(ascanRegistration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ascanRegistration_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry_node src/feature_based_FrontEnd/laserOdometry.cpp)
#add_dependencies(alaserOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alaserOdometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping_node src/feature_based_FrontEnd/laserMapping.cpp)
#add_dependencies(alaserMapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alaserMapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ins_node src/localization/ins_node.cpp)
target_link_libraries(ins_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})