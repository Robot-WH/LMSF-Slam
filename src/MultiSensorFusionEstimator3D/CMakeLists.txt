cmake_minimum_required(VERSION 3.0.2)
project(MultiSensorFusionEstimator3D)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g")
# set(CMAKE_BUILD_TYPE Release)   # Debug
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
# find_package(G2O REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(glog  REQUIRED)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

ADD_GLOBAL_INCLUDE_DIR(${CMAKE_CURRENT_SOURCE_DIR}/include/)

include_directories(
    include 
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

set(ALL_TARGET_LIBRARIES "")
include(cmake/g2o.cmake)

add_library(base_lib SHARED
    src/parameters.cpp
)

target_link_libraries(base_lib 
    ${EIGEN3_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${CERES_LIBRARIES}
    ${ALL_TARGET_LIBRARIES}
    ${GLOG_LIBRARIES}
    gtsam
)

add_library(backend_lib SHARED
    src/BackEnd/GraphOptimization/graph_optimization_g2o.cpp
    src/BackEnd/GraphOptimization/graph_optimization_gtsam.cpp
)

target_link_libraries(backend_lib 
    base_lib
)

ADD_GLOBAL_LIBRARY(base_lib)
ADD_GLOBAL_LIBRARY(backend_lib)

# add_executable(test_feature_registration
#     src/test/registration/feature_registration_test.cpp
# )
# # add_dependencies(OldPointCloudProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_feature_registration
#     ${PCL_LIBRARIES}
#     base_lib
# )

# add_executable(test_handeye_calib
#     src/test/handeye_calib_test.cpp
# )
# # add_dependencies(OldPointCloudProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_handeye_calib
#     ${PCL_LIBRARIES}
#     base_lib
# )





