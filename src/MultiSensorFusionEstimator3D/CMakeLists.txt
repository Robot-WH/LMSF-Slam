
cmake_minimum_required(VERSION 3.0.2)
project(MultiSensorFusionEstimator3D)
set(CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g")

find_package(Boost REQUIRED COMPONENTS thread)

ADD_GLOBAL_INCLUDE_DIR(${CMAKE_CURRENT_SOURCE_DIR}/include/)

include_directories(
    include 
    ${Boost_INCLUDE_DIRS}
)





