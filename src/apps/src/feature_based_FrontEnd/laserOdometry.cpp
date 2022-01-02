// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/*****************************************************************************
    将当前帧点云TransformToStart和将上一帧点云TransformToEnd的作用：
         去除畸变，并将两帧点云数据统一到同一个坐标系下计算
*****************************************************************************/

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <mutex>
#include <queue>

#include "common.h"
#include "tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;


pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

// 从队列取出的特征点云    
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
// 上一帧匹配对象
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
// 匹配特征的数量  
int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame    相对世界坐标的POSE  
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

// 通过这个映射   ceres优化para_q，para_t时  也会改变 q_last_curr、t_last_curr   
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);              
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);
// 接受特征点云的队列
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;

// undistort lidar point
//当前点云中的点相对第一个点去除因匀速运动产生的畸变，效果相当于得到在点云扫描开始位置静止扫描得到的点云
void TransformToStart(PointType const *const pi, PointType *const po)
{
    // odom 是上一帧的，上一帧的最后一个点的odom，也就是当前帧的第一个点的
    // 插值系数计算，云中每个点的相对时间/点云周期10
    // interpolation ratio
    double s;
    if (DISTORTION)  // 计算插值系数 
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;    // 因为point.intensity = scanID + scanPeriod * relTime       
    else
        s = 1.0;
    //s = 1;
    //线性插值：根据匀速运动假设  由每个点的插值系数获取旋转和平移  
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);   // 插值系数×上一帧四元数旋转   四元数球面插值  学习球面插值！！！！！！！！！！！
    Eigen::Vector3d t_point_last = s * t_last_curr;                                           // 插值系数×上一帧的位移
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    // 真实的位置   将该点转换到起始点坐标系！   P0 = ROt*Pt + T0t
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame
//将上一帧点云中的点相对结束位置去除因匀速运动产生的畸变，效果相当于得到在点云扫描结束位置静止扫描得到的点云
void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);       // 首先转换到起始坐标系

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);        // 转到最后坐标系    

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}


/************************************下面5个话题的回调函数都是将数据放入队列中************************************************/
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

    printf("Mapping %d Hz \n", 10 / skipFrameNum);
    // 订阅曲率大的特征点
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    // 订阅曲率小的特征点
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
    // 订阅平面的特征点
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
    // 订阅次平面的特征点
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
    // 滤波后计算出了每个点的时间以及scanID后的点云
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler); 

    // 发布当前的  cornerPointsLessSharp
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    // 当前发布的  surfPointsLessFlat
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    // velodyne_cloud_2话题订阅的数据
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    // 发布里程计信息  
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    // 发布雷达的pose   轨迹  
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        // 一次数据都收到了之后再执行  
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            // 同步作用，确保同时收到同一个点云特征点   因为在scanRegistration中发送时  时间戳相同
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
            // 检查时间戳是否相同
            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");
                ROS_BREAK();
            }

            // 队列的数据结构, 取出当前符合的帧
            mBuf.lock();    // 加锁    放置数据被新加的改变
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            mBuf.unlock();     // 解锁  

            TicToc t_whole;
            // initializing , 第一帧不做处理
            if (!systemInited)
            {
                systemInited = true;
                std::cout << "Initialization finished \n";
            }
            else
            {   
                // 获取特征的数量
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                TicToc t_opt;
                // 优化次数   
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    corner_correspondence = 0;
                    plane_correspondence = 0;

                    // ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);      // 鲁棒核函数    
                    // 四元数的广义加法  
                    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
                    
                    ceres::Problem::Options problem_options;
                    
                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;             // 选中的特征点去畸变后
                    std::vector<int> pointSearchInd;     //搜索到的点序
                    std::vector<float> pointSearchSqDis; //搜索到的点平方距离

                    TicToc t_data;
                    // find correspondence for corner features  首先对角点寻找匹配构造残差  遍历全部sharp点  
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);       // 根据匀速运动假设去畸变
                        // 当前sharp点通过kdtree搜索上一帧lesssharp点中与之最近的点
                        // kdtree细节    ！！！！！！！！！！！！！！！！！！
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        // 寻找相邻线距离目标点距离最小的点
                        // 再次提醒：velodyne是2度一线，scanID相邻并不代表线号相邻，相邻线度数相差2度，也即线号scanID相差2
                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)      //找到的最近点距离的确很近的话
                        {
                            // 获得序号  
                            closestPointInd = pointSearchInd[0];
                            //提取最近点线号   取整数部分  
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);
                            // 次近点的距离最小值
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;      //初始门槛值sqrt(DISTANCE_SQ_THRESHEOLD)米，可大致过滤掉scanID相邻，但实际线不相邻的值
                            // search in the direction of increasing scan line
                            // 寻找距离目标点最近距离的平方和最小的点
                            // laserCloudCornerLast=cornerPointsLessSharp  而cornerPointsLessSharp中，scanID大的点次序一定大于scanID小的点 
                            // 所以   往closestPointInd增大的方向  即使scanID增加的方向 
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // 向scanID增大的方向查找
                                // if in the same scan line, continue
                                // 确保两个点不在同一条scan上（相邻线查找应该可以用scanID == closestPointScan +/- 1 来做）
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop 
                                // 只搜索相邻scan   
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                                // 计算距离的模长
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);
                                
                                if (pointSqDis < minPointSqDis2) //距离更近，要小于初始值
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line   向 scanID 减小的方向搜索  
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;
                                // 计算距离    
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {
                            //选择的特征点记为O，kd-tree最近距离点记为A，另一个最近距离点记为B
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            double s;
                            if (DISTORTION)
                                // 对点进行投影  获得相对时间  
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;
                            // ceres构建残差的方式
                            // 先求出代价函数    static 函数  
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            // 添加残差   代价函数  核函数   优化变量 
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
                        }
                    }

                    //对本次接收到的曲率最小的点,从上次接收到的点云曲率比较小的点中找三点组成平面，一个使用kd-tree查找，另外一个在同一线上查找满足要求的，第三个在不同线上查找满足要求的
                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        // 去畸变 ！！！！！！！！！！！！！！！！！
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        // 细节 ！！！！！！！！！！！！！！！！！！！！！！！！！！
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            // 获取最近的lessflat点序号 
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            // 向scanID增加的方向搜索   
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                // 说明找到位于同一个scan的点   就是要找到次近点
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                // 相邻scan上找一个最近点
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }
                            // 向scanID减少的方向搜索 
                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                // 找同一个scan的次近点  
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }
                            // 成功找到两个  可以构成平面
                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                // 3个点构成一个对应   
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                if (DISTORTION)   
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                // ceres 构建残差   
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
                            }
                        }
                    }

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());
                    // 约束过少  
                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }

                    TicToc t_solver;
                    // 配置求解器   
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;   // 方程求解算法  
                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;   // 不输出
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);      // 开始优化
                    printf("solver time %f ms \n", t_solver.toc());
                }
                printf("optimization twice time %f \n", t_opt.toc());
                // ceres优化完后   结果就是  t_last_curr    q_last_curr
                // 转换到全局坐标上   t_w_curr     q_w_curr
                t_w_curr = t_w_curr + q_w_curr * t_last_curr;    // 因为用了eigen::map，所以在优化器里优化了para_q,para_t时，对应的向量也改变了   twl + qwl*tlc
                q_w_curr = q_w_curr * q_last_curr;               // qwl * qlc = qwc  
            }

            TicToc t_pub;

            // publish odometry   发布里程计消息 
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/odom";
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);
            // 发布轨迹  
            geometry_msgs::PoseStamped laserPose;    
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;      // 和laserOdometry的pose相同  
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/odom";
            pubLaserPath.publish(laserPath);

            // transform corner features and plane features to the scan end point
            /*
            if (0)
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }   */

            // 注意 是将cornerPointsLessSharp和surfPointsLessFlat作为下一帧的匹配对象  
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;                                    // 没用吧!!!!!!!!!!!!!!!!!!!!!
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;
            // 获取匹配数量   
            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();
            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';
            // 设置为kdtree搜索对象   
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);


            // 按周期发布点云  skipFrameNum决定了lasermapping的执行频率  这里skipFrameNum=5  即每5帧发布一次
            // 但是这里发布的点云与数据处理节点发布的点云  没什么区别   也没有去除畸变   需要优化  !!!!!!!!!!!!!!!!   
            if (frameCount % skipFrameNum == 0)    // skipFrameNum初始化为5  
            {
                frameCount = 0;
                // laserCloudCornerLast 即 cornerPointsLessSharp
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);
                // laserCloudSurfLast 即 surfPointsLessFlat
                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
                // laserCloudFullRes 即是数据处理节点中 去NaN + 距离滤波 + 数据模型化后的点云 
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
        }
        rate.sleep();
    }
    return 0;
}