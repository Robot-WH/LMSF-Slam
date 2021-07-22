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

#include <cmath>
#include <vector>
#include <string>
#include "common.h"
#include "tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

#define badpoint 0

const double scanPeriod = 0.1;

const int systemDelay = 0; // 放弃前X帧
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;            // main函数中赋值   默认16   
float cloudCurvature[400000];     // 曲率   
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];     // 记录每个点的曲率的程度  大:2    小：1   平面：-1 
//点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)
// 比较序号为i和j的点的曲率大小
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
std::vector<ros::Publisher> pubEachScan;


double MINIMUM_RANGE = 0.1; 

template <typename PointT>

// 距离滤波   
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres) //移除球体范围内距离太近的点
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    // 遍历输入点云的全部点  对合理的点重新排布
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y 
            + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue; // 以thres为半径的球内的点都不要
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

// 每一帧激光的回调函数
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{   
    // 舍弃前几sweep的数据
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    TicToc t_whole;
    TicToc t_prepare;
    // 记录每个scan有曲率的点的开始的索引
    std::vector<int> scanStartInd(N_SCANS, 0);   
    // 记录每个scan有曲率的点结束的索引
    std::vector<int> scanEndInd(N_SCANS, 0);     
    // ROS 消息数据转换为 PCL PointXYZ数据  多线激光雷达每一扫描点云sweep中的点的排列次序，是按照激光雷达的扫描顺序排列的，即先从初始旋转角度开始，
    // 依次获取垂直方向上所有的点，然后按旋转顺序，获取下一个旋转角度下每个垂直方向的点，直到旋转一周完成。
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices); // 去除无效点
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE); // 去除距离传感器太近的点

    /***********************************************数据模型化********************************************/
    int cloudSize = laserCloudIn.points.size();
    //lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转,atan2逆时针为正角
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    //lidar scan结束点的旋转角 
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;
    // 处理  保证  M_PI < endOri - startOri < 3 * M_PI
    if (endOri - startOri > 3 * M_PI)   // startOri<0
    {
        endOri -= 2 * M_PI;  
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    //lidar扫描线是否旋转过半
    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    // 遍历当前扫描全部点  根据每个点的坐标求出每个点的旋转角和scanID  根据旋转角确定时间
    // 将每个点按照帧的scanID划分到laserCloudScans中    注意每一scanID的laserCloudScans点依然是按照旋转顺序排列的
    for (int i = 0; i < cloudSize; i++)
    {
        // 读取每个点的坐标
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        /*********** 计算 scanID 即垂直激光帧的序号 (根据lidar文档垂直角计算公式),根据仰角排列激光线号，velodyne每两个scan之间间隔2度）**************/
        // 计算点的仰角
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {   // + 0.5 是用于四舍五入   因为 int 只会保留整数部分  如 int(3.7) = 3  
            scanID = int((angle + 15) / 2 + 0.5);    // +0.5是为了四舍五入, /2是每两个scan之间的间隔为2度，+15是过滤垂直上为[-,15,15]范围内

            if (scanID > (N_SCANS - 1) || scanID < 0)
            {   // 说明该点所处于的位置有问题  舍弃
                count--;
                continue;
            }
        }                                // 下面两种为 32线与64线的   用的少
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        /************************求出该点的旋转角*************************/
        float ori = -atan2(point.y, point.x);
        //std::cout<<"ori: "<<ori<<std::endl;
        if (!halfPassed)    // false
        {
            // 确保-pi/2 < ori - startOri < 3*pi/2
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            { 
                ori -= 2 * M_PI;     // 结果为   ori - startOri > - M_PI / 2
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else    // true
        {
            ori += 2 * M_PI;
            //确保-3*pi/2 < ori - endOri < pi/2
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        // -0.5 < relTime < 1.5（点旋转的角度与整个周期旋转角度的比率, 即点云中点的相对时间）
        float relTime = (ori - startOri) / (endOri - startOri);
        //点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
        point.intensity = scanID + scanPeriod * relTime;         // scanPeriod每一帧的时间   
        laserCloudScans[scanID].push_back(point);                // 将该点放入  scanID 帧中
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    /**********************************特征提取***********************************************/
    // 将scan从1-16按顺序排列组成的点云   scanID小的序号在前面  
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());   
    for (int i = 0; i < N_SCANS; i++)
    {   // 记录每个scan的可用曲率的序号  
        scanStartInd[i] = laserCloud->size() + 5;      // 可以纳入计算曲率的有效开始点从第六个开始，前面有5个点   符合论文所述
        *laserCloud += laserCloudScans[i];             // 将scanID为i的scan放入laserCloud  
        scanEndInd[i] = laserCloud->size() - 6;        // 记录记录曲率的终止点  后面还有5个点  
    }

    printf("prepare time %f \n", t_prepare.toc());

    /*************开始计算点的曲率**********************/
    for (int i = 5; i < cloudSize - 5; i++)
    {
        //使用每个点的前后五个点计算曲率，因此前五个与最后五个点跳过
        //但是中间scan的前后5个点没有跳过啊    中间scan的前后5个点会把前后scan的点算进去  这些跨scan的点存在高度差  
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x 
                    + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x
                    - 10 * laserCloud->points[i].x ;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y
                    + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y
                    - 10 * laserCloud->points[i].y ;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z
                    + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z
                    - 10 * laserCloud->points[i].z ;
        //曲率计算
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        // 记录曲率点索引   
        cloudSortInd[i] = i;
        //初始时，点全未筛选过
        cloudNeighborPicked[i] = 0;
        //初始化为less flat点
        cloudLabel[i] = 0;
    }

    /*************************************************** 将一些不好的点去掉 ********************************/
    #if badpoint==1
    for(int i=0; i<N_SCANS; i++)
    {
       int scan_start = scanStartInd[i];   
       int scan_end = scanEndInd[i];
       for(int j = scan_start; j<=scan_end; j++)
       {
         float diffX = laserCloud->points[j + 1].x - laserCloud->points[j].x;
         float diffY = laserCloud->points[j + 1].y - laserCloud->points[j].y;
         float diffZ = laserCloud->points[j + 1].z - laserCloud->points[j].z;
         //计算有效曲率点与后一个点之间的距离平方和
         float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

         if (diff > 0.1) {           //前提:两个点之间距离要大于0.1

                //点的深度diffY2
            float depth1 = sqrt(laserCloud->points[j].x * laserCloud->points[j].x + 
                            laserCloud->points[j].y * laserCloud->points[j].y +
                            laserCloud->points[j].z * laserCloud->points[j].z);

            //后一个点的深度
            float depth2 = sqrt(laserCloud->points[j + 1].x * laserCloud->points[j + 1].x + 
                            laserCloud->points[j + 1].y * laserCloud->points[j + 1].y +
                            laserCloud->points[j + 1].z * laserCloud->points[j + 1].z);

            //  将短的一边 做圆弧 求与长边相交点与短边点的距离
            if (depth1 > depth2) {
                diffX = laserCloud->points[j + 1].x - laserCloud->points[j].x * depth2 / depth1;
                diffY = laserCloud->points[j + 1].y - laserCloud->points[j].y * depth2 / depth1;
                diffZ = laserCloud->points[j + 1].z - laserCloud->points[j].z * depth2 / depth1;

                //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上    d/l = theta 
                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {     //排除容易被斜面挡住的点
                    //该点及前面五个点（大致都在斜面上）全部置为筛选过
                    cloudNeighborPicked[j - 5] = 1;
                    cloudNeighborPicked[j - 4] = 1;
                    cloudNeighborPicked[j - 3] = 1;
                    cloudNeighborPicked[j - 2] = 1;
                    cloudNeighborPicked[j - 1] = 1;
                    cloudNeighborPicked[j] = 1;
                    continue;
                 //   std::cout<<"bad point! scan: "<<i<<" index: "<<j<<std::endl; 
                }
            } else {
                diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
                    cloudNeighborPicked[j + 1] = 1;
                    cloudNeighborPicked[j + 2] = 1;
                    cloudNeighborPicked[j + 3] = 1;
                    cloudNeighborPicked[j + 4] = 1;
                    cloudNeighborPicked[j + 5] = 1;
                    cloudNeighborPicked[j + 6] = 1;

                    j+=6;    // 直接跳过后面这6个点  
                 //   std::cout<<"bad point! scan: "<<i<<" index: "<<j<<std::endl; 
                    continue;
                }
            }
        }

        //与前一个点的距离平方和
        float diffX2 = laserCloud->points[j].x - laserCloud->points[j - 1].x;
        float diffY2 = laserCloud->points[j].y - laserCloud->points[j - 1].y;
        float diffZ2 = laserCloud->points[j].z - laserCloud->points[j - 1].z;
        
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        //点深度的平方和
        float dis = laserCloud->points[j].x * laserCloud->points[j].x
                + laserCloud->points[j].y * laserCloud->points[j].y
                + laserCloud->points[j].z * laserCloud->points[j].z;

        //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
        if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
        cloudNeighborPicked[j] = 1;
        }
       }
    }
    #endif
    /*************开始提取特征**********************/
    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;           // 包含曲率很大的特征点  ——每scan中 曲率最大的前2个
    pcl::PointCloud<PointType> cornerPointsLessSharp;       // 曲率稍微小的特征点 —— 每scan中曲率前20 且曲率要大于0.1的点  注意包括cornerPointsSharp的点        
    pcl::PointCloud<PointType> surfPointsFlat;              // 挑选曲率最小的4个特征点 且曲率小于0.1  
    pcl::PointCloud<PointType> surfPointsLessFlat;          // 包含所有非sharp的点 

    float t_q_sort = 0;
    // 遍历每个scan
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] + 1 < 6)                 // 点的个数大于6才行，不然不能6等分
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        // 对于每个scan进行6等分      每个等分单独提取特征                              
        for (int j = 0; j < 6; j++)
        {
            // 获取该等份的点的序号范围
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i] + 1) * j / 6;             // 6等分,当前份的起点  
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i] + 1) * (j + 1) / 6 - 1;   // 6等分，当前份的终点   为什么要减1呢？ 避免两份重叠  提取重复特征点  
            // if(j==5) ep++;

            TicToc t_tmp;
            // 对cloudSortInd进行排序   依据的是序号cloudSortInd[i]的点的曲率从小到大 
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);        //按照曲率从小到大排序   注意sort函数的排序范围为  [，)
            t_q_sort += t_tmp.toc();

            //挑选每个分段的曲率很大和比较大的点
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];           //曲率最大点的序号   

                //如果曲率大的点，曲率的确比较大，并且未被筛选过滤掉  曲率的阈值为 0.1    
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 2)      //挑选曲率最大的前2个点放入sharp点集合
                    {                        
                        cloudLabel[ind] = 2;        // 2 代表曲率很大
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);     // cornerPointsLessSharp容器也包括cornerPointsSharp的点 ！！！！！！
                    }
                    else if (largestPickedNum <= 20)   //挑选曲率最大的前20个点放入less sharp点集合
                    {                        
                        cloudLabel[ind] = 1;           //1代表曲率比较尖锐
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;                           // 数量足够了  直接跳出循环   
                    }

                    cloudNeighborPicked[ind] = 1;        //过滤标志置位

                    // 考虑该特征点序号前后5个点，如果有两两之间距离过于接近的点  则不考虑将其前一个作为下一个特征点的候选点  这是防止特征点聚集，使得特征点在每个方向上尽量分布均匀  
                    // 注意这里特征点也是普通的点云中的点 
                      
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.1)
                        {
                            break;
                        }
                        // 该点直接取消特征点的候选资格
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.1)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 墙面同理
            int smallestPickedNum = 0;
            // 这里循环注意是按曲率从小到大
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];
                // 平面的标准是 曲率要小于0.1   
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1;     
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)               // 只选最小的四个
                    { 
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    // 同样前后5个点  不能两两之间不能太密集  
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            //将剩余的点（包括之前被排除的点以及平面点 ）全部归入平面点中less flat类别中
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)   //全部曲率<0的点    注意：  每个点的cloudLabel都初始化为0了   所以除了部分角点之外  其他的点都满足cloudLabel<=0   
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        //由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;      // 将每一个scan都叠加起来  
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());

    /*************完成提取特征**********************/
    // 发布这些点云  
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);        // laserCloud 为滤波处理后的点     去NaN + 距离滤波 + 数据模型化  
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/lidar_opt_odom";
    pubLaserCloud.publish(laserCloudOutMsg);
    // 发布曲率最大的一些点  
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/lidar_opt_odom";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);
    
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/lidar_opt_odom";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/lidar_opt_odom";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);        // 除了角点之外全部点  经过了滤波处理
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/lidar_opt_odom";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;
    
    // 读取扫描的线数
    nh.param<int>("scan_line", N_SCANS, 16);

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }
    //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 100, laserCloudHandler);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 100, laserCloudHandler);
    //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);
    // 滤波处理后的全部点云
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
    // 大曲率特征点
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    // 小曲率特征点 包含 大曲率
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
    // 平面点
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
    // 除了角点之外全部点  经过了滤波处理 
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    ros::spin();

    return 0;
}
