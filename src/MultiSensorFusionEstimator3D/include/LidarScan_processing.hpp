
#ifndef LIDARSCAN_PROCESSING_HPP
#define LIDARSCAN_PROCESSING_HPP

#include "utility.hpp"


class LidarScanProcessing
{

private:
    float scanPeriod;

public: 

    LidarScanProcessing():scanPeriod(0.1)
    {};
    ~LidarScanProcessing()
    {};  

    void FindStartEndAngle(pcl::PointCloud<PointType>::Ptr const& laserCloudIn, float &startOri, float &endOri)
    {
        int cloudSize = laserCloudIn->points.size();
        //lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转,atan2逆时针为正角
        startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        //lidar scan结束点的旋转角 
        endOri = -atan2(laserCloudIn->points[cloudSize - 1].y,
                            laserCloudIn->points[cloudSize - 1].x) +
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
        return;
    }

    // 计算雷达线束模型 
    void CalculateLidarModel(pcl::PointCloud<PointType>::Ptr const& laserCloudIn, int const& N_SCANS, 
                             double const& startOri, double const& endOri)
    {
        //lidar扫描线是否旋转过半
        bool halfPassed = false;
        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
        // 遍历当前扫描全部点  根据每个点的坐标求出每个点的旋转角和scanID  根据旋转角确定时间
        // 将每个点按照帧的scanID划分到laserCloudScans中    注意每一scanID的laserCloudScans点依然是按照旋转顺序排列的
        for (int i = 0; i < laserCloudIn->points.size(); i++)
        {
            // 读取每个点的坐标
            point.x = laserCloudIn->points[i].x;
            point.y = laserCloudIn->points[i].y;
            point.z = laserCloudIn->points[i].z;

            /*********** 计算 scanID 即垂直激光帧的序号 (根据lidar文档垂直角计算公式),根据仰角排列激光线号，velodyne每两个scan之间间隔2度）**************/
            // 计算点的仰角
            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (N_SCANS == 16)
            {   // + 0.5 是用于四舍五入   因为 int 只会保留整数部分  如 int(3.7) = 3  
                scanID = int((angle + 15) / 2 + 0.5);    // +0.5是为了四舍五入, /2是每两个scan之间的间隔为2度，+15是过滤垂直上为[-,15,15]范围内

                if (scanID > (N_SCANS - 1) || scanID < 0)
                {   // 说明该点所处于的位置有问题  舍弃
                    continue;
                }
            }                                // 下面两种为 32线与64线的   用的少
            else if (N_SCANS == 32)
            {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
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
            float relTime = (ori - startOri) / (endOri - startOri);      // 改成   relTime = (ori - startOri) / 2*Pi;  
            //点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
            point.intensity = scanID + scanPeriod * relTime;         // scanPeriod每一帧的时间   
            laserCloudScans[scanID].push_back(point);                // 将该点放入  scanID 帧中
       }
    }


};


#endif