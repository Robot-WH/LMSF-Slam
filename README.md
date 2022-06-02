# LMSF-SLAM
lifelong-多传感器融合SLAM

目前以完成：

1、多激光在线外参标定与融合建图。

2、地图、pose-graph、描述子数据库保存与载入。

3、基于激光外观描述子+几何位置搜索的回环检测。

4、基于外观描述子的重定位，开机自动重定位当前位置，定位丢失自动进行重定位找回。

5、定位模式下地图扩展。

6、建图时依据历史环境变化率选择是否插入节点(提升建图时节点的稀疏性，部分地图更新功能，缺陷是不具备主动稀疏化以及更新静态环境变化的能力)。

TODO：

1、激光-IMU-WHEEL-GNSS融合的里程计。

2、多轨迹融合。

3、3D点云，2D栅格地图有同步构建。

4、动态物体滤除。

5、基于静态环境变化的地图更新。

依赖：

1、安装OPENCV、PCL、Eigen、Ceres、G2o

2、安装的ROS依赖
sudo apt install ros-kinetic-geographic-msgs ros-kinetic-nmea-msgs


