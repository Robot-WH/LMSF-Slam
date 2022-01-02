
/**
 *  后端优化节点：  
 *  用于滤波器前端 
 *  约束数据：   1、里程计约束    2、回环约束     3、GNSS约束
 **/

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <thread>
#include <iostream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>     // pcl::fromROSMsg
#include <queue>

#include "utility.hpp"
#include "ros_utils.hpp" 
#include "keyframe_updater.hpp" 
#include "keyframe.hpp"
#include "loop_detector.hpp"
#include "information_matrix_calculator.hpp"
#include "map_cloud_generator.hpp"
#include "nmea_sentence_parser.hpp"
#include "Sensor/Gnss_data.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
//GPS
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Sentence.h>
// IMU
#include <sensor_msgs/Imu.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_so3_prior.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <ros_time_hash.hpp>


typedef pcl::PointXYZI PointT;  
using namespace std;
using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::ColMajor>;

//静态成员变量必须在类外初始化
bool Sensor::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian Sensor::GNSSData::geo_converter;

const int Optimize_duration = 3;  
const int Map_updata_duration = 10;  
const bool SaveOdometryPath = true;  
const bool SaveBackEndPath = false;  

class SlidingWindowBackEnd
{
  public:

    SlidingWindowBackEnd()
    {
      private_nh = ros::NodeHandle("~");
      commInit();
      paramsInit();  
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 读取前端发送的激光里程计数据  判断是否需要添加关键帧   
     * @details 检测是否需要添加关键帧, 如果需要将关键帧信息添加到 new_keyframe_queue 
     */
    void CloudCallback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg); 

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 读取组合导航仪的imu以及GNSS数据     
     * @details 初始化前, 将原始GNSS数据保存, 初始化后, 将GNSS数据转换为MAP坐标系值然后保存  并实时输出  
     */
    void GnssCallback(const sensor_msgs::ImuConstPtr& imu_msg, const sensor_msgs::NavSatFixConstPtr& navsat_msg);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 后端处理线程 
     * @details 1、每存在新的关键帧  
     */    
    void BackEndProcess();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 回环检测线程
     * @details 对 wait_loopDetect_keyframes 中的关键帧进行回环检测
     */
    void LoopDetectProcess();

  protected:

  private:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ROS 通信初始化 
    void commInit();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 参数初始化 
    void paramsInit();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 创建可视化队列
     * @param stamp
     * @return
     */
    visualization_msgs::MarkerArray createMarkerArray(const ros::Time& stamp);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief GNSS坐标与Lidar坐标对齐
     * @param first_gnss_data 第一个有效GPS数据 
     * @param align_keyframe 与该GPS数据对齐的关键帧数据  其中 orientation 是通过IMU插值出来的精确姿态 
     * @return 是否初始化成功
     * TODO: 如何判断初始化失败 返回false ????? 
     */ 
    void gnssInitialize(Sensor::GNSSData & first_gnss_data, Eigen::Quaterniond const& first_gnss_rotate, 
                                              KeyFrame::Ptr align_keyframe);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 将GNSS 经纬高信息 转换为 Map坐标系下Lidar坐标  
     * @param gnss_data 经纬高的值
     * @param orientation 对应位置IMU的姿态    由于 gnss_data 中的 imu 数据不是插值的数据  所以如果 有插值 这里要传入插值后的IMU   
     * @param[out] utm_coords 转换完成的值  
     */
    void transformWgs84ToMap(Sensor::GNSSData &gnss_data);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 数据处理   主要进行数据对齐
     * @details 批量对所有的关键帧以及GNSS数据进行匹配 
     */
    bool processDataInBatch();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 数据处理   主要进行数据对齐
     * @details 单独对每个关键帧进行匹配 
     */
    void processDataInSingle();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 将关键帧与GNSS的数据进行插值对齐
     * @details 批量匹配 
     * @return 处理的关键帧的数量  
     */
    int pairGnssOdomInBatch(std::vector<KeyFrame::Ptr> &odom_queue, std::deque<Sensor::GNSSData> &gnss_queue);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 将关键帧与GNSS的数据进行配对  
     * @param GNSS_queue GNSS队列 
     * @param keyframe 待匹配的关键帧 
     * @return 匹配状态  
     * @details 匹配失败 有3种情况  ：1、 没有GNSS数据  2、最早GNSS数据都比当前关键帧晚很多   
     *                             3、 插值的两个GNSS数据间隔太大   4、还少一个插值的GNSS数据 
     */
    bool pairGnssOdomInSingle(std::deque<Sensor::GNSSData> &GNSS_queue, KeyFrame::Ptr const& keyframe);
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 更新滑动窗口 
     * @return 是否需要优化 
     */
    bool updataSlidingWindows();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 局部优化      里程计约束+地面约束+GNSS约束   
    void slidingWindowLocalOptimize();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 全局优化      里程计约束+GNSS约束+回环约束+(可选)地面约束   
    void globalOptimize(const Loop::Ptr& loop);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
     */
    void optimization();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 发布地图给rviz显示
     */
    void mapPointsPublish();

  private:
    ros::NodeHandle nh;  
    ros::NodeHandle private_nh;  
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imu_sub;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> navsat_sub;
    std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> Sync_odomCloud;
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::NavSatFix>> Sync_GPSIMU;

    ros::Subscriber floor_sub;
    //ros::Subscriber odom_sub;
    //ros::Subscriber cloud_sub;
    // 可视化
    ros::Publisher markers_pub;

    std::string map_frame_id;
    std::string odom_frame_id;
    // optOdom坐标系到map坐标系的变换  
    Eigen::Isometry3d trans_odom2map = Eigen::Isometry3d::Identity();

    ros::Publisher odom2map_pub;

    std::string odom_topic;
    std::string points_topic;

    ros::Publisher map_points_pub;

    ros::Publisher pubGNSSPath;                 // 发布轨迹
    nav_msgs::Path GNSSPath;                    // 记录轨迹  
    //tf::TransformListener tf_listener;

    //ros::ServiceServer dump_service_server;
    //ros::ServiceServer save_map_service_server;

    // keyframe queue
    std::string base_frame_id;

    // 新添加的关键帧的处理队列
    std::vector<KeyFrame::Ptr> new_keyframe_queue;

    // floor_coeffs queue
    double floor_edge_stddev;

    //std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

    // for map cloud generation
    double map_cloud_resolution;

    std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
    //std::unique_ptr<MapCloudGenerator> map_cloud_generator;

    // 局部优化每次处理的最大帧数  
    int max_keyframes_per_update;
    std::vector<KeyFrame::Ptr> wait_optimize_keyframes;
    std::vector<KeyFrame::Ptr> wait_loopDetect_keyframes;
    // 局部优化的滑动窗口 
    std::deque<KeyFrame::Ptr> sliding_windows;
    int sliding_windows_size = 50;  

    g2o::VertexPlane* floor_plane_node;

    std::vector<KeyFrame::Ptr> keyframes;                            // 保存关键帧信息
    std::vector<Loop::Ptr> Loops;                                    // 保存回环检测   
    std::vector<pair<Eigen::Vector3d,Eigen::Vector3d>> EdgeSE3;      // 保存连接节点的边  
    std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

    //std::unique_ptr<GraphSLAM> graph_slam;
    //std::unique_ptr<LoopDetector> loop_detector;
    // 用于判断是否需要添加关键帧 
    std::unique_ptr<KeyframeUpdater> keyframe_updater;
    std::unique_ptr<LoopDetector> loop_detector;
    std::unique_ptr<InformationMatrixCalculator> inf_calclator;    // 计算信息矩阵  
    // std::unique_ptr<GraphSLAM> graph_slam;                         // 后端优化  
    std::unique_ptr<MapCloudGenerator> map_cloud_generator;
    std::unique_ptr<NmeaSentenceParser> nmea_parser;               // nmea数据解析
    //std::unique_ptr<InformationMatrixCalculator> inf_calclator;

    std::mutex keyframe_queue_mutex;
    std::mutex floor_coeffs_queue_mutex;
    std::mutex keyframes_snapshot_mutex;
    std::mutex GNSS_queue_mutex;

    //ros::WallTimer optimization_timer;
    //ros::WallTimer map_publish_timer;

    g2o::VertexSE3* anchor_node;
    g2o::EdgeSE3* anchor_edge;

    string fix_first_node_stddev;                       // 第一个节点的信息矩阵参数
    string odometry_edge_robust_kernel;                 // 里程计边鲁棒核函数开关
    double odometry_edge_robust_kernel_size;
    string loop_closure_edge_robust_kernel;             // 闭环边的鲁棒核函数   
    double loop_closure_edge_robust_kernel_size;  
    bool fix_first_node_adaptive;
    int num_iterations; 
    double gps_time_offset;
    string imu_orientation_edge_robust_kernel;
    double imu_orientation_edge_robust_kernel_size;
    double imu_orientation_edge_stddev;
    double imu_time_offset;
    Eigen::Vector3d Gnss_init_Map_loc = {0,0,0};     // GNSS初始化时  在地图上的坐标  
    int init_imu_num = 1;                            // 用于初始化Map系位姿的imu数量     
    // GPS权重  
    double gps_edge_stddev_xy;
    double gps_edge_stddev_z;
    std::string key_frames_path;

    // 数据队列
    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> cloudBuf;
    std::deque<Sensor::GNSSData> GNSS_queue;

    bool enable_imu_orientation;
    bool gnss_origin_position_inited = false;
    bool IMU_pose_inited = true;
    bool enable_GNSS_optimize = false;  
    bool enable_planeConstraint_optimize = false;  
    int GNSS_optimize_freq;
    int planeConstraint_optimize_freq;

    std::mutex mBuf;
    // 记录关键帧的index  
    int KF_index = 0;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    Eigen::Quaterniond rotation_map2enu = Eigen::Quaterniond::Identity();       // Map到ENU系的旋转
    Eigen::Vector3d gravity_in_map = {0,0,0};  
    Eigen::Quaterniond MAP2ENU2 = {0,0,0,0};       // Map到ENU系的旋转
    Eigen::Matrix4d Tme = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Til;
    ros::Time Optimize_previous_time = ros::Time(0);
    ros::Time Map_updata_previous_time = ros::Time(0);
}; // class SlidingWindowBackEnd

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
visualization_msgs::MarkerArray SlidingWindowBackEnd::createMarkerArray(const ros::Time& stamp) 
{
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(4);
  // node markers    位姿节点
  visualization_msgs::Marker& traj_marker = markers.markers[0];
  traj_marker.header.frame_id = "map";
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "nodes";
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;
  // 数量
  traj_marker.points.resize(keyframes.size()+sliding_windows.size()+new_keyframe_queue.size());
  // 颜色
  traj_marker.colors.resize(keyframes.size()+sliding_windows.size()+new_keyframe_queue.size());
  // 新增位姿节点
  for(int i=0; i<new_keyframe_queue.size(); i++) 
  {
    // 设置位置
    Eigen::Vector3d pos = new_keyframe_queue[i]->Pose.translation();
    traj_marker.points[i].x = pos.x();
    traj_marker.points[i].y = pos.y();
    traj_marker.points[i].z = pos.z();
    // 颜色
    traj_marker.colors[i].r = 1.0;
    traj_marker.colors[i].g = 0;
    traj_marker.colors[i].b = 0.0;
    traj_marker.colors[i].a = 1.0;
  }   
  // 滑动窗口节点位姿
  // 优化后位姿节点 
  for(int i=0; i<sliding_windows.size(); i++) 
  {
    // 设置位置
    Eigen::Vector3d pos = sliding_windows.at(i)->Pose.translation();
    traj_marker.points[new_keyframe_queue.size()+i].x = pos.x();
    traj_marker.points[new_keyframe_queue.size()+i].y = pos.y();
    traj_marker.points[new_keyframe_queue.size()+i].z = pos.z();
    // 颜色
    traj_marker.colors[new_keyframe_queue.size()+i].r = 0.0;
    traj_marker.colors[new_keyframe_queue.size()+i].g = 1.0;
    traj_marker.colors[new_keyframe_queue.size()+i].b = 0.0;
    traj_marker.colors[new_keyframe_queue.size()+i].a = 1.0;
  }  
  // 优化后位姿节点 
  for(int i=0; i<keyframes.size(); i++) 
  {
    // 设置位置
    Eigen::Vector3d pos = keyframes[i]->Pose.translation();
    traj_marker.points[new_keyframe_queue.size()+sliding_windows.size()+i].x = pos.x();
    traj_marker.points[new_keyframe_queue.size()+sliding_windows.size()+i].y = pos.y();
    traj_marker.points[new_keyframe_queue.size()+sliding_windows.size()+i].z = pos.z();
    // 颜色
    traj_marker.colors[new_keyframe_queue.size()+sliding_windows.size()+i].r = 0;
    traj_marker.colors[new_keyframe_queue.size()+sliding_windows.size()+i].g = 0;
    traj_marker.colors[new_keyframe_queue.size()+sliding_windows.size()+i].b = 1.0;
    traj_marker.colors[new_keyframe_queue.size()+sliding_windows.size()+i].a = 1.0;
  }   
  // edge markers  边
  visualization_msgs::Marker& edge_marker = markers.markers[2];
  edge_marker.header.frame_id = "map";
  edge_marker.header.stamp = stamp;
  edge_marker.ns = "edges";
  edge_marker.id = 2;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.1;
  // 这里要注意 ！！！！！！！！！！！！！！！！！
  edge_marker.points.resize(keyframes.size() * 2 * 2 * 2 + sliding_windows.size() * 2 * 2 * 2 + Loops.size() * 2);
  edge_marker.colors.resize(keyframes.size() * 2 * 2 * 2 + sliding_windows.size() * 2 * 2 * 2 + Loops.size() * 2);
  int i=0;

  for(int num = 0; num<keyframes.size(); num++) 
  {
    // 里程计边    Pc
    Eigen::Vector3d pt1 = keyframes[num]->Pose.translation();
    // Twc*Tlc^-1 = Twl
    Eigen::Vector3d pt2 = (keyframes[num]->Pose*keyframes[num]->deltaOdom.inverse()).translation();
    // 设置位置关系     每个frame 2个点 
    edge_marker.points[i*2].x = pt1.x();
    edge_marker.points[i*2].y = pt1.y();
    edge_marker.points[i*2].z = pt1.z();
    edge_marker.points[i*2 + 1].x = pt2.x();
    edge_marker.points[i*2 + 1].y = pt2.y();
    edge_marker.points[i*2 + 1].z = pt2.z();

    edge_marker.colors[i*2].r = 1.0;
    edge_marker.colors[i*2].g = 2.0;
    edge_marker.colors[i*2].a = 1.0;
    edge_marker.colors[i*2 + 1].r = 1.0;
    edge_marker.colors[i*2 + 1].g = 2.0;
    edge_marker.colors[i*2 + 1].a = 1.0;
    i++;
    
    if(enable_GNSS_optimize)
    {
      // GNSS 先验边     2个点  
      if(keyframes[num]->GNSS_Valid) {
        Eigen::Vector3d pt1 = keyframes[num]->Pose.translation();  
        Eigen::Vector3d pt2 = keyframes[num]->utm_coord;

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z()+1;
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;
        i++;
      }
    }
    // 地面约束边  
    if(enable_planeConstraint_optimize)
    {
      if(keyframes[num]->planeConstraint_Valid) 
      {
       Eigen::Vector3d plane = {pt1[0], pt1[1], 0};

       edge_marker.points[i*2].x = pt1.x();
       edge_marker.points[i*2].y = pt1.y();
       edge_marker.points[i*2].z = pt1.z();
       edge_marker.points[i*2 + 1].x = plane.x();
       edge_marker.points[i*2 + 1].y = plane.y();
       edge_marker.points[i*2 + 1].z = plane.z()-1;

       edge_marker.colors[i*2].r = 1.0;
       edge_marker.colors[i*2].a = 2.0;
       edge_marker.colors[i*2 + 1].r = 1.0;
       edge_marker.colors[i*2 + 1].a = 2.0;
       i++;
      }
    }
  }

  for(int num = 0; num<sliding_windows.size(); num++) 
  {
    // 里程计边    Pc
    Eigen::Vector3d pt1 = sliding_windows.at(num)->Pose.translation();
    // Twc*Tlc^-1 = Twl
    Eigen::Vector3d pt2 = (sliding_windows.at(num)->Pose*sliding_windows.at(num)->deltaOdom.inverse()).translation();
    // 设置位置关系     每个frame 2个点 
    edge_marker.points[i*2].x = pt1.x();
    edge_marker.points[i*2].y = pt1.y();
    edge_marker.points[i*2].z = pt1.z();
    edge_marker.points[i*2 + 1].x = pt2.x();
    edge_marker.points[i*2 + 1].y = pt2.y();
    edge_marker.points[i*2 + 1].z = pt2.z();

    edge_marker.colors[i*2].r = 1.0;
    edge_marker.colors[i*2].g = 2.0;
    edge_marker.colors[i*2].a = 1.0;
    edge_marker.colors[i*2 + 1].r = 1.0;
    edge_marker.colors[i*2 + 1].g = 2.0;
    edge_marker.colors[i*2 + 1].a = 1.0;
    i++;
    
    if(enable_GNSS_optimize)
    {
      // GNSS 先验边     2个点  
      if(sliding_windows.at(num)->GNSS_Valid) {
        Eigen::Vector3d pt1 = sliding_windows.at(num)->Pose.translation();  
        Eigen::Vector3d pt2 = sliding_windows.at(num)->utm_coord;

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z()+1;
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;
        i++;
      }
    }
    // 地面约束边  
    if(enable_planeConstraint_optimize)
    {
      if(sliding_windows.at(num)->planeConstraint_Valid) {

       Eigen::Vector3d plane = {pt1[0], pt1[1], 0};

       edge_marker.points[i*2].x = pt1.x();
       edge_marker.points[i*2].y = pt1.y();
       edge_marker.points[i*2].z = pt1.z();
       edge_marker.points[i*2 + 1].x = plane.x();
       edge_marker.points[i*2 + 1].y = plane.y();
       edge_marker.points[i*2 + 1].z = plane.z()-1;

       edge_marker.colors[i*2].r = 1.0;
       edge_marker.colors[i*2].a = 2.0;
       edge_marker.colors[i*2 + 1].r = 1.0;
       edge_marker.colors[i*2 + 1].a = 2.0;
       i++;
      }
    }
  }
  // 回环检测的边   2个点 
  for(auto& loop:Loops)
  { 
    Eigen::Vector3d pt1 = loop->key1->Pose.translation();    // 新帧  
    Eigen::Vector3d pt2 = (loop->key1->Pose.matrix()*loop->relative_pose.inverse().cast<double>()).block<3,1>(0,3);     // 与新帧闭环的老帧   Twc * Tlc^-1

    edge_marker.points[i*2].x = pt1.x();
    edge_marker.points[i*2].y = pt1.y();
    edge_marker.points[i*2].z = pt1.z();
    edge_marker.points[i*2 + 1].x = pt2.x();
    edge_marker.points[i*2 + 1].y = pt2.y();
    edge_marker.points[i*2 + 1].z = pt2.z();

    edge_marker.colors[i*2].r = 2.0;
    edge_marker.colors[i*2].a = 2.0;
    edge_marker.colors[i*2 + 1].r = 2.0;
    edge_marker.colors[i*2 + 1].a = 2.0;
    i++;
  }

  return markers;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::gnssInitialize(Sensor::GNSSData &first_gnss_data, Eigen::Quaterniond const& first_gnss_rotate, 
                                          KeyFrame::Ptr align_keyframe)
{
  // 进行初始化      将当前  gnss_data 的LLT数据 作为原点的 值    
  first_gnss_data.InitOriginPosition();
  Eigen::Quaterniond Rel = first_gnss_rotate * Eigen::Quaterniond(Til.block<3,3>(0,0));    // Rel = Rei * Ril   激光雷达到ENU系的旋转  
  rotation_map2enu =  Rel * align_keyframe->Pose.matrix().block<3,3>(0,0).inverse();    // Rem = Rel * Rml^-1      局部MAP系到ENU系的旋转  
  Eigen::Isometry3d Tem = Eigen::Isometry3d::Identity();
  // 构建 map系到enu系的变换 , 认为它们只有旋转的变化, enu系的原点还是在map系原点上  
  Tem.rotate(rotation_map2enu);
  Tem.pretranslate(Eigen::Vector3d(0,0,0));
  // 计算当前ENU坐标系坐标原点(GNSS初始化时的坐标系)相对与原点ENU坐标系的位置
  // 首先求得imu在MAP系下的位姿      
  Eigen::Matrix4d Tmi = align_keyframe->Pose.matrix() * Til.matrix().inverse();     // Tmi = Tml * Tli
  Gnss_init_Map_loc = rotation_map2enu * Tmi.block<3,1>(0,3);    // Rem * tmi = tei  ,  tmi 为 map 系下 m->i的位移向量   
  ROS_INFO_STREAM("GNSS init OK !  Map loc: "<<Gnss_init_Map_loc);
  ROS_INFO_STREAM("MAP->ENU:  "<< Tem.matrix());
  trans_odom2map = Tem;  
  // 如果有订阅者  发布odom到map坐标系的变换  
  if(odom2map_pub.getNumSubscribers()) 
  {
    ROS_INFO_STREAM("BackEnd_node - trans_odom2map: "<<std::endl<<trans_odom2map.matrix());   
    // 构造 ROS Msg
    geometry_msgs::TransformStamped ts = matrix2transform(ros::Time::now(), trans_odom2map.matrix().cast<float>(), map_frame_id, odom_frame_id);
    odom2map_pub.publish(ts);
  }
  // 将之前的坐标系转为导航系下
  // 更新keyframe的Pose     由于 坐标系只是变化了一个旋转  所以这里乘的Tem 只有旋转没有平移 
  // 之后 会更新 trans_odom2map 
  for(auto& keyframe:keyframes)
  {
    keyframe->Pose = Tem * keyframe->Pose;
  }

  // 遍历本次优化的全部关键帧  
  for(deque<KeyFrame::Ptr>::iterator keyframe_it = sliding_windows.begin(); keyframe_it != sliding_windows.end(); ++keyframe_it)
  {
    (*keyframe_it)->Pose = Tem * (*keyframe_it)->Pose;
  }
  // 还需要对滑窗中的帧进行转换 
  for(auto& keyframe:new_keyframe_queue)
  {
    keyframe->Pose = Tem * keyframe->Pose;
  }

  // 如果初始化失败   这个就要保持false  
  gnss_origin_position_inited = true;

  return;   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::transformWgs84ToMap(Sensor::GNSSData &gnss_data)
{
  // 如果初始化成功  
  if(gnss_origin_position_inited&&IMU_pose_inited)
  {
    gnss_data.UpdateXYZ();       // 计算ENU坐标  以初始化时IMU位置处ENU坐标系为原点    
    // 求得组合导航仪位于Map坐标系下的坐标         
    gnss_data.Lidar_Map_coords.x() = gnss_data.local_E + Gnss_init_Map_loc.x();
    gnss_data.Lidar_Map_coords.y() = gnss_data.local_N + Gnss_init_Map_loc.y();
    gnss_data.Lidar_Map_coords.z() = gnss_data.local_U + Gnss_init_Map_loc.z();  
    
    // 从IMU的UTM坐标转换为Lidar的UTM坐标   tel = Tei * til = Rei*til + tei  
    gnss_data.Lidar_Map_coords =  gnss_data.IMU_orientation * Til.block<3,1>(0,3) + gnss_data.Lidar_Map_coords;
    gnss_data.Lidar_Map_orientation = gnss_data.IMU_orientation * Eigen::Quaterniond(Til.block<3,3>(0,0));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 前端 点云与里程计信息回调函数  
 * @param odom_msg 包括 位姿增量、里程计位姿、位姿增量协方差矩阵   
 * 
 */
void SlidingWindowBackEnd::CloudCallback(const nav_msgs::OdometryConstPtr& odom_msg, 
                                         const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
{
  static double last_odom_t = -1;
  // 检查时间戳是否乱序  
  // 检查时间戳 看是否乱序    
  if (odom_msg->header.stamp.toSec() <= last_odom_t)
  {
      ROS_WARN("odom message in disorder!");
      return;
  }

  last_odom_t = odom_msg->header.stamp.toSec(); 

  // 线程锁开启
  std::lock_guard<std::mutex> lock(keyframe_queue_mutex); 
  // ROS_INFO_STREAM("backend receive !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  const ros::Time& stamp = odom_msg->header.stamp;
  // 读取前端里程计的odom数据  
  Eigen::Isometry3d odom = odom2isometry(odom_msg);                               // ROS ->Eigen
  // ROS_INFO_STREAM("receive odom cloud");
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if(base_frame_id.empty()) 
  {
    base_frame_id = cloud_msg->header.frame_id;
  }

  // 是否需要添加关键帧
  if(!keyframe_updater->Update(odom)) 
  {
    return;
  }
  
  // 把关键帧点云存储到硬盘里     不消耗内存
  // 如果未来维护关键帧包括关键帧的删除或替换的话 , 那么 KF_index 的也需要去维护 
  std::string file_path = key_frames_path + "/key_frame_" + std::to_string(KF_index) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *cloud);
  
  // 通过点云与里程计和累计距离等来创建关键帧   实际的关键帧中就不包含点云数据了  
  KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, KF_index, cloud));
  keyframe->deltaOdom = keyframe_updater->GetdeltaOdom();      // 获取该关键帧与上一关键帧之间的增量位姿  
  // 获取在MAP系下的坐标   注意  这里只是为了可视化   在关键帧处理函数中会用最新的优化矩阵去转换 
  keyframe->Pose = trans_odom2map * keyframe->odom;  
  new_keyframe_queue.push_back(keyframe);     // 加入处理队列
  // KF index 更新  
  KF_index++;

  // 可视化     
  if(markers_pub.getNumSubscribers()) 
  {
    auto markers = createMarkerArray(ros::Time::now());
    markers_pub.publish(markers);
  }  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::GnssCallback(const sensor_msgs::ImuConstPtr& imu_msg, 
                                        const sensor_msgs::NavSatFixConstPtr& navsat_msg)
{
  static double last_t = -1;
  std::lock_guard<std::mutex> lock(GNSS_queue_mutex);
  // 检查时间戳是否乱序  
  // 检查时间戳 看是否乱序    
  if (imu_msg->header.stamp.toSec() <= last_t)
  {
      ROS_WARN("GNSS message in disorder!");
      return;
  }
  last_t = imu_msg->header.stamp.toSec(); 

  Sensor::GNSSData gnss_data;
  
  gnss_data.time = navsat_msg->header.stamp + ros::Duration(gps_time_offset);
  gnss_data.latitude = navsat_msg->latitude;
  gnss_data.longitude = navsat_msg->longitude;
  gnss_data.altitude = navsat_msg->altitude;
  gnss_data.status = navsat_msg->status.status;
  gnss_data.service = navsat_msg->status.service;
  // IMU 
  gnss_data.imu = imu_msg;
  gnss_data.IMU_orientation.w() = imu_msg->orientation.w;
  gnss_data.IMU_orientation.x() = imu_msg->orientation.x;
  gnss_data.IMU_orientation.y() = imu_msg->orientation.y;
  gnss_data.IMU_orientation.z() = imu_msg->orientation.z; 

  // 如果初始化成功 
  if(gnss_origin_position_inited&&IMU_pose_inited)
  {
    transformWgs84ToMap(gnss_data);      // 将WGS84坐标值转换为Map系坐标

    // 发布odom
    geometry_msgs::PoseStamped Pose_lidar_in_map; 
    Pose_lidar_in_map.header.stamp = gnss_data.time;
    Eigen::Quaterniond Qml = Eigen::Quaterniond::Identity();
    Eigen::Vector3d tml = Eigen::Vector3d(0,0,0);
    tml = gnss_data.Lidar_Map_coords;
    Qml = gnss_data.Lidar_Map_orientation;
    Pose_lidar_in_map.pose.position.x = tml.x();
    Pose_lidar_in_map.pose.position.y = tml.y();
    Pose_lidar_in_map.pose.position.z = tml.z();
    Pose_lidar_in_map.pose.orientation.w = Qml.w();
    Pose_lidar_in_map.pose.orientation.x = Qml.x();
    Pose_lidar_in_map.pose.orientation.y = Qml.y();
    Pose_lidar_in_map.pose.orientation.z = Qml.z();
    GNSSPath.header.stamp = Pose_lidar_in_map.header.stamp;
    GNSSPath.poses.push_back(Pose_lidar_in_map);

    std::cout<<" GNSS pos x: " << tml.x() << " y: " << tml.y() << " z: " << tml.z() << std::endl; 

    // 控制发布频率
    static int i = 0;
    if(i++==2)
    {
      GNSSPath.header.frame_id = map_frame_id; // odom坐标
      pubGNSSPath.publish(GNSSPath);
      i = 0;
    }
  }

  // 原始数据直接存放
  GNSS_queue.push_back(gnss_data);
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int SlidingWindowBackEnd::pairGnssOdomInBatch(std::vector<KeyFrame::Ptr> &odom_queue, std::deque<Sensor::GNSSData> &gnss_queue)
{ 
  /************************** 首先进行 GNSS 与 关键帧待处理队列 wait_optimize_keyframes 数据对齐 ***************************/    
  // 里程计容器空   
  if(odom_queue.empty()) 
  {
    GNSS_queue.clear();    // 清空无效的GNSS数据 
    return 0;  
  }
  // GNSS 容器为空 
  if(gnss_queue.empty()) 
  {
    return odom_queue.size();  
  }
  // ROS_INFO_STREAM("GNSS_queue size: "<<GNSS_queue.size());
  // 如果GNSS 队尾 即 最新的GNSS数据早于 最早的一帧关键帧数据  说明没有与关键帧匹配的GNSS数据  直接退出
  if(gnss_queue.back().time.toSec()<odom_queue.front()->stamp.toSec())
  {
    // ROS_INFO_STREAM("GNSS_queue.back().time < wait_optimize_keyframes.front() "<< GNSS_queue.back().time.toSec() - wait_optimize_keyframes.front()->stamp.toSec());
    GNSS_queue.clear();    // 清空无效的GNSS数据 
    return odom_queue.size();
  }
  // 如果GNSS 队首 即 最早的GNSS数据晚于 最晚的一帧关键帧数据  说明没有与关键帧匹配的GNSS数据  直接退出
  if(gnss_queue.front().time.toSec()>odom_queue.back()->stamp.toSec())
  {
    // ROS_INFO_STREAM("GNSS_queue.front().time > wait_optimize_keyframes.back() "<<GNSS_queue.front().time.toSec() - wait_optimize_keyframes.back()->stamp.toSec());
    return odom_queue.size();;
  }
  
  int count = 0;     // 处理的帧的数量 
  // 剩下的情况说明都有GNSS与关键帧有匹配的情况
  // ROS_INFO_STREAM("KEYFRAME SIZE: "<<wait_optimize_keyframes.size()<<"gps num: "<<gps_queue.size());
  for(auto& keyframe:odom_queue)
  { 
    // ROS_INFO_STREAM(" time diff : "<< keyframe->stamp.toSec() - gps_queue.front().time);
    // GNSS_queue.front() 的数据 比 keyframe 早很多 
    while(keyframe->stamp.toSec() - GNSS_queue.front().time.toSec() > 0.10)
    {
      // ROS_INFO_STREAM("keyframe - gps > 0.1 : "<< keyframe->stamp.toSec() - GNSS_queue.front().time.toSec());
      GNSS_queue.pop_front();   // 头数据丢弃 
      if(GNSS_queue.empty())  return count;   
    }
    // gps 比 keyframe 晚很多     无法匹配
    if(keyframe->stamp.toSec() - GNSS_queue.front().time.toSec() < -0.005)
    {
      count++;  
      // ROS_INFO_STREAM("keyframe - gps < -0.01" << keyframe->stamp.toSec() - GNSS_queue.front().time.toSec());
      continue;
    }
    // 当前数据合格    GNSSData 包括 经纬高以及 IMU测得的数据  
    Sensor::GNSSData gnss_data = GNSS_queue.front();                  
    GNSS_queue.pop_front();                    // 将头数据弹出
    Sensor::GNSSData lidar_gnss_data = gnss_data;      // 插值后 lidar位置处 的数据 
    // 判断是否需要插值     如果GNSS与keyframe的数据差距在 5ms以上 那么进行插值 
    if(fabs(keyframe->stamp.toSec() - gnss_data.time.toSec()) > 0.005)
    {
       if(GNSS_queue.empty())  
       {
         // 将GNSS数据补回去 
         GNSS_queue.push_back(gnss_data);  
         return count;       
       }

       Sensor::GNSSData gnss_data_2 = GNSS_queue.front();      // 取下一个      注意尾数据不要丢弃掉   要留着给下一次插值  
       double t1 = keyframe->stamp.toSec() - gnss_data.time.toSec();
       double t2 = gnss_data_2.time.toSec() - keyframe->stamp.toSec();
       // 时间间隔太大  那么线性插值就不准了 
       if(t1+t2 > 0.2)   
       {
         ROS_INFO_STREAM("t1+t2 > 0.2"<<t1+t2);
         count++;
         continue;
       }
      /************************************************* 进行插值  插值出 关键帧对应 IMU的 经纬高, 姿态 ********************************************/
      // 计算插值系数     
      double front_scale = t2 / (t2+t1);
      double back_scale = t1 / (t2+t1);
      // 对GPS进行插值 
      // 是否初始化   
      if (!gnss_origin_position_inited) 
      { // 如果没有对齐GNSS与Lidar的坐标  那么对经纬高进行插值  准备坐标对齐 
        lidar_gnss_data.longitude = gnss_data_2.longitude*back_scale + gnss_data.longitude*front_scale;
        lidar_gnss_data.latitude = gnss_data_2.latitude*back_scale + gnss_data.latitude*front_scale;
        lidar_gnss_data.altitude = gnss_data_2.altitude*back_scale + gnss_data.altitude*front_scale; 
        // 对IMU的姿态进行插值 设置给  keyframe->orientation 
        Eigen::Quaterniond imu_front,imu_back;
        imu_front.w() = gnss_data.imu->orientation.w;
        imu_front.x() = gnss_data.imu->orientation.x;
        imu_front.y() = gnss_data.imu->orientation.y;
        imu_front.z() = gnss_data.imu->orientation.z;
        imu_back.w() = gnss_data_2.imu->orientation.w;
        imu_back.x() = gnss_data_2.imu->orientation.x;
        imu_back.y() = gnss_data_2.imu->orientation.y;
        imu_back.z() = gnss_data_2.imu->orientation.z;

        Eigen::Quaterniond rotate_by_gnss;
        // 球面插值  
        lidar_gnss_data.IMU_orientation = imu_front.slerp(back_scale, imu_back);      
        /**************** 如果GNSS没有初始化  那么进行初始化   初始化主要是求 map->enu 的 Tem, 以及 Gnss_init_Map_loc   ***********************************/
        gnssInitialize(lidar_gnss_data, lidar_gnss_data.IMU_orientation, keyframe);    // 进行GNSS与MAP对齐  
        // 初始化后转换为Map坐标
        transformWgs84ToMap(lidar_gnss_data);
      }
      else
      {
        lidar_gnss_data.Lidar_Map_coords.x() = gnss_data_2.Lidar_Map_coords.x()*back_scale + gnss_data.Lidar_Map_coords.x()*front_scale;
        lidar_gnss_data.Lidar_Map_coords.y() = gnss_data_2.Lidar_Map_coords.y()*back_scale + gnss_data.Lidar_Map_coords.y()*front_scale;
        lidar_gnss_data.Lidar_Map_coords.z() = gnss_data_2.Lidar_Map_coords.z()*back_scale + gnss_data.Lidar_Map_coords.z()*front_scale;  
        // 对IMU的姿态进行插值 设置给  keyframe->orientation 
        Eigen::Quaterniond imu_front,imu_back;
        imu_front.w() = gnss_data.Lidar_Map_orientation.w();
        imu_front.x() = gnss_data.Lidar_Map_orientation.x();
        imu_front.y() = gnss_data.Lidar_Map_orientation.y();
        imu_front.z() = gnss_data.Lidar_Map_orientation.z();
        imu_back.w() = gnss_data_2.Lidar_Map_orientation.w();
        imu_back.x() = gnss_data_2.Lidar_Map_orientation.x();
        imu_back.y() = gnss_data_2.Lidar_Map_orientation.y();
        imu_back.z() = gnss_data_2.Lidar_Map_orientation.z();
        // 球面插值  
        lidar_gnss_data.Lidar_Map_orientation = imu_front.slerp(back_scale, imu_back);       
      }
      count++;
    }
    else
    {
      count++;
    }
    // 获得Lidar的GNSS在Map系的坐标     
    keyframe->utm_coord = lidar_gnss_data.Lidar_Map_coords;
    keyframe->orientation = lidar_gnss_data.Lidar_Map_orientation.normalized();   
    keyframe->GNSS_Valid = true;
  }

  return count;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SlidingWindowBackEnd::processDataInBatch()
{
  // 对新加入的关键帧进行处理      
  // 如果没有新的关键帧  
  if(new_keyframe_queue.empty()) 
  {
    return false;
  }
  // count 为 new_keyframe_queue 中处理的关键帧的数量 
  int count = pairGnssOdomInBatch(new_keyframe_queue, GNSS_queue);
  std::cout<<" pairGnssOdomInBatch down, count: "<< count << std::endl;
  std::copy(new_keyframe_queue.begin(), new_keyframe_queue.begin() + count, std::back_inserter(wait_optimize_keyframes));
  std::copy(new_keyframe_queue.begin(), new_keyframe_queue.begin() + count, std::back_inserter(wait_loopDetect_keyframes));
  std::cout<<" copy down: "<< std::endl;

  static int n = 0; 
  // 将里程计数据存储
  if(count > 0 && SaveOdometryPath)
  {
    Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();  
    for(auto const& frame : wait_optimize_keyframes)
    { 
      if(frame->GNSS_Valid)
      {
        std::cout<<" n: "<<n<<std::endl;
        if(n<3)
        {
          n++;
          break;  
        }
        gt_pose.block<3,1>(0, 3) = frame->utm_coord; 
        gt_pose.block<3,3>(0, 0) =  frame->orientation.matrix(); 
        SaveTrajectory(gt_pose, frame->Pose.matrix(), "/slam_data/trajectory", "/slam_data/trajectory/ground_truth.txt",
                       "/slam_data/trajectory/est_path.txt");
      }
      else{
        std::cout<<" invalid GNSS "<<std::endl;
      }
    }
  }
  std::cout<<" SaveTrajectory down "<< std::endl;

  new_keyframe_queue.erase(new_keyframe_queue.begin(), new_keyframe_queue.begin() + count);
  std::cout<<" erase down "<< std::endl;
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::processDataInSingle()
{
  // 对新加入的关键帧进行处理      
  // 如果没有新的关键帧  
  if(new_keyframe_queue.empty()) 
  {
    return;
  }
  // 处理的数量  
  int num_processed = std::min<int>(new_keyframe_queue.size(), max_keyframes_per_update);
  int num = 0;
  // 遍历全部关键帧队列       
  for(int i=0; i<num_processed; i++) 
  {
    // 从keyframe_queue中取出关键帧
    const auto& keyframe = new_keyframe_queue[i];
    // 获取在MAP系下的坐标 
    keyframe->Pose = trans_odom2map * keyframe->odom;  
    // 与GNSS进行匹配 
    // 寻找有无匹配的GPS   有则
    if(!pairGnssOdomInSingle(GNSS_queue, keyframe))                // return true 匹配完成   false 继续等待数据  
    { 
      if(ros::Time::now().toSec() - keyframe->stamp.toSec() < 1)   // 最多等待1s 
      {
        break;
      }
    }else
    { 
      // 匹配上GNSS数据了
      // 按照一定频率添加GNSS约束  
      static int gnss_freq_count = 0;
      if(gnss_freq_count <= 0) {
        keyframe->GNSS_Valid = true;  
        gnss_freq_count = GNSS_optimize_freq;  
      }
      else{
        gnss_freq_count--;
      }
      
      if(keyframe->gnss_matched) 
      {
        Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();  
        
        gt_pose.block<3,1>(0, 3) = keyframe->utm_coord; 
        gt_pose.block<3,3>(0, 0) =  keyframe->orientation.matrix(); 
        SaveTrajectory(gt_pose, keyframe->Pose.matrix(), "/slam_data/trajectory", "/slam_data/trajectory/ground_truth.txt",
                        "/slam_data/trajectory/est_path.txt");
      }
    }

    // 按照一定频率添加先验平面约束  
    static int planeConstraint_freq_count = 0;
    if(planeConstraint_freq_count <= 0) {
      keyframe->planeConstraint_Valid = true;  
      planeConstraint_freq_count = planeConstraint_optimize_freq;  
    }
    else{
      planeConstraint_freq_count--;
    }
    
    // 放置到待优化容器中     
    wait_optimize_keyframes.emplace_back(std::move(keyframe));   
    // 放置到待回环检测容器中
    wait_loopDetect_keyframes.emplace_back(std::move(keyframe)); 
    num++;  
  }

  new_keyframe_queue.erase(new_keyframe_queue.begin(), new_keyframe_queue.begin() + num);     //  [first,last) 

  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SlidingWindowBackEnd::pairGnssOdomInSingle(std::deque<Sensor::GNSSData> &GNSS_queue, KeyFrame::Ptr const& keyframe)
{
  uint8_t flag;   
  Sensor::GNSSData paired_gnss; 

  // 容器为空   表示没有GNSS      匹配结束   
  if(GNSS_queue.empty()) 
  {
    std::cout<<"GNSS_queue.empty()"<<std::endl;
    return true;  
  }

  // 最早的GNSS数据晚于当前keyframe 5ms以上   那么无法匹配   匹配结束 
  if(GNSS_queue.front().time.toSec()-keyframe->stamp.toSec() > 0.005)
  {
    std::cout<<"GNSS_queue.front().time.toSec()-keyframe->stamp.toSec() > 0.005"<<std::endl;
    return true;
  }

  // 不需要插值   如果最早的GNSS数据和keyframe很接近那么就直接匹配了   
  if(fabs(GNSS_queue.front().time.toSec()-keyframe->stamp.toSec()) <= 0.005)
  {
    std::cout<<"fabs(GNSS_queue.front().time.toSec()-keyframe->stamp.toSec()) <= 0.005"<<std::endl;
    paired_gnss = GNSS_queue.front();  
  }

  // 需要插值  
  if(keyframe->stamp.toSec() - GNSS_queue.front().time.toSec() > 0.005)
  { 
    // 当前数据合格    GNSSData 包括 经纬高以及 IMU测得的数据  
    Sensor::GNSSData gnss_data = GNSS_queue.front();                  
    GNSS_queue.pop_front();         // 将头数据弹出

    // 获取下一个数据  
    if(GNSS_queue.empty())                // 如果为空  那么配对失败   但是 可以等待下一个GNSS数据  
    { 
      // 如果该gnss数据太早了  那么就算有下一个gnss数据那么也是无效的    匹配结束  
      if(keyframe->stamp.toSec() - gnss_data.time.toSec() > 0.15)
      {
        return true;
      }
      // 否则还可以利用该数据进行插值 
      GNSS_queue.push_front(gnss_data);   // 把数据放回去  
      return false;                       // 等待下一次gnss数据  
    }

    // 寻找用于插值的下一个GNSS数据 
    while(!GNSS_queue.empty())
    {
      Sensor::GNSSData gnss_data_2 = GNSS_queue.front();      // 取下一个      注意尾数据不要丢弃   要留着给下一次插值  

      // 如果下一帧和keyframe很接近  那么直接匹配
      if(fabs(gnss_data_2.time.toSec()-keyframe->stamp.toSec()) <= 0.005)
      {
        paired_gnss = gnss_data_2;  
        break;  
      }

      // 如果下一个的时间戳远小于关键帧 那么这次没法插值
      if(gnss_data_2.time.toSec() < keyframe->stamp.toSec())
      {
        gnss_data = gnss_data_2;
        GNSS_queue.pop_front();         // 将头数据弹出
        continue;  
      }

      // 如果两个GNSS数据差的太远了  也不能插值    匹配结束   
      if(gnss_data_2.time.toSec() - gnss_data.time.toSec() > 0.15)
      {
        return true;
      }

      double t1 = keyframe->stamp.toSec() - gnss_data.time.toSec();
      double t2 = gnss_data_2.time.toSec() - keyframe->stamp.toSec();

      /************************************************* 进行插值  插值出 关键帧对应 IMU的 经纬高, 姿态 ********************************************/
      // 计算插值系数     
      double front_scale = t2 / (t2+t1);
      double back_scale = t1 / (t2+t1);

      // 可以进行插值  
      // 如果没有初始化 
      if (!gnss_origin_position_inited) 
      { // 如果没有对齐GNSS与Lidar的坐标  那么对经纬高进行插值  准备坐标对齐 
        paired_gnss.longitude = gnss_data_2.longitude*back_scale + gnss_data.longitude*front_scale;
        paired_gnss.latitude = gnss_data_2.latitude*back_scale + gnss_data.latitude*front_scale;
        paired_gnss.altitude = gnss_data_2.altitude*back_scale + gnss_data.altitude*front_scale; 

        // 对IMU的姿态进行插值 设置给  keyframe->orientation 
        Eigen::Quaterniond imu_front,imu_back;
        imu_front.w() = gnss_data.imu->orientation.w;
        imu_front.x() = gnss_data.imu->orientation.x;
        imu_front.y() = gnss_data.imu->orientation.y;
        imu_front.z() = gnss_data.imu->orientation.z;
        imu_back.w() = gnss_data_2.imu->orientation.w;
        imu_back.x() = gnss_data_2.imu->orientation.x;
        imu_back.y() = gnss_data_2.imu->orientation.y;
        imu_back.z() = gnss_data_2.imu->orientation.z;

        // 球面插值  
        paired_gnss.IMU_orientation = imu_front.slerp(back_scale, imu_back);       
        break;  
      }
      else
      {
        paired_gnss.Lidar_Map_coords.x() = gnss_data_2.Lidar_Map_coords.x()*back_scale + gnss_data.Lidar_Map_coords.x()*front_scale;
        paired_gnss.Lidar_Map_coords.y() = gnss_data_2.Lidar_Map_coords.y()*back_scale + gnss_data.Lidar_Map_coords.y()*front_scale;
        paired_gnss.Lidar_Map_coords.z() = gnss_data_2.Lidar_Map_coords.z()*back_scale + gnss_data.Lidar_Map_coords.z()*front_scale;  

        // 对IMU的姿态进行插值 设置给  keyframe->orientation 
        Eigen::Quaterniond imu_front,imu_back;
        imu_front.w() = gnss_data.Lidar_Map_orientation.w();
        imu_front.x() = gnss_data.Lidar_Map_orientation.x();
        imu_front.y() = gnss_data.Lidar_Map_orientation.y();
        imu_front.z() = gnss_data.Lidar_Map_orientation.z();
        imu_back.w() = gnss_data_2.Lidar_Map_orientation.w();
        imu_back.x() = gnss_data_2.Lidar_Map_orientation.x();
        imu_back.y() = gnss_data_2.Lidar_Map_orientation.y();
        imu_back.z() = gnss_data_2.Lidar_Map_orientation.z();
        
        // 球面插值  
        paired_gnss.Lidar_Map_orientation = imu_front.slerp(back_scale, imu_back);       
        break;  
      }
    }

    if(GNSS_queue.empty())    // 如果为空  那么配对失败   但是 可以等待下一个GNSS数据  
    { 
      GNSS_queue.push_back(gnss_data);  
      return false;       
    }
    
  }

  // 有匹配数据 
  // 若没初始化 
  if (!gnss_origin_position_inited) 
  {
    /**************** 如果GNSS没有初始化  那么进行初始化   初始化主要是求 map->enu 的 Tem, 以及 Gnss_init_Map_loc   ***********************************/
    gnssInitialize(paired_gnss, paired_gnss.IMU_orientation, keyframe);   // 进行GNSS与MAP对齐
    transformWgs84ToMap(paired_gnss);  
    // 将之前的GNSS数据进行转换
    for(int n = 0; n<GNSS_queue.size(); n++)
    {
      transformWgs84ToMap(GNSS_queue.at(n));
    }
    
    std::cout<<"GNSS-Lidar initialize --------------------------------------!"<<std::endl;
  }
  
  // 获得Lidar的GNSS在Map系的坐标     
  keyframe->utm_coord = paired_gnss.Lidar_Map_coords;
  keyframe->orientation = paired_gnss.Lidar_Map_orientation;   
  keyframe->gnss_matched = true;  
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SlidingWindowBackEnd::updataSlidingWindows()
{
  if(wait_optimize_keyframes.empty()){
    return false;  
  }

  for(int i = 0; i<wait_optimize_keyframes.size(); i++){ 
    // 判断滑窗满了没   满了就要滑动  
    if(sliding_windows.size()>=sliding_windows_size){
      keyframes.emplace_back( std::move(sliding_windows.front()) );  
      sliding_windows.pop_front();
      sliding_windows.emplace_back(std::move(wait_optimize_keyframes[i]));   
    }
    else{
      sliding_windows.emplace_back(std::move(wait_optimize_keyframes[i]));   
    }
  }

  wait_optimize_keyframes.clear();

  return true; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::slidingWindowLocalOptimize()
{
  // 更新滑动窗口 
  if(!updataSlidingWindows())
    return;

  // Setup optimizer
  g2o::SparseOptimizer optimizer;                                   // 定义稀疏求解器  
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,-1>> Block;     // 重命名 块求解器     
  //Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();    // ubuntu 18下不行  
  std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
  //Block* solver_ptr = new Block(linearSolver);       // 定义 块求解器  
  std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));     // 构造优化方法  LM  
  optimizer.setAlgorithm(solver);            // 设置优化方法到 稀疏求解器

  // 记录本次局部优化中  节点与边的序号   
  int vertexCnt=1, edgeCnt=0;
  // 记录本次优化的全部节点  用于边寻找关联 节点    
  vector<g2o::VertexSE3*> vertexs;         
  
  // 遍历本次优化的全部关键帧  
  for(deque<KeyFrame::Ptr>::iterator keyframe_it = sliding_windows.begin(); keyframe_it != sliding_windows.end(); ++keyframe_it)
  {    
    // 添加节点  
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(vertexCnt++);
    // ROS_INFO_STREAM("setEstimate() : "<<keyframe->odom.matrix());
    v->setEstimate((*keyframe_it)->Pose);                                // 设置先验位姿
    optimizer.addVertex(v);     
    vertexs.push_back(v);
    
    if(enable_GNSS_optimize)
    {   
        // 判断是否执行GNSS优化
        // 添加先验的边    每一条GNSS的边  残差维度 3   单边   只与一个节点状态有关  Jt(6*3)*W(3*3)*J(3*6)  
        if((*keyframe_it)->GNSS_Valid)
        {
          g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
          edge->setMeasurement((*keyframe_it)->utm_coord);      // 设置观测先验  
          edge->vertices()[0] = v;
          // 信息矩阵     3*3      JtWJ  
          Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
          information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy * gps_edge_stddev_xy;     // 1
          information_matrix(2, 2) /= gps_edge_stddev_z * gps_edge_stddev_z;                  // 2  
          edge->setInformation(information_matrix);
          optimizer.addEdge(edge);
        //  optimizer.add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));
        }
    }

    // 可以添加全局平面约束   
    if(enable_planeConstraint_optimize)
    {   
      if((*keyframe_it)->planeConstraint_Valid){
        
        /*
        g2o::EdgeSE3PlanePrior* plane_prior(new g2o::EdgeSE3PlanePrior());
        plane_prior->setVertex(0, v); 
        std::cout<<"origin_pose--------------------------"<<v->estimate().matrix()<<std::endl;
        // 设置先验
        Eigen::Matrix3d Rwb = v->estimate().linear();     // 旋转
        Eigen::Vector3d twb = v->estimate().translation();   // XYZ   
        Eigen::Vector3d euler_angles = Rwb.eulerAngles(2,1,0);   // 转欧拉角    y, p , r
        std::cout<<"Rwb--------------------------"<<Rwb<<std::endl;
        std::cout<<"twb--------------------------"<<twb<<std::endl;
        euler_angles[1] = 0;
        euler_angles[2] = 0;    
        twb[2] = 0;
        // 欧拉角转旋转矩阵
        Eigen::AngleAxisd rollAngle(euler_angles(2),Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(euler_angles(1),Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(euler_angles(0),Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd prior_rotation_vector (yawAngle*pitchAngle*rollAngle);
        //Eigen::AngleAxisd prior_rotation_vector(Rwb) ;

        Eigen::Isometry3d prior_pose = Eigen::Isometry3d::Identity();
        prior_pose.rotate(prior_rotation_vector); 
        prior_pose.pretranslate(twb); 
        std::cout<<"prior_pose--------------------------"<<prior_pose.matrix()<<std::endl;
        plane_prior->setMeasurement(prior_pose); 

        // 设置信息矩阵 
        // 信息矩阵     6*6       JtWJ     与残差维度相等
        Matrix6d information_matrix = Matrix6d::Identity();
        information_matrix.block<3, 3>(0, 0) /= 5;     // 旋转相关  约束不大 
        information_matrix.block<3,3>(3,3) /= 1;                  // 
        plane_prior->setInformation(information_matrix);
        
        optimizer.addEdge(plane_prior);
        */
      // 高度约束以及姿态约束
      // 高度约束 
      g2o::EdgeSE3PriorXYZ* prior_height_edge(new g2o::EdgeSE3PriorXYZ());
      Eigen::Vector3d twb = v->estimate().translation();   // XYZ 
      std::cout<<" origin twb ----------------------------"<<std::endl<<twb.transpose()<<std::endl;
      twb[2] = 0;  
      prior_height_edge->setMeasurement(twb);              // 设置观测先验  
      prior_height_edge->vertices()[0] = v;
      // 信息矩阵     3*3      JtWJ  
      Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
      information_matrix.block<2, 2>(0, 0) *= 0;     // x,y直接设为0  
      information_matrix(2, 2) /= 10;                 // 5  
      prior_height_edge->setInformation(information_matrix);
      optimizer.addEdge(prior_height_edge);
      
      /*
      // 姿态约束
      g2o::EdgeSO3Prior* prior_rot_edge(new g2o::EdgeSO3Prior());
      prior_rot_edge->vertices()[0] = v;
      Eigen::Matrix3d Rwb = v->estimate().linear();   // 旋转
      
      // 转换到欧拉角
      Eigen::Vector3d euler_angles = Rwb.eulerAngles(2,1,0);   // 转欧拉角    y, p, r 
      euler_angles[1] = 0;
      euler_angles[2] = 0;
      // 欧拉角转旋转矩阵
      Eigen::Matrix3d prior_rotation_matrix;
      prior_rotation_matrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ()) * 
                              Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) * 
                              Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());
      prior_rot_edge->setMeasurement(prior_rotation_matrix);
      std::cout<<"prior_pose_RPY--------------------------"<<std::endl<<prior_rotation_matrix.eulerAngles(2,1,0)<<std::endl;
      // 信息矩阵     3*3      JtWJ  
      Eigen::Matrix3d prior_rot_information_matrix = Eigen::Matrix3d::Identity();
      prior_rot_information_matrix.block<3, 3>(0, 0) /= 1;                                
      prior_rot_edge->setInformation(prior_rot_information_matrix);
      optimizer.addEdge(prior_rot_edge);
      */
      }
    }      

    // 处理本次局部优化的第一个节点  
    if(vertexs.size()==1)
    { 
      // 历史上第一个节点  
      if(keyframes.empty())
      {
        v->setFixed(true);            // 要进行fix处理  
      }
      else
      {    
        // 本次优化的第一个节点  
        // 创建一个固定的原点节点   当前 keyframes 最后一个节点   
        g2o::VertexSE3* org = new g2o::VertexSE3();
        org->setId(0);    
        org->setEstimate(keyframes.back()->Pose);
        org->setFixed(true);
        optimizer.addVertex(org);
        // 激光里程计的边  
        g2o::EdgeSE3* edge(new g2o::EdgeSE3());
        edge->setId(edgeCnt++);
        edge->setVertex(0, org);
        edge->setVertex(1, v);
        // Tlw * Twc = Tlc   
        edge->setMeasurement((*keyframe_it)->deltaOdom);
        // 计算信息矩阵    
        // 通过kdtree检查点云通过变换后的匹配程度反映里程计是否准确   匹配程度越高  则信息矩阵各权重越大   则优化时  会更靠近里程计的结果   
        // Eigen::MatrixXd information = inf_calclator->calc_information_matrix( keyframes.back()->cloud, (*keyframe_it)->cloud, relative_pose);
        Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) / 0.01; 
        edge->setInformation(information);
        optimizer.addEdge(edge);
     }

     continue;
    }

    // 激光里程计的边  
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setId(edgeCnt++);
    edge->setVertex(0, vertexs[vertexCnt-3]);
    edge->setVertex(1, vertexs[vertexCnt-2]);
    // add edge between consecutive keyframes
    // 计算相对位移        Tlw * Twc = Tlc 
    // 里程计约束  
    edge->setMeasurement((*keyframe_it)->deltaOdom);
    // 信息矩阵  里程计模块需要给出一个值   
    // prev_keyframe->cloud = Tlc * keyframe->cloud
    // Eigen::MatrixXd information = inf_calclator->calc_information_matrix((sliding_windows.at(vertexCnt-2))->cloud, (*keyframe_it)->cloud,  relative_pose);
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) / 0.005; 
    edge->setInformation(information);
    optimizer.addEdge(edge);
  }

  // optimize the pose graph
  // 执行优化
  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << optimizer.vertices().size() << "   edges: " << optimizer.edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  std::cout << "init" << std::endl;
  optimizer.initializeOptimization(0);
  optimizer.setVerbose(true);

  std::cout << "chi2" << std::endl;
  double chi2 = optimizer.chi2();

  std::cout << "optimize!!" << std::endl;
  auto t1 = ros::WallTime::now();
  optimizer.optimize(30);

  auto t2 = ros::WallTime::now();
  std::cout << "done" << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer.chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

  int i = 0;
  for(deque<KeyFrame::Ptr>::iterator keyframe_it = sliding_windows.begin(); keyframe_it != sliding_windows.end(); ++keyframe_it)
  {
    (*keyframe_it)->Pose = vertexs[i]->estimate();    //  获取优化结果
    i++;  
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::globalOptimize(const Loop::Ptr& loop)
{
  // Setup optimizer
  g2o::SparseOptimizer optimizer;   // 定义稀疏求解器  
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,-1>> Block;     // 重命名 块求解器     
  //Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();    // ubuntu 18下不行  
  std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
  //Block* solver_ptr = new Block(linearSolver);       // 定义 块求解器  
  std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));     // 构造优化方法  LM  
  optimizer.setAlgorithm(solver);            // 设置优化方法到 稀疏求解器

  // 本次回环连接的关键帧id  
  int id_min = loop->key2->id;
  vector<Loop::Ptr> add_loop; 
  map<int, g2o::VertexSE3*> loopInfo;        // 记录回环的每个keyfram 的 id与生成的节点  
  add_loop.push_back(loop);
  loopInfo[loop->key1->id] = NULL;
  loopInfo[loop->key2->id] = NULL;
  // 检查是否需要添加之前的回环约束     更新回环的最早节点id   
  for(int i = Loops.size()-1; i>=0; i--)
  {  // 该回环需要添加  
     if(Loops[i]->key1->id >= id_min)
     {
       add_loop.push_back(Loops[i]);
       if(Loops[i]->key2->id < id_min)
         id_min = Loops[i]->key2->id;
       // 记录该回环的两个帧的id
       loopInfo[Loops[i]->key1->id] = NULL;
       loopInfo[Loops[i]->key2->id] = NULL;
     }  
     else
       break;
  }
  int loop_size = add_loop.size();
  ROS_INFO_STREAM("********** global optimation!  loops num: "<<loop_size);

  int vertexCnt=0, edgeCnt=0, loop_vertex_num=0, frist_kf_index=0;
  vector<g2o::VertexSE3*> vertexs;    // 保存当前优化器中的节点  
  
  KeyFrame::Ptr pre_keyframe = NULL; 

  for(const auto& keyframe:keyframes)
  {  
      // 只将闭环内的节点添加  
      if(keyframe->id<id_min)  
      {
        frist_kf_index++;
        continue;
      }
      // 添加节点  
      g2o::VertexSE3* v = new g2o::VertexSE3();
      v->setId(vertexCnt++);
      v->setEstimate(keyframe->Pose);
      if(vertexCnt==1)
        v->setFixed(true);
      optimizer.addVertex(v);
      vertexs.push_back(v);
      // 查找该节点是否属于回环  
      if(loop_vertex_num<loop_size*2&&loopInfo.find(keyframe->id)!=loopInfo.end())
      {
          loopInfo[keyframe->id] = v;      // 覆盖掉原结果  
          loop_vertex_num++;
      }
      if(vertexCnt==1) {
        pre_keyframe = keyframe;
        continue;
      }
      
      // 添加边  
      g2o::EdgeSE3* edge(new g2o::EdgeSE3());
      edge->setId(edgeCnt++);
      edge->setVertex(0, optimizer.vertices()[vertexCnt-2]);            // 老一帧  
      edge->setVertex(1, optimizer.vertices()[vertexCnt-1]);
      
      // 计算两节点间的相对位移         T t * delta = Tt-1
      Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
      relative_pose = pre_keyframe->Pose.matrix().inverse()*keyframe->Pose.matrix();
      //  ROS_INFO_STREAM("edge : "<<relative_pose.matrix());
      edge->setMeasurement(relative_pose);
      // 计算信息矩阵
      //Eigen::MatrixXd information = inf_calclator->calc_information_matrix(pre_keyframe->cloud, keyframe->cloud,  relative_pose);
      // 固定信息矩阵  
      Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
      inf.topLeftCorner(3, 3).array() /= 20;
      inf.bottomRightCorner(3, 3).array() /= 5;
      edge->setInformation(inf);
      optimizer.addEdge(edge);
      /*
      // 添加该keyframe 的GNSS先验
      if(keyframe->GNSS_Valid)
      { 
        g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
        edge->setMeasurement(keyframe->utm_coord);      // 设置观测先验  
        edge->vertices()[0] = v;
        // 信息矩阵  
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;     // 20
        information_matrix(2, 2) /= gps_edge_stddev_z;                  // 5  
        edge->setInformation(information_matrix);
        optimizer.addEdge(edge);
      //  optimizer.add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));
      } */
      // 添加地面约束

      // 添加imu Pose 约束  
      
      pre_keyframe = keyframe;
  }

  for(auto& l:add_loop) 
  {
    // 添加回环边
    if(!loopInfo[l->key1->id]||!loopInfo[l->key2->id])  continue;   
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setId(edgeCnt++);
    edge->setVertex(0, loopInfo[l->key2->id]);     // 相对早一点的
    edge->setVertex(1, loopInfo[l->key1->id]);     // 相对晚一点的
    Eigen::Isometry3d relpose(l->relative_pose.cast<double>());    // relpose :  curr -> last  
    edge->setMeasurement(relpose);
    // 计算信息矩阵    维度与优化状态的维度相当   
    Eigen::MatrixXd information = inf_calclator->calc_information_matrix(l->key1->cloud, l->key2->cloud, relpose);
    ROS_INFO_STREAM("LOOP inf: "<<information.matrix());
    edge->setInformation(information);
    optimizer.addEdge(edge);
  }

  // optimize the pose graph
  // 执行优化
  std::cout << std::endl;
  std::cout << "--- global optimization ---" << std::endl;
  std::cout << "nodes: " << optimizer.vertices().size() << "   edges: " << optimizer.edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  std::cout << "init" << std::endl;
  optimizer.initializeOptimization(0);
  optimizer.setVerbose(true);

  std::cout << "chi2" << std::endl;
  double chi2 = optimizer.chi2();

  std::cout << "optimize!!" << std::endl;
  auto t1 = ros::WallTime::now();
  int iterations = optimizer.optimize(30);

  auto t2 = ros::WallTime::now();
  std::cout << "done" << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer.chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
  
  for(int i=0; i<vertexs.size(); i++)
  {
     keyframes[frist_kf_index+i]->Pose = vertexs[i]->estimate();    //  获取优化结果
  }
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::optimization() 
{
  std::cout<<"optimization ------------------------ begin "<<std::endl;

  if(wait_optimize_keyframes.empty())
  {
    return;  
  }

  std::cout<<"slidingWindowLocalOptimize ----------------------- begin "<<std::endl;
  // 执行局部优化
  slidingWindowLocalOptimize();
  std::cout<<"slidingWindowLocalOptimize ----------------------- down "<<std::endl;
  // 计算优化矩阵
  trans_odom2map = sliding_windows.back()->Pose * sliding_windows.back()->odom.inverse();  

  // 如果有订阅者  发布odom到map坐标系的变换  
  if(odom2map_pub.getNumSubscribers()) 
  {
    ROS_INFO_STREAM("BackEnd_node - trans_odom2map: "<<std::endl<<trans_odom2map.matrix());   
    // 构造 ROS Msg
    geometry_msgs::TransformStamped ts = matrix2transform(ros::Time::now(), trans_odom2map.matrix().cast<float>(), map_frame_id, odom_frame_id);
    odom2map_pub.publish(ts);
  }

  if(SaveBackEndPath)
  {
    // 保存局部优化的结果 
    Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();  
    for(auto const& frame : wait_optimize_keyframes)
    { 
      if(frame->GNSS_Valid)
      { 
        gt_pose.block<3,1>(0, 3) = frame->utm_coord; 
        gt_pose.block<3,3>(0, 0) =  frame->orientation.matrix(); 
        SaveTrajectory(gt_pose, frame->Pose.matrix(), "/slam_data/trajectory", "/slam_data/trajectory/backend_ground_truth.txt", 
                      "/slam_data/trajectory/backend_est_path.txt");
      }
    }
  }

  // // 如果存在回环 
  // if(loop)
  // {
  //   // 执行全局优化
  //   globalOptimize(loop);
  //   Loops.push_back(loop);
  // }
  
  // // KeyFrameSnapshot 用于建图的关键帧节点  只有位姿与点云信息  
  // std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
  // std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(),
  //   [=](const KeyFrame::Ptr& k) {     
  //     return std::make_shared<KeyFrameSnapshot>(k);     // 用 KeyFrame 指针 k 构造  KeyFrameSnapshot 
  // });
  // keyframes_snapshot_mutex.lock();
  // keyframes_snapshot.swap(snapshot);
  // keyframes_snapshot_mutex.unlock();
  std::cout<<"optimization ------------------------ down "<<std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::mapPointsPublish() 
{  
  if(!map_points_pub.getNumSubscribers()) {
    return;
  }

  std::vector<KeyFrameSnapshot::Ptr> snapshot;

  keyframes_snapshot_mutex.lock();
  snapshot = keyframes_snapshot;
  keyframes_snapshot_mutex.unlock();

  auto cloud = map_cloud_generator->generate(snapshot, 0.05);
  if(!cloud) {
    return;
  }

  cloud->header.frame_id = map_frame_id;
  cloud->header.stamp = snapshot.back()->cloud->header.stamp;

  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud, *cloud_msg);
  
  map_points_pub.publish(cloud_msg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::BackEndProcess()
{
   while(true)
   {
      if(Optimize_previous_time.toSec() == 0 && Map_updata_previous_time.toSec() == 0)
      {
        Optimize_previous_time = ros::Time::now();
        Map_updata_previous_time = ros::Time::now();
      }
      else
      {
        double Map_updata_diff_time = (ros::Time::now() - Map_updata_previous_time).toSec();     // 计算时间差
        // odom线程锁开启 
        {
          std::lock_guard<std::mutex> lock(keyframe_queue_mutex); 
          // 数据处理 
          processDataInSingle();
        }
        // 进行优化 
        optimization();
          
        // 地图更新
        if(Map_updata_diff_time>= Map_updata_duration)
        {
          Map_updata_previous_time = ros::Time::now();
          ROS_INFO_STREAM("updata Map!");
          mapPointsPublish();
        }   
        
        // 500ms的延时  
        std::chrono::milliseconds dura(500);
        std::this_thread::sleep_for(dura);
      }
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::LoopDetectProcess()
{
  while(1)
  {
     
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::commInit()
{
    // publishers
    markers_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);        // 可视化
    odom2map_pub = private_nh.advertise<geometry_msgs::TransformStamped>("/odom2pub", 16);      // odom到map的校正  
    map_points_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1);          // 发布地图  可视化   
    pubGNSSPath = private_nh.advertise<nav_msgs::Path>("/GNSSpath", 100);
    // subscrible      
    // IMU的订阅                                                                                             // 点云的回调函数  
    //imu_sub = nh.subscribe("/kitti/oxts/imu", 1024, imu_callback);
    // 地面检测的订阅
    //floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024,  floor_coeffs_callback);
    
    //  navsat_sub = nh.subscribe("/kitti/oxts/gps/fix", 1024, navsat_callback);
    // subscribers
    
    // 点云和里程计数据的订阅  并且进行同步处理  
    odom_topic = private_nh.param<std::string>("odom_topic", "/");
    points_topic = private_nh.param<std::string>("points_topic", "/");
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(private_nh, odom_topic, 1000));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(private_nh, points_topic, 1000));
    Sync_odomCloud.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(*odom_sub, *cloud_sub, 1000));          
    // Sync_odomCloud->registerCallback(boost::bind(&SlidingWindowBackEnd::CloudCallback, _1, _2)); 
    Sync_odomCloud->registerCallback(&SlidingWindowBackEnd::CloudCallback, this); 

    // 同步订阅 GNSS的数据   包括IMU与GPS
    // GPS订阅
    string ins_imu_topic;
    nh.param<std::string>("liv_slam/insImuTopic", ins_imu_topic, "/imu_raw");  
    string gps_topic;
    nh.param<std::string>("liv_slam/gpsTopic", gps_topic, "/gps/fix");  
    std::cout<<" Backend IMU topic: "<< ins_imu_topic << " gnss topic: "<< gps_topic << std::endl;
    // imu_sub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/kitti/oxts/imu", 1000));
    // navsat_sub.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, "/kitti/oxts/gps/fix", 1000));
    imu_sub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, ins_imu_topic, 1000));
    navsat_sub.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, gps_topic, 1000));
    Sync_GPSIMU.reset(new message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::NavSatFix>(*imu_sub, *navsat_sub, 1000));          
    Sync_GPSIMU->registerCallback(&SlidingWindowBackEnd::GnssCallback, this);   

    // 服务
    //dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    //save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SlidingWindowBackEnd::paramsInit()
{
    
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    // graph_slam.reset(new GraphSLAM(nh.param<std::string>("g2o_solver_type", "lm_var")));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));

    // 按 graph_update_interval 时间间隔定时执行optimization_timer_callback
    //  optimization_timer = nh.createWallTimer(ros::WallDuration(graph_update_interval), optimization_timer_callback);
    //  map_publish_timer = nh.createWallTimer(ros::WallDuration(10), map_points_publish_timer_callback);
    odometry_edge_robust_kernel = private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE");
    odometry_edge_robust_kernel_size = 1.0;
    loop_closure_edge_robust_kernel_size = 1.0;
    fix_first_node_adaptive = true;
    num_iterations = 50;                  // 迭代次数
    fix_first_node_stddev = "10 10 10 1 1 1";
    max_keyframes_per_update = 10;
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    trans_odom2map.setIdentity();              // 里程计到map坐标的转换  
    gps_time_offset = 0;
    imu_orientation_edge_stddev = 1;           // IMU的信息矩阵的权重  
    imu_orientation_edge_robust_kernel = private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE");
    imu_orientation_edge_robust_kernel_size = private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0);
    enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
    imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);

    Til <<  0.999998 ,   -0.000785403 ,  0.00202441  ,   0.810544,
            0.000755307 ,     0.99989 ,   0.0148245,    -0.307054,
            -0.00203583 ,   -0.014823  ,   0.999888  ,   0.802724,
            0      ,      0       ,     0   ,         1;

    gps_edge_stddev_xy = 8;
    gps_edge_stddev_z = 8;
    // GNSS优化
    enable_GNSS_optimize = private_nh.param<bool>("enable_GNSS_optimize", false);
    // GNSS优化频率
    GNSS_optimize_freq = nh.param<int>("liv_slam/GNSS_optimize_freq", 5);
    cout<<"GNSS_optimize_freq: " << GNSS_optimize_freq << endl;
    // 平面优化
    enable_planeConstraint_optimize = private_nh.param<bool>("enable_planeConstraint_optimize", false);
    // 平面优化频率
    planeConstraint_optimize_freq = nh.param<int>("liv_slam/planeConstraint_optimize_freq", 5);

    // 关键帧点云保存的空间
    key_frames_path = "/home/gogo/lwh/lwh_ws/src/liv_slam-master/Map";  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init (argc, argv, "BackEnd_node");   
    ROS_INFO("Started BackEnd_node node");    
    SlidingWindowBackEnd back_end;
    // 观测处理线程  - 后端优化主线程  
    std::thread measurement{&SlidingWindowBackEnd::BackEndProcess, &back_end}; 
    // 回环检测线程 
    std::thread loopDetect{&SlidingWindowBackEnd::LoopDetectProcess, &back_end}; 
    ros::spin(); 
    return 0;
}














