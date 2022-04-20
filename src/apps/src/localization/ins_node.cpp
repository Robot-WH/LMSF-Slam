
#include "ros_utils.hpp"
#include "Estimator/states.hpp"
#include "Estimator/estimator/ins_filter_estimator.hpp" 
#include "boost/scoped_ptr.hpp"
#include "Estimator/Correction/GNSS/position_correction.hpp"

using namespace Slam3D; 

ros::Subscriber subImu;                             // IMU
ros::Subscriber subGnss;                           // gnss    提供真值用于比较  
ros::Publisher pubGnssPath;                    // 发布GNSS轨迹
ros::Publisher pubFusionPath;                // 发布GNSS轨迹
ros::Publisher pubImuPredictPath;                 // 发布GNSS轨迹
nav_msgs::Path GnssPath;                           // 记录gnss轨迹  
nav_msgs::Path FusionPath;                       // 记录融合后轨迹  
nav_msgs::Path ImuPredictPath;              // IMU预测的轨迹  
// 维护IMU系相对于world系的姿态    
Eigen::Matrix3d Rwi = Eigen::Matrix3d::Identity();
Eigen::Vector3d twi = {0, 0, 0};
// 通过滤波器估计前后两帧间的运动 
std::unique_ptr<InsEstimator<StatesWithImu, 15>> ins_estimator;  
// 数据缓存队列
queue<ImuDataPtr> imu_buf;  
queue<GnssDataPtr> gnss_buf;
// 线程同步相关
std::mutex m_buf;
// 坐标系
std::string map_frame_id="map";
// IMU
float imuAccNoise = 0;
float imuGyroNoise = 0;
float imuAccBiasN = 0;
float imuGyroBiasN = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void allocateMemory() {
    ins_estimator.reset( new InsEstimator<StatesWithImu, 15>(
                                                imuAccNoise, imuGyroNoise, imuAccBiasN, imuGyroBiasN, 
                                                XYZCorrectorPtr<StatesWithImu, 15>(
                                                    new PositionCorrection<StatesWithImu, 16, 15>()
                                                )));  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imuHandler( sensor_msgs::ImuConstPtr const& imu_msg) 
{
    // 保证队列中 数据的顺序正确 
    static double last_imu_t = -1; 
    if (imu_msg->header.stamp.toSec() <= last_imu_t) {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    // 解析IMU数据 
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    // 保存时间戳 
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg->linear_acceleration.x, 
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg->angular_velocity.x,
                        imu_msg->angular_velocity.y,
                        imu_msg->angular_velocity.z;
    imu_data_ptr->rot.w() = imu_msg->orientation.w;
    imu_data_ptr->rot.x() = imu_msg->orientation.x;
    imu_data_ptr->rot.y() = imu_msg->orientation.y;
    imu_data_ptr->rot.z() = imu_msg->orientation.z;

    m_buf.lock();
    imu_buf.push(imu_data_ptr);
    m_buf.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gnssHandler(sensor_msgs::NavSatFixConstPtr const& navsat_msg) 
{
    // 保证队列中 数据的顺序正确 
    static double last_gnss_t = -1; 
    if (navsat_msg->header.stamp.toSec() <= last_gnss_t) {
        ROS_WARN("gnss message in disorder!");
        return;
    }
    last_gnss_t = navsat_msg->header.stamp.toSec();
    // 解析Gnss数据 
    GnssDataPtr gnss_data_ptr = std::make_shared<GnssData>();
    // 保存时间戳 
    gnss_data_ptr->timestamp = navsat_msg->header.stamp.toSec();
    gnss_data_ptr->lla << navsat_msg->latitude,
                        navsat_msg->longitude,
                        navsat_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(navsat_msg->position_covariance.data());

    m_buf.lock();
    gnss_buf.push(gnss_data_ptr);
    m_buf.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Process() 
{
    while (true) 
    {   
        // 如果有数据容器非空  那么进行处理 
        if(!imu_buf.empty()||!gnss_buf.empty()) 
        {
            uint8_t sensor_id = 0;
            double earliest_time = numeric_limits<double>::max();  
            // 找到最早的数据  
            if(!imu_buf.empty()) 
            {   
                earliest_time = imu_buf.front()->timestamp;  
                sensor_id = imu;
            }
            if(!gnss_buf.empty()) 
            {   
                // 如果gnss更早 
                if(earliest_time > gnss_buf.front()->timestamp) {
                    earliest_time = gnss_buf.front()->timestamp;
                    sensor_id = gnss;
                }
            }
            switch(sensor_id) 
            {
                // 如果是imu数据执行 预测  
                case imu: 
                {
                    // 取出头数据 
                    ImuDataConstPtr imu_ptr = imu_buf.front();  
                    imu_buf.pop();  
                    ins_estimator->Process(imu_ptr);      // 将数据传入到估计器中进行
                    // 显示当前IMU的预测轨迹
                    if(ins_estimator->IsInitialized()) 
                    {   
                        CommonStates const& common_states = ins_estimator->GetState().common_states_;     // 获取估计后的通用状态 
                        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
                        geometry_msgs::PoseStamped imu_predict_pose; 
                        imu_predict_pose.header.stamp = ros::Time{imu_ptr->timestamp};
                        imu_predict_pose.pose.position.x = common_states.P_.x();
                        imu_predict_pose.pose.position.y = common_states.P_.y();
                        imu_predict_pose.pose.position.z = common_states.P_.z();
                        imu_predict_pose.pose.orientation.w = rot.w();
                        imu_predict_pose.pose.orientation.x = rot.x();
                        imu_predict_pose.pose.orientation.y = rot.y();
                        imu_predict_pose.pose.orientation.z = rot.z();
                        ImuPredictPath.header.stamp = imu_predict_pose.header.stamp;
                        ImuPredictPath.poses.push_back(imu_predict_pose);
                        ImuPredictPath.header.frame_id = map_frame_id; // odom坐标
                        pubImuPredictPath.publish(ImuPredictPath);
                        std::cout<<"pubImuPredictPath! "<<" num: "<< ImuPredictPath.poses.size() <<std::endl;
                    }
                }
                break;
                case gnss: 
                {   
                    // 至少要保证容器中有2个以上数据才进行处理 
                    if(gnss_buf.size()<2) {
                        break;
                    }
                    // 取出头数据 
                    GnssDataConstPtr gnss_ptr = gnss_buf.front();  
                    gnss_buf.pop();  
                    // 如果初始化了  那么开始校正之前 检查是否需要增加预测 
                    if(ins_estimator->IsInitialized()) 
                    {
                        // 由于校正时，上一次IMU的预测状态的时间戳可能与当前校正的时间戳有较大的偏差
                        // 如果存在IMU数据    那么在校正之前需要插值出GNSS时间戳处的IMU数据  然后再次进行预测 
                        if(!imu_buf.empty())
                         {
                            // 插值 
                            ImuDataConstPtr curr_imu = imu_buf.front();  
                            imu_buf.pop();   
                            ImuDataConstPtr const& last_imu = ins_estimator->GetLastIMU();     // 获取上一个IMU数据 
                            ImuDataConstPtr interpolation_imu = ImuDataLinearInterpolation(last_imu, curr_imu, 
                                                                                                        gnss_ptr->timestamp); // 插值出 GNSS 位置处的 IMU数据                                                                       
                            // 用插值后的数剧进行预测
                            ins_estimator->Process(interpolation_imu);  
                        } else {    // 如果之后没有IMU数据 即IMU挂了
                                        // 如果IMU没有    那么可以采用匀速运动学模型进行预测   
                                        // 首先判断上一个IMU的时间间隔是否足够大，若足够大那么需要通过运动学模型
                                        // 进行预测，若比较小，则不需要预测 
                        }
                    }
                    // 校正 
                    ins_estimator->Process(gnss_ptr);  
                    // 保存GNSS的轨迹 
                    if(ins_estimator->IsInitialized()) 
                    {   
                        // 获取更新后的GNSS轨迹 
                        GnssDataProcess& gnss_processor = GnssDataProcess::GetInstance();  
                        Eigen::Vector3d xyz = gnss_processor.GetEnuPosition();
                        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
                        // 发布odom
                        geometry_msgs::PoseStamped Pose; 
                        Pose.header.stamp = ros::Time{gnss_ptr->timestamp};
                        Pose.pose.position.x = xyz.x();
                        Pose.pose.position.y = xyz.y();
                        Pose.pose.position.z = xyz.z();
                        Pose.pose.orientation.w = rot.w();
                        Pose.pose.orientation.x = rot.x();
                        Pose.pose.orientation.y = rot.y();
                        Pose.pose.orientation.z = rot.z();
                        GnssPath.header.stamp = Pose.header.stamp;
                        GnssPath.poses.push_back(Pose);
                        GnssPath.header.frame_id = map_frame_id; // odom坐标
                        pubGnssPath.publish(GnssPath);
                        std::cout<<"pubGnssPath! "<<" num: "<< GnssPath.poses.size() 
                        << " xyz: "<< xyz.transpose() <<std::endl;
                        // 发布滤波估计后的轨迹 
                        CommonStates const& common_states = ins_estimator->GetState().common_states_;    // 获取估计后的通用状态 
                        Pose.pose.position.x = common_states.P_.x();
                        Pose.pose.position.y = common_states.P_.y();
                        Pose.pose.position.z = common_states.P_.z();
                        FusionPath.header.stamp = Pose.header.stamp;
                        FusionPath.poses.push_back(Pose);
                        FusionPath.header.frame_id = map_frame_id; // odom坐标
                        pubFusionPath.publish(FusionPath);
                        // IMU预测的轨迹清空
                        ImuPredictPath.poses.clear();
                    }
                }
                break;  
            }
        }
        // 1ms的延时  
        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init(ros::NodeHandle &private_nh, ros::NodeHandle &nh) {   
    private_nh = ros::NodeHandle("~");
    string imu_topic, gps_topic;
    imu_topic = private_nh.param<std::string>("ImuTopic", "/");
    gps_topic = private_nh.param<std::string>("GnssTopic", "/");
    ROS_WARN("imu_topic:");
    std::cout<<imu_topic<<std::endl;
    ROS_WARN("gps_topic:");
    std::cout<<gps_topic<<std::endl;
    subImu = private_nh.subscribe(imu_topic, 1000, &imuHandler,
                                                    ros::TransportHints().tcpNoDelay());
    subGnss = private_nh.subscribe(gps_topic, 1000, &gnssHandler, 
                                                    ros::TransportHints().tcpNoDelay());  
    pubGnssPath = private_nh.advertise<nav_msgs::Path>("/Gnsspath", 100);
    pubFusionPath = private_nh.advertise<nav_msgs::Path>("/Fusionpath", 100);
    pubImuPredictPath = private_nh.advertise<nav_msgs::Path>("/ImuPredictpath", 100);
    nh.param<float>("liv_slam/imuAccNoise", imuAccNoise, 0.01);
    nh.param<float>("liv_slam/imuGyroNoise", imuGyroNoise, 0.001);
    nh.param<float>("liv_slam/imuAccBiasN", imuAccBiasN, 0.0002);
    nh.param<float>("liv_slam/imuGyroBiasN", imuGyroBiasN, 0.01);

    std::cout<<"imuAccNoise: "<<imuAccNoise<<std::endl;
    std::cout<<"imuGyroNoise: "<<imuGyroNoise<<std::endl;
    std::cout<<"imuAccBiasN: "<<imuAccBiasN<<std::endl;
    std::cout<<"imuGyroBiasN: "<<imuGyroBiasN<<std::endl;
    allocateMemory();  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "INS_node");
    ROS_INFO("Started INS node");  
    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    init(private_nh, nh);  
    // 融合线程  
    std::thread measurement_process{&Process};
    ros::spin();
    return 0;
}