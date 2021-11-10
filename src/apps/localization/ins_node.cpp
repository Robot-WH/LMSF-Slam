
    
    #include "ros_bridge/LidarImuGnssFilterFusionOdometry_bridge.h"

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    LidarImuGnssFilterFusionOdometryBridge::LidarImuGnssFilterFusionOdometryBridge(
        std::unique_ptr<LidarImuGnssFilterEstimatorInterFace> &estimator_ptr) : estimator_ptr_(std::move(estimator_ptr))
    {   
        private_nh_ = ros::NodeHandle("~");
        string lidar_topic,imu_topic, gps_topic;
        lidar_topic = private_nh_.param<std::string>("LidarTopic", "/Lidar");
        imu_topic = private_nh_.param<std::string>("ImuTopic", "/");
        gps_topic = private_nh_.param<std::string>("GnssTopic", "/");

        subLidar = private_nh_.subscribe(lidar_topic, 1000, 
                                                            &LidarImuGnssFilterFusionOdometryBridge::lidarPointCloudHandler, 
                                                            this, ros::TransportHints().tcpNoDelay());

        subImu = private_nh_.subscribe(imu_topic, 1000, 
                                                        &LidarImuGnssFilterFusionOdometryBridge::imuHandler, this, 
                                                        ros::TransportHints().tcpNoDelay());

        subGnss = private_nh_.subscribe(gps_topic, 1000, 
                                                        &LidarImuGnssFilterFusionOdometryBridge::gnssHandler, 
                                                        this, ros::TransportHints().tcpNoDelay());  

        pubGnssPath = private_nh_.advertise<nav_msgs::Path>("/Gnsspath", 100);
        pubFusionPath = private_nh_.advertise<nav_msgs::Path>("/Fusionpath", 100);
        pubImuPredictPath = private_nh_.advertise<nav_msgs::Path>("/ImuPredictpath", 100);
        
        cout << " lidar odometry topic: " << lidar_topic << " ,imu topic: " 
                << imu_topic << " ,gnss topic: " << gps_topic << std::endl;     
    
        // 读取 是否需要使用IMU    使用IMU即为
        private_nh_.param<bool>("IMUFusionSwitch", IMU_fusion_switch, true);
        private_nh_.param<bool>("IMUPathSwitch", IMU_path_switch, true);

        // 读取 是否需要使用gnss 
        private_nh_.param<bool>("GnssFusionSwitch", gnss_fusion_switch, false);
        private_nh_.param<bool>("GnssPathSwitch", gnss_path_switch, false);

        allocateMemory();  
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LidarImuGnssFilterFusionOdometryBridge::allocateMemory()
    {  
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LidarImuGnssFilterFusionOdometryBridge::predict(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        // double t = imu_msg->header.stamp.toSec();
        // if (init_imu)
        // {
        //     latest_time = t;
        //     init_imu = 0;
        //     return;
        // }
        // double dt = t - latest_time;
        // latest_time = t;

        // double dx = imu_msg->linear_acceleration.x;
        // double dy = imu_msg->linear_acceleration.y;
        // double dz = imu_msg->linear_acceleration.z;
        // Eigen::Vector3d linear_acceleration{dx, dy, dz};

        // double rx = imu_msg->angular_velocity.x;
        // double ry = imu_msg->angular_velocity.y;
        // double rz = imu_msg->angular_velocity.z;
        // Eigen::Vector3d angular_velocity{rx, ry, rz};

        // Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

        // Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
        // tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

        // Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

        // Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        // tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
        // tmp_V = tmp_V + dt * un_acc;

        // acc_0 = linear_acceleration;
        // gyr_0 = angular_velocity;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LidarImuGnssFilterFusionOdometryBridge::update()
    {
        // TicToc t_predict;
        // latest_time = current_time;
        // tmp_P = estimator.Ps[WINDOW_SIZE];
        // tmp_Q = estimator.Rs[WINDOW_SIZE];
        // tmp_V = estimator.Vs[WINDOW_SIZE];
        // tmp_Ba = estimator.Bas[WINDOW_SIZE];
        // tmp_Bg = estimator.Bgs[WINDOW_SIZE];
        // acc_0 = estimator.acc_0;
        // gyr_0 = estimator.gyr_0;

        // queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
        // for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        //     predict(tmp_imu_buf.front());

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LidarImuGnssFilterFusionOdometryBridge::imuHandler( sensor_msgs::ImuConstPtr const& imu_msg)
    {
        // 保证队列中 数据的顺序正确 
        static double last_imu_t = -1; 
    
        if (imu_msg->header.stamp.toSec() <= last_imu_t)
        {
            ROS_WARN("imu message in disorder!");
            return;
        }

        last_imu_t = imu_msg->header.stamp.toSec();
        // 解析IMU数据 
        Sensor::ImuDataPtr imu_data_ptr = std::make_shared<Sensor::ImuData>();
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

        // con.notify_one();

        // last_imu_t = imu_msg->header.stamp.toSec();

        // {
        //     std::lock_guard<std::mutex> lg(m_state);
        //     predict(imu_msg);
        //     std_msgs::Header header = imu_msg->header;
        //     header.frame_id = "world";
        //     if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        //         pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
        // }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LidarImuGnssFilterFusionOdometryBridge::gnssHandler(sensor_msgs::NavSatFixConstPtr const& navsat_msg)
    {
        // 保证队列中 数据的顺序正确 
        static double last_gnss_t = -1; 

        if (navsat_msg->header.stamp.toSec() <= last_gnss_t)
        {
            ROS_WARN("gnss message in disorder!");
            return;
        }

        last_gnss_t = navsat_msg->header.stamp.toSec();
        // 解析Gnss数据 
        Sensor::GnssDataPtr gnss_data_ptr = std::make_shared<Sensor::GnssData>();
        // 保存时间戳 
        gnss_data_ptr->timestamp = navsat_msg->header.stamp.toSec();
        gnss_data_ptr->lla << navsat_msg->latitude,
                            navsat_msg->longitude,
                            navsat_msg->altitude;
        gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(navsat_msg->position_covariance.data());

        m_buf.lock();
        gnss_buf.push(gnss_data_ptr);
        m_buf.unlock();
        //con.notify_one();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // thread: Lidar-inertial odometry
    void LidarImuGnssFilterFusionOdometryBridge::Process()
    {
        while (true)
        {   
            // 如果有数据容器非空  那么进行处理 
            if(!imu_buf.empty()||!lidar_buf.empty()||!gnss_buf.empty())
            {
                uint8_t sensor_id = 0;
                double earliest_time = numeric_limits<double>::max();  
                // 找到最早的数据  
                if(!imu_buf.empty())
                {   
                    earliest_time = imu_buf.front()->timestamp;  
                    sensor_id = imu;
                }

                if(!lidar_buf.empty())
                {   
                    // 如果激光数据更早  
                    if(earliest_time > lidar_buf.front()->timestamp)
                    {
                        earliest_time = lidar_buf.front()->timestamp;
                        sensor_id = lidar;
                    }
                }

                if(!gnss_buf.empty())
                {   
                    // 如果gnss更早 
                    if(earliest_time > gnss_buf.front()->timestamp)
                    {
                        earliest_time = gnss_buf.front()->timestamp;
                        sensor_id = gnss;
                    }
                }
                
                switch(sensor_id)
                {
                    // 如果是激光里程计数据   那么 执行观测矫正  
                    case lidar:
                    {
                        // 至少要保证容器中有2个以上激光数据才进行处理 
                        if(lidar_buf.size()<2)
                        {
                            break;
                        }
                        // 取出头数据 
                        Sensor::LidarDataConstPtr lidar_ptr = lidar_buf.front();  
                        lidar_buf.pop();  
                        // 如果初始化了  那么开始校正
                        if(estimator_ptr_->IsLidarInitialized())
                        {
                            // 由于校正时，上一次IMU的预测状态的时间戳可能与当前校正的时间戳有较大的偏差
                            // 如果存在IMU数据    那么在校正之前需要插值出Lidar时间戳处的IMU数据  然后再次进行预测 
                            if(!imu_buf.empty())
                            {
                                //插值 
                                Sensor::ImuDataConstPtr curr_imu = imu_buf.front();  
                                imu_buf.pop();   
                                Sensor::ImuDataConstPtr const& last_imu = estimator_ptr_->GetLastImuData();  
                                Sensor::ImuDataConstPtr interpolation_imu = ImuDataLinearInterpolation(last_imu, curr_imu, 
                                                                                                            lidar_ptr->timestamp);                                                                           
                                //用插值后的数剧进行预测
                                estimator_ptr_->ProcessSensorData(interpolation_imu);  
                            }
                            else // 如果之后没有IMU数据 即IMU挂了
                            {  
                                // 如果IMU没有    那么可以采用匀速运动学模型进行预测   
                            }
                        }
                        // 进行校正
                    
                    }
                    break;
                    // 如果是imu数据执行 预测  
                    case imu:
                    {
                        // 取出头数据 
                        Sensor::ImuDataConstPtr imu_ptr = imu_buf.front();  
                        imu_buf.pop();  
                        estimator_ptr_->ProcessSensorData(imu_ptr);  
                        // 显示当前IMU的预测轨迹
                        if(estimator_ptr_->IsGnssInitialized())
                        {   
                            CommonStates const& common_states = estimator_ptr_->GetCommonStates();     // 获取估计后的通用状态 
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
                        if(gnss_buf.size()<2)
                        {
                            break;
                        }
                        // 取出头数据 
                        Sensor::GnssDataConstPtr gnss_ptr = gnss_buf.front();  
                        gnss_buf.pop();  
                        // 如果初始化了  那么开始校正
                        if(estimator_ptr_->IsGnssInitialized())
                        {
                            // 由于校正时，上一次IMU的预测状态的时间戳可能与当前校正的时间戳有较大的偏差
                            // 如果存在IMU数据    那么在校正之前需要插值出GNSS时间戳处的IMU数据  然后再次进行预测 
                            if(!imu_buf.empty())
                            {
                                //插值 
                                Sensor::ImuDataConstPtr curr_imu = imu_buf.front();  
                                imu_buf.pop();   
                                Sensor::ImuDataConstPtr const& last_imu = estimator_ptr_->GetLastImuData();  
                                Sensor::ImuDataConstPtr interpolation_imu = ImuDataLinearInterpolation(last_imu, curr_imu, 
                                                                                                            gnss_ptr->timestamp);                                                                           
                                //用插值后的数剧进行预测
                                estimator_ptr_->ProcessSensorData(interpolation_imu);  
                            }
                            else // 如果之后没有IMU数据 即IMU挂了
                            {  
                                // 如果IMU没有    那么可以采用匀速运动学模型进行预测   
                            }
                        }
                        // 校正 
                        estimator_ptr_->ProcessSensorData(gnss_ptr);  
                        // 保存GNSS的轨迹 
                        if(estimator_ptr_->IsGnssInitialized())
                        {   
                            // 获取更新后的GNSS轨迹 
                            GnssDataProcess* gnss_processor_ptr = GnssDataProcess::GetInstance();  
                            Eigen::Vector3d xyz = gnss_processor_ptr->GetEnuPosition();
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
                            std::cout<<"pubGnssPath! "<<" num: "<< GnssPath.poses.size() << " xyz: "<< xyz.transpose() <<std::endl;

                            // 发布滤波估计后的轨迹 
                            CommonStates const& common_states = estimator_ptr_->GetCommonStates();     // 获取估计后的通用状态 
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