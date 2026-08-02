//
// Created by shiboz on 2021-10-18.
//

#include "super_odometry/LaserMapping/laserMapping.h"

double parameters[7] = {0, 0, 0, 0, 0, 0, 1};
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters);
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters+3);

Eigen::Vector3d vel_b;
Eigen::Vector3d ang_vel_b;

namespace super_odometry {

    laserMapping::laserMapping(const rclcpp::NodeOptions & options)
    : Node("laser_mapping_node", options) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    }

    void laserMapping::initInterface() {
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        if(!readGlobalparam(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read calibration. Exiting...");
            rclcpp::shutdown();
        }

        if (!readParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }

        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[AriseSlam::laserMapping] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "DEBUG VIEW: %d", config_.debug_view_enabled);
        RCLCPP_INFO(this->get_logger(), "ENABLE OUSTER DATA: %d", config_.enable_ouster_data);
        RCLCPP_INFO(this->get_logger(), "line resolution %f plane resolution %f vision_laser_time_offset %f",
                config_.lineRes, config_.planeRes, vision_laser_time_offset);

        downSizeFilterCorner.setLeafSize(config_.lineRes, config_.lineRes, config_.lineRes);
        downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);


        subLaserFeatureInfo = this->create_subscription<super_odometry_msgs::msg::LaserFeature>(
            ProjectName+"/feature_info", 10,
            std::bind(&laserMapping::laserFeatureInfoHandler, this,
                        std::placeholders::_1), sub_options);
                        

        pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/laser_cloud_surround", 2);

        pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/laser_cloud_map", 2);

        pubLaserCloudPrior = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/overall_map", 2);

        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/registered_scan0", 2);


        pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/laser_odometry", 1);

        pubLaserOdometryIncremental = this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/aft_mapped_to_init_incremental", 1);


        pubVIOPrediction=  this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/vio_prediction", 1);

        pubLIOPrediction= this->create_publisher<nav_msgs::msg::Odometry>(
            ProjectName+"/lio_prediction", 1);


        pubLaserAfterMappedPath = this->create_publisher<nav_msgs::msg::Path>(
            ProjectName+"/laser_odom_path", 1);

        pubOptimizationStats = this->create_publisher<super_odometry_msgs::msg::OptimizationStats>(
            ProjectName+"/super_odometry_stats", 1);

  

        pubprediction_source = this->create_publisher<std_msgs::msg::String>(
            ProjectName+"/prediction_source", 1);

        process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(5.)),
            std::bind(&laserMapping::process, this));

        slam.initROSInterface(shared_from_this());
        slam.localMap.lineRes_ = config_.lineRes;
        slam.localMap.planeRes_ = config_.planeRes;
        slam.Visual_confidence_factor=config_.visual_confidence_factor;
        slam.Pos_degeneracy_threshold=config_.pos_degeneracy_threshold;
        slam.Ori_degeneracy_threshold=config_.ori_degeneracy_threshold;
        slam.LocalizationICPMaxIter=config_.max_iterations;
        slam.OptSet.debug_view_enabled=config_.debug_view_enabled;
        slam.OptSet.velocity_failure_threshold=config_.velocity_failure_threshold;
        slam.OptSet.max_surface_features=config_.max_surface_features;
        slam.OptSet.imu_roll_pitch_weight=config_.imu_roll_pitch_weight;
        slam.OptSet.map_update_interval=config_.map_update_interval;
        slam.OptSet.yaw_ratio=yaw_ratio;
        slam.map_dir=config_.map_dir;
        slam.localization_mode=config_.localization_mode;
        slam.init_x=config_.init_x;
        slam.init_y=config_.init_y;
        slam.init_z=config_.init_z;
        slam.init_roll=config_.init_roll;
        slam.init_pitch=config_.init_pitch;
        slam.init_yaw=config_.init_yaw;

        prediction_source = PredictionSource::IMU_ORIENTATION;
        timeLatestImuOdometry = rclcpp::Time(0,0,RCL_ROS_TIME);

        initializationParam();

    }

    void laserMapping::initializationParam() {

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());

        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurround.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes_rot.reset(new pcl::PointCloud<PointType>());
        laserCloudRawRes.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());
        laserCloudRealsense.reset(new pcl::PointCloud<PointType>());
        laserCloudPriorOrg.reset(new pcl::PointCloud<PointType>());
        laserCloudPrior.reset(new pcl::PointCloud<PointType>());

        Eigen::Quaterniond q_wmap_wodom_(1, 0, 0, 0);
        Eigen::Vector3d t_wmap_wodom_(0, 0, 0);
        Eigen::Quaterniond q_wodom_curr_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_curr_(0, 0, 0);
        Eigen::Quaterniond q_wodom_pre_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_pre_(0, 0, 0);

        q_wmap_wodom = q_wmap_wodom_;
        t_wmap_wodom = t_wmap_wodom_;
        q_wodom_curr = q_wodom_curr_;
        t_wodom_curr = t_wodom_curr_;
        q_wodom_pre = q_wodom_pre_;
        t_wodom_pre = t_wodom_pre_;

        imu_odom_buf.allocate(5000);
        visual_odom_buf.allocate(5000);
        
        slam.localMap.setOrigin(Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z));

        if (slam.localization_mode) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m Loading GT Map now.... Please wait for 10 sec before running rosbag.\033[0m");
            if(utils::readPointCloud(config_.map_dir, laserCloudPrior)) {
                slam.localMap.addSurfPointCloud(*laserCloudPrior);
                pcl::toROSMsg(*laserCloudPrior, priorCloudMsg);
                priorCloudMsg.header.frame_id = WORLD_FRAME;
                RCLCPP_INFO(this->get_logger(), "\033[1;32m Loading GT Map Succesfully. Localization mode is Ready.\033[0m");
            } else {
                slam.localization_mode = false;
                RCLCPP_INFO(this->get_logger(), "\033[1;32mCannot read map file, switch to mapping mode.\033[0m");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mStart SLAM in mapping mode.\033[0m");
        }
    }

    bool laserMapping::readParameters()
    {
        // Declare with default values
        this->declare_parameter("laser_mapping_node.mapping_line_resolution", 0.1);
        this->declare_parameter("laser_mapping_node.mapping_plane_resolution", 0.2);
        this->declare_parameter("laser_mapping_node.max_iterations", 4);
        this->declare_parameter("laser_mapping_node.debug_view", false);
        this->declare_parameter("laser_mapping_node.enable_ouster_data", false);
        this->declare_parameter("laser_mapping_node.publish_only_feature_points", false);
        this->declare_parameter("laser_mapping_node.use_imu_roll_pitch", false);
        this->declare_parameter("laser_mapping_node.imu_roll_pitch_weight", 100.0);
        this->declare_parameter("laser_mapping_node.max_surface_features", 2000);
        this->declare_parameter("laser_mapping_node.map_update_interval", 5);
        this->declare_parameter("laser_mapping_node.path_publish_stride", 10);
        this->declare_parameter("laser_mapping_node.max_feature_queue_size", 20);
        this->declare_parameter("laser_mapping_node.velocity_failure_threshold", 30.0);
        this->declare_parameter("laser_mapping_node.auto_voxel_size", true);
        this->declare_parameter("laser_mapping_node.forget_far_chunks", false);
        this->declare_parameter("laser_mapping_node.visual_confidence_factor", 1.0);
        this->declare_parameter("laser_mapping_node.localization_mode", false); // Add default value!
        this->declare_parameter("laser_mapping_node.read_pose_file", false);
        this->declare_parameter("laser_mapping_node.init_x", 0.0);
        this->declare_parameter("laser_mapping_node.init_y", 0.0);
        this->declare_parameter("laser_mapping_node.init_z", 0.0);
        this->declare_parameter("laser_mapping_node.init_roll", 0.0);
        this->declare_parameter("laser_mapping_node.init_pitch", 0.0);
        this->declare_parameter("laser_mapping_node.init_yaw", 0.0);
        this->declare_parameter("map_dir", "pointcloud_local.pcd");


        // Get parameters
        config_.lineRes = this->get_parameter("laser_mapping_node.mapping_line_resolution").as_double();
        config_.planeRes = this->get_parameter("laser_mapping_node.mapping_plane_resolution").as_double();
        config_.max_iterations = this->get_parameter("laser_mapping_node.max_iterations").as_int();
        config_.debug_view_enabled = this->get_parameter("laser_mapping_node.debug_view").as_bool();
        config_.enable_ouster_data = this->get_parameter("laser_mapping_node.enable_ouster_data").as_bool();
        config_.publish_only_feature_points = this->get_parameter("laser_mapping_node.publish_only_feature_points").as_bool();
        config_.use_imu_roll_pitch = this->get_parameter("laser_mapping_node.use_imu_roll_pitch").as_bool();
        config_.imu_roll_pitch_weight = this->get_parameter("laser_mapping_node.imu_roll_pitch_weight").as_double();
        config_.max_surface_features = this->get_parameter("laser_mapping_node.max_surface_features").as_int();
        config_.map_update_interval = this->get_parameter("laser_mapping_node.map_update_interval").as_int();
        config_.path_publish_stride = this->get_parameter("laser_mapping_node.path_publish_stride").as_int();
        config_.max_feature_queue_size = this->get_parameter("laser_mapping_node.max_feature_queue_size").as_int();
        config_.velocity_failure_threshold = this->get_parameter("laser_mapping_node.velocity_failure_threshold").as_double();
        config_.auto_voxel_size = this->get_parameter("laser_mapping_node.auto_voxel_size").as_bool();
        config_.forget_far_chunks = this->get_parameter("laser_mapping_node.forget_far_chunks").as_bool();
        config_.visual_confidence_factor = this->get_parameter("laser_mapping_node.visual_confidence_factor").as_double();
        config_.map_dir = this->get_parameter("map_dir").as_string(); 
        config_.localization_mode = this->get_parameter("laser_mapping_node.localization_mode").as_bool();
        config_.read_pose_file = this->get_parameter("laser_mapping_node.read_pose_file").as_bool();

        if(config_.read_pose_file)
        {   
            std::vector<utils::OdometryData> odometryResults;
            utils::readLocalizationPose(config_.map_dir, odometryResults);
            config_.init_x= odometryResults[0].x;
            config_.init_y= odometryResults[0].y;
            config_.init_z= odometryResults[0].z;
            config_.init_roll= odometryResults[0].roll;
            config_.init_pitch= odometryResults[0].pitch;
            config_.init_yaw= odometryResults[0].yaw;
        }
        else
        {  
            config_.init_x = get_parameter("laser_mapping_node.init_x").as_double(); 
            config_.init_y = get_parameter("laser_mapping_node.init_y").as_double(); 
            config_.init_z = get_parameter("laser_mapping_node.init_z").as_double(); 
            config_.init_roll = get_parameter("laser_mapping_node.init_roll").as_double();
            config_.init_pitch = get_parameter("laser_mapping_node.init_pitch").as_double();
            config_.init_yaw = get_parameter("laser_mapping_node.init_yaw").as_double(); 
        }

        return true;
    }
    
   

    void laserMapping::laserFeatureInfoHandler(const super_odometry_msgs::msg::LaserFeature::SharedPtr msgIn) {
        std::lock_guard<std::mutex> lock(mBuf);
        featureBuf.push_back(msgIn);
        ++received_feature_frames_;

        const std::size_t max_queue =
            static_cast<std::size_t>(std::max(1, config_.max_feature_queue_size));
        if (featureBuf.size() > max_queue) {
            featureBuf.pop_front();
            ++dropped_feature_frames_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Mapping queue overflow: dropped %lu complete frames "
                "(received %lu, processed %lu)",
                static_cast<unsigned long>(dropped_feature_frames_.load()),
                static_cast<unsigned long>(received_feature_frames_.load()),
                static_cast<unsigned long>(processed_feature_frames_.load()));
        }
    }


void laserMapping::setInitialGuess()
{
  //Case1: First Frame Initialization 
  if(!initialization){
    initializeFirstFrame();
    return;
  }
  //Case2: Startup period -continue using IMU for stability 
  if(startupCount>0){
    initializeWithIMU();
    startupCount--;
    return; 
  }
  //Case3: Normal operation -select prediction source 
  selectPosePrediction();
}

void laserMapping::initializeFirstFrame(){

    //Get initial orientation from IMU prediction 
    if(sensorMeas.imuPrediction.w()!=0){   //Have IMU data
        //Extract roll and pitch, zero out yaw 
        tf2::Quaternion initial_orientation=utils::extractRollPitch(sensorMeas.imuPrediction);
        q_w_curr=Eigen::Quaterniond(initial_orientation.w(), initial_orientation.x(),
              initial_orientation.y(), initial_orientation.z());
        imu_world_alignment = q_w_curr * sensorMeas.imuPrediction.inverse();
        imu_world_alignment.normalize();
        imu_world_alignment_initialized = true;
        
        
    }else{

        q_w_curr=Eigen::Quaterniond(1,0,0,0); //If no IMU data, use identity rotation 

    }

    //initialize position 
    q_wodom_pre=q_w_curr;
    T_w_lidar.rot=q_w_curr;
    T_w_lidar.pos=Eigen::Vector3d::Zero();

    //Overide with predefined pose if localization mode 
    if(slam.localization_mode){
        T_w_lidar.pos=Eigen::Vector3d(slam.init_x,slam.init_y,slam.init_z);
        tf2::Quaternion localization_pose;
        localization_pose.setRPY(slam.init_roll, slam.init_pitch,slam.init_yaw);
        T_w_lidar.rot=Eigen::Quaterniond(localization_pose.w(), localization_pose.x(),
                                        localization_pose.y(), localization_pose.z());
        slam.last_T_w_lidar=T_w_lidar;
    }

}

void laserMapping::initializeWithIMU(){
    if(sensorMeas.imuPrediction.w()!=0){  //Have IMU data
    // Use the IMU orientation in the mapping frame established at startup.
    const Eigen::Quaterniond curr_imu = alignedImuPrediction(sensorMeas.imuPrediction);
    
    //Keep position from last frame 
    t_w_curr=last_T_w_lidar.pos;
    T_w_lidar.pos=t_w_curr;

    //Update rotation 
    q_w_curr=curr_imu;
    T_w_lidar.rot=q_w_curr;
    // Recovery/startup forces the absolute IMU attitude. Keep the incremental
    // predictor synchronized so the first normal frame does not reapply the
    // entire IMU rotation accumulated during the recovery window.
    q_wodom_pre=curr_imu;


    }else
    {
      //No IMU data, use last rotation 
      q_w_curr=last_T_w_lidar.rot;
      t_w_curr=last_T_w_lidar.pos;
      T_w_lidar=last_T_w_lidar;

    } 
}

void laserMapping::selectPosePrediction(){

// Step1: Decide prediction source based on system state 
prediction_source=determinePredictionSource();

//Step2: Get prediction from selected source 
switch(prediction_source){
    case PredictionSource::LIO_ODOM:{
    T_w_lidar= T_w_lidar*sensorMeas.lioPrediction;
    break;
    } 
   
    case PredictionSource::VIO_ODOM:{
    T_w_lidar= T_w_lidar*sensorMeas.vioPrediction;
    break; 
    } 

    case PredictionSource::NEURAL_IMU_ODOM:{
    T_w_lidar= T_w_lidar*sensorMeas.nioPrediction;
    break; 
    } 
    case PredictionSource::IMU_ORIENTATION:{
    Eigen::Quaterniond q_w_predict=q_w_curr*q_wodom_pre.inverse()*q_wodom_curr;
    q_w_predict.normalize();
    T_w_lidar.rot=q_w_predict;
    q_wodom_pre=q_wodom_curr;
    break;
    } 
   
    case PredictionSource::CONSTANT_VELOCITY:{  
    Transformd relative_pose=last_T_w_lidar.inverse()*T_w_lidar;
    T_w_lidar=T_w_lidar*relative_pose;
    break; 
    }
}

//Step4: Update current pose 
q_w_curr=T_w_lidar.rot;
t_w_curr=T_w_lidar.pos;

}

Eigen::Quaterniond laserMapping::alignedImuPrediction(
    const Eigen::Quaterniond& imuPrediction) const {
    Eigen::Quaterniond aligned = imuPrediction;
    if (imu_world_alignment_initialized) {
        aligned = imu_world_alignment * imuPrediction;
    }
    aligned.normalize();
    return aligned;
}

laserMapping::PredictionSource laserMapping::determinePredictionSource(){
// If system is degerenate, prefer VIO or learning imu odom

if(slam.isDegenerate){
    if(sensorMeas.vio_prediction_status){
        return PredictionSource::VIO_ODOM;
    }
    if(sensorMeas.nio_prediction_status){
        return PredictionSource::NEURAL_IMU_ODOM;
    }

}else{
    // If system is not degenerate, use IMU orientation 
    if(sensorMeas.lio_prediction_status){
        return PredictionSource::LIO_ODOM;
    }
    sensorMeas.imu_orientation_status=useIMUPrediction(sensorMeas.imuPrediction);
    if(sensorMeas.imu_orientation_status){
        return PredictionSource::IMU_ORIENTATION;
    }
   
}



// If no prediction source is available, use constant velocity
return PredictionSource::CONSTANT_VELOCITY;

}

    void laserMapping::publishTopic(){

        TicToc t_pub;
        std_msgs::msg::String prediction_source_msg;
        switch (prediction_source) {
            case PredictionSource::IMU_ORIENTATION :
                prediction_source_msg.data = "IMU Only Orientation Prediction";
                break;
            case PredictionSource::LIO_ODOM :
                prediction_source_msg.data = "Using Laser-Inertial Odometry (LIO)";
                break;
            case PredictionSource::VIO_ODOM :
                prediction_source_msg.data = "Using Visual-Inertial Odometry (VIO)";
                break;
            case PredictionSource::NEURAL_IMU_ODOM :
                prediction_source_msg.data = "Using Neural-Inertial Odometry (Neural-IMU)";
                break;
            case PredictionSource::CONSTANT_VELOCITY :
                prediction_source_msg.data = "Using Constant Velocity Prediction";
                break;
        }
        pubprediction_source->publish(prediction_source_msg);

        if (frameCount % 5 == 0 && config_.debug_view_enabled) {
            laserCloudSurround->clear();
            *laserCloudSurround = slam.localMap.get5x5LocalMap(slam.pos_in_localmap);
            sensor_msgs::msg::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
            laserCloudSurround3.header.stamp =
                    rclcpp::Time(timeLaserOdometry*1e9);
            laserCloudSurround3.header.frame_id = WORLD_FRAME;
            pubLaserCloudSurround->publish(laserCloudSurround3);
        }

        if (frameCount % 20 == 0 &&
            (pubLaserCloudMap->get_subscription_count() > 0 ||
             pubLaserCloudPrior->get_subscription_count() > 0)) {
            pcl::PointCloud<PointType> laserCloudMap;
            laserCloudMap = slam.localMap.getAllLocalMap();
            sensor_msgs::msg::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(laserCloudMap, laserCloudMsg);
            laserCloudMsg.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserCloudMsg.header.frame_id = WORLD_FRAME;
            pubLaserCloudMap->publish(laserCloudMsg);
            
            if (slam.localization_mode) {
                priorCloudMsg.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
                pubLaserCloudPrior->publish(priorCloudMsg);
            }
        }

        if (pubLaserCloudFullRes->get_subscription_count() > 0) {
            pcl::PointCloud<PointType> registered_scan;
            registered_scan.reserve(laserCloudFullRes->size());
            for (const auto &point : laserCloudFullRes->points) {
                if (point.x * point.x + point.y * point.y + point.z * point.z <= 0.01) {
                    continue;
                }
                PointType transformed;
                utils::pointAssociateToMap(
                    &point, &transformed, q_w_curr, t_w_curr);
                registered_scan.push_back(transformed);
            }
            sensor_msgs::msg::PointCloud2 registered_msg;
            pcl::toROSMsg(registered_scan, registered_msg);
            registered_msg.header.stamp = rclcpp::Time(timeLaserOdometry * 1e9);
            registered_msg.header.frame_id = WORLD_FRAME;
            pubLaserCloudFullRes->publish(registered_msg);
        }

        nav_msgs::msg::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = WORLD_FRAME;
        odomAftMapped.child_frame_id = SENSOR_FRAME;
        odomAftMapped.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);

        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();

        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();

        odomAftMapped.twist.twist.linear.x = vel_b.x();
        odomAftMapped.twist.twist.linear.y = vel_b.y();
        odomAftMapped.twist.twist.linear.z = vel_b.z();

        odomAftMapped.twist.twist.angular.x = ang_vel_b.x();
        odomAftMapped.twist.twist.angular.y = ang_vel_b.y();
        odomAftMapped.twist.twist.angular.z = ang_vel_b.z();

        nav_msgs::msg::Odometry laserOdomIncremental;

        if (initialization == false)
        {
            laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserOdomIncremental.header.frame_id = WORLD_FRAME;
            laserOdomIncremental.child_frame_id =  SENSOR_FRAME;
            laserOdomIncremental.pose.pose.position.x = t_w_curr.x();
            laserOdomIncremental.pose.pose.position.y = t_w_curr.y();
            laserOdomIncremental.pose.pose.position.z = t_w_curr.z();
            laserOdomIncremental.pose.pose.orientation.x = q_w_curr.x();
            laserOdomIncremental.pose.pose.orientation.y = q_w_curr.y();
            laserOdomIncremental.pose.pose.orientation.z = q_w_curr.z();
            laserOdomIncremental.pose.pose.orientation.w = q_w_curr.w();
        }
        else
        {

            laser_incremental_T = T_w_lidar;
            laser_incremental_T.rot.normalized();

            laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry*1e9);
            laserOdomIncremental.header.frame_id = WORLD_FRAME;
            laserOdomIncremental.child_frame_id =  SENSOR_FRAME;
            laserOdomIncremental.pose.pose.position.x = laser_incremental_T.pos.x();
            laserOdomIncremental.pose.pose.position.y = laser_incremental_T.pos.y();
            laserOdomIncremental.pose.pose.position.z = laser_incremental_T.pos.z();
            laserOdomIncremental.pose.pose.orientation.x = laser_incremental_T.rot.x();
            laserOdomIncremental.pose.pose.orientation.y = laser_incremental_T.rot.y();
            laserOdomIncremental.pose.pose.orientation.z = laser_incremental_T.rot.z();
            laserOdomIncremental.pose.pose.orientation.w = laser_incremental_T.rot.w();
        }

        pubLaserOdometryIncremental->publish(laserOdomIncremental);


        if (slam.isDegenerate) {
            odomAftMapped.pose.covariance[0] = 1;
        } else {
            odomAftMapped.pose.covariance[0] = 0;
        }

        rclcpp::Time pub_time = rclcpp::Clock{RCL_ROS_TIME}.now(); //PARV_TODO - find how to syncrynoise this with rosbag time
        pubOdomAftMapped->publish(odomAftMapped);

        geometry_msgs::msg::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = WORLD_FRAME;
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        const int path_stride = std::max(1, config_.path_publish_stride);
        if (frameCount == 1 || frameCount % path_stride == 0) {
            pubLaserAfterMappedPath->publish(laserAfterMappedPath);
        }


        slam.stats.header = odomAftMapped.header;
        if (timeLatestImuOdometry.seconds() < 1.0)
        {
            timeLatestImuOdometry = pub_time;
        }
        rclcpp::Duration latency = timeLatestImuOdometry - pub_time;  
        slam.stats.latency = latency.seconds() * 1000;
        slam.stats.n_iterations = slam.stats.iterations.size();
        // Avoid breaking rqt_multiplot
        while (slam.stats.iterations.size() < 4)
        {
            slam.stats.iterations.push_back(super_odometry_msgs::msg::IterationStats());
        }

        pubOptimizationStats->publish(slam.stats);
        slam.stats.iterations.clear();
    }


    void laserMapping::adjustVoxelSize(){

        // Calculate cloud statistics
        bool increase_blind_radius = false;
        if(config_.auto_voxel_size)
        {
            if (laserCloudSurfLast->empty()) {
                laserCloudCornerStack->clear();
                laserCloudSurfStack->clear();
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Received an empty surface cloud; skipping voxel adjustment");
                return;
            }
            Eigen::Vector3f average(0,0,0);
            int count_far_points = 0;
            for (auto &point : *laserCloudSurfLast)
            {
                average(0) += fabs(point.x);
                average(1) += fabs(point.y);
                average(2) += fabs(point.z);
                if(point.x*point.x + point.y*point.y + point.z*point.z>9){
                    count_far_points++;
                }
            }
            if (count_far_points > 3000)
            {
                increase_blind_radius = true;
            }

            average /= laserCloudSurfLast->points.size();
            slam.stats.average_distance = average(0)*average(1)*average(2);
            if (slam.stats.average_distance < 25)
            {
                config_.lineRes = 0.1;
                config_.planeRes = 0.2;
            }
            else if (slam.stats.average_distance > 65)
            {
                config_.lineRes = 0.4;
                config_.planeRes = 0.8;
            }
            downSizeFilterSurf.setLeafSize(config_.planeRes , config_.planeRes , config_.planeRes );
            downSizeFilterCorner.setLeafSize(config_.lineRes , config_.lineRes , config_.lineRes );
        }

        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerStack);


        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
      

        slam.localMap.lineRes_ = config_.lineRes;
        slam.localMap.planeRes_ = config_.planeRes;

    }

    
    bool  laserMapping::checkDataAvailable() const{
        return !featureBuf.empty();
    }

    laserMapping::SensorData laserMapping::extractSensorData(
        const super_odometry_msgs::msg::LaserFeature::SharedPtr &msg){
        
        SensorData data;
        //1. Extract timestamp
        data.timestamp=secs(msg);
        timeLaserOdometry=data.timestamp;

        //2. Extract point cloud data 
        pcl::fromROSMsg(msg->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msg->cloud_surface, *laserCloudSurfLast);
        if (pubLaserCloudFullRes->get_subscription_count() > 0) {
            pcl::fromROSMsg(msg->cloud_nodistortion, *laserCloudFullRes);
        } else {
            laserCloudFullRes->clear();
        }

        //3. Extract IMU prediction 
        data.imuPrediction = Eigen::Quaterniond(
            msg->initial_quaternion_w, msg->initial_quaternion_x,
            msg->initial_quaternion_y, msg->initial_quaternion_z);
        data.imuPrediction.normalize();

        //4 set status for prediction source (TODO: didn't release code other prediction source yet) 
        data.vio_prediction_status=false;
        data.lio_prediction_status=false;
        data.nio_prediction_status=false;
        data.imu_orientation_status=false;

        return data;
    }

    void laserMapping::performSLAMOptimization(){
        tf2::Quaternion imu_roll_pitch;
        if(config_.use_imu_roll_pitch){  // TODO: Livox mid360 not use roll pitch angle
            slam.OptSet.use_imu_roll_pitch=true;
            Eigen::Quaterniond aligned = alignedImuPrediction(sensorMeas.imuPrediction);
            imu_roll_pitch=utils::extractRollPitch(aligned);
            slam.OptSet.imu_roll_pitch=imu_roll_pitch;
        }else{
            slam.OptSet.use_imu_roll_pitch=false;
            slam.OptSet.imu_roll_pitch=tf2::Quaternion(0,0,0,1);
        }

        slam.Localization(initialization, static_cast<LidarSLAM::PredictionSource>(prediction_source), T_w_lidar,
                 laserCloudCornerStack, laserCloudSurfStack, timeLaserOdometry);
    }


    bool laserMapping::useIMUPrediction(const Eigen::Quaterniond& imuPrediction){
        if (imuPrediction.w()!=0)
        {
            q_wodom_curr=alignedImuPrediction(imuPrediction);
            q_wodom_curr.normalize();
            return true;
        }
        else
        {
            return false;
        }
    }

    void laserMapping::updatePoseAndPublish(){

        //1. Update pose 
        q_w_curr=slam.T_w_lidar.rot;
        t_w_curr=slam.T_w_lidar.pos;
        T_w_lidar.rot=slam.T_w_lidar.rot;
        T_w_lidar.pos=slam.T_w_lidar.pos;
        startupCount=slam.startupCount;
        frameCount++;
        slam.frame_count=frameCount;
        slam.laser_imu_sync=laser_imu_sync;
        initialization = true;

        // Calculate linear and angular velocity
        double dt = timeLaserOdometry - timeLaserOdometryPrev;

        if (dt > 1e-6) {  
            Eigen::Vector3d vel_w = (t_w_curr - last_T_w_lidar.pos) / dt;
            vel_b = q_w_curr.inverse() * vel_w;     

            Eigen::Quaterniond dq = q_w_curr * last_T_w_lidar.rot.inverse();
            Eigen::AngleAxisd angle_axis(dq);
            Eigen::Vector3d ang_vel_w = angle_axis.axis() * angle_axis.angle() / dt;
            ang_vel_b = q_w_curr.inverse() * ang_vel_w;
        } else {
            vel_b = Eigen::Vector3d::Zero();
            ang_vel_b = Eigen::Vector3d::Zero();
        }

        //2. Publish results 
        publishTopic();

        //3. Store current pose and time for next iteration
        last_T_w_lidar = slam.T_w_lidar;
        timeLaserOdometryPrev = timeLaserOdometry;
    }

    void laserMapping::process() {
        super_odometry_msgs::msg::LaserFeature::SharedPtr feature_msg;
        {
            std::lock_guard<std::mutex> lock(mBuf);
            if (!checkDataAvailable()) {
                return;
            }
            feature_msg = featureBuf.front();
            featureBuf.pop_front();
        }

        const auto frame_start = std::chrono::steady_clock::now();
        try {
            sensorMeas = extractSensorData(feature_msg);
            setInitialGuess();
            adjustVoxelSize();
            performSLAMOptimization();
            updatePoseAndPublish();
            ++processed_feature_frames_;
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error in frame processing: %s", e.what());
        }

        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - frame_start).count();
        if (elapsed_ms > 100) {
            std::size_t queued_frames = 0;
            {
                std::lock_guard<std::mutex> lock(mBuf);
                queued_frames = featureBuf.size();
            }
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Mapping is slower than 10 Hz: last frame %ld ms, queue %zu, "
                "received %lu, processed %lu, dropped %lu",
                elapsed_ms, queued_frames,
                static_cast<unsigned long>(received_feature_frames_.load()),
                static_cast<unsigned long>(processed_feature_frames_.load()),
                static_cast<unsigned long>(dropped_feature_frames_.load()));
        }
    }


#pragma clang diagnostic pop

} // namespace super_odometry
