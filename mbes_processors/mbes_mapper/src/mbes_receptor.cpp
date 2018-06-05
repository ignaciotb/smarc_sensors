#include "mbes_mapper/mbes_receptor.hpp"


MBESReceptor::MBESReceptor(ros::NodeHandle &nh, std::string node_name):
    node_name_(node_name), nh_(&nh){

    std::string mbes_topic;
    std::string pcl_pub_topic;
    std::string auv_pose_topic;
    std::string pcl_poses_topic;
    std::vector<double> Q_mbes_diag;

    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/mbes_frame"), mbes_frame_, "/mbes_link");
    nh_->param<std::string>((node_name_ + "/map_frame"), map_frame_, "/map_link");
    nh_->param<int>((node_name_ + "/submap_size"), submap_size_, 5);
    nh_->param<std::string>((node_name_ + "/mbes_laser_topic"), mbes_topic, "/mbes_laser_topic");
    nh_->param<std::string>((node_name_ + "/pcl_pub_topic"), pcl_pub_topic, "/pcl_pub_topic");
    nh_->param<std::string>((node_name_ + "/pose_estimate_topic"), auv_pose_topic, "/pose_estimate_topic");
    nh_->param<std::string>((node_name_ + "/pcl_poses_topic"), pcl_poses_topic, "/pcl_poses_topic");
    nh_->param("meas_mbes_noise_cov_diag", Q_mbes_diag, std::vector<double>());

    // Subscribers for MBES pings and AUV poses
    mbes_laser_sub_ = nh_->subscribe(mbes_topic, 10, &MBESReceptor::MBESLaserCB, this);
    auv_pose_sub_ = nh_->subscribe(auv_pose_topic, 10, &MBESReceptor::auvPoseCB, this);

    // RVIZ pcl output for testing
    pcl_pub_ = nh_->advertise<sensor_msgs::PointCloud2> (pcl_pub_topic, 2);
    pcl_poses_pub_ = nh_->advertise<geometry_msgs::PoseWithCovariance>(pcl_poses_topic, 10);

    this->init(Q_mbes_diag);
    ros::spin();
}


void MBESReceptor::auvPoseCB(const nav_msgs::Odometry auv_pose_t){
    auv_poses_.push_back(auv_pose_t);
    while(auv_poses_.size() > submap_size_ + 1){
        auv_poses_.pop_front();
    }
}


void MBESReceptor::init(std::vector<double> q_mbes_diag){
    tf::TransformListener tf_listener;
    try{
        tf_listener.waitForTransform(base_frame_, mbes_frame_, ros::Time(0), ros::Duration(100));
        tf_listener.lookupTransform(base_frame_, mbes_frame_, ros::Time(0), tf_base_mbes_);
        ROS_INFO_STREAM(node_name_ << ": Locked transform MBES --> base");
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
    }

    // Meas noise model covariance
    Eigen::Matrix3f Sigma_mbes = Eigen::Matrix3f::Identity();
    for(unsigned int i=0; i<q_mbes_diag.size(); i++){
        Sigma_mbes(i,i) = q_mbes_diag.at(i);
    }

    // Information form
    Omega_mbes_ = Sigma_mbes.inverse();

    ROS_INFO_STREAM(node_name_ << ": launched");

}


void MBESReceptor::savePointCloud(PointCloud submap_pcl, std::string file_name){

    std::ofstream myfile ("/home/nacho/Documents/Varios/" + file_name);
    if (myfile.is_open()){
        for(unsigned int i=0; i<submap_pcl.points.size(); i++){
            myfile << submap_pcl.points.at(i).x << " " << submap_pcl.points.at(i).y << " " << submap_pcl.points.at(i).z << "\n";
        }
      myfile.close();
    }
}


void MBESReceptor::bcMapSubmapsTF(std::vector<tf::Transform> tfs_meas_map){

    int cnt_i = 0;
    tf::StampedTransform tf_map_submap_stp;
    geometry_msgs::TransformStamped msg_map_submap;
    for(tf::Transform tf_measi_map: tfs_meas_map){
         tf_map_submap_stp = tf::StampedTransform(tf_measi_map,
                                                  ros::Time::now(),
                                                  map_frame_,
                                                  "submap_" + std::to_string(cnt_i) + "_frame");

         cnt_i += 1;
         tf::transformStampedTFToMsg(tf_map_submap_stp, msg_map_submap);
         submaps_bc_.sendTransform(msg_map_submap);
    }
}

void MBESReceptor::pubPCLPosesWithCov(const PointCloud& ping, const std::vector<Eigen::Matrix3f>& ping_covs){

    // Parse pcl points positions and covariances
    int point_cnt = 0;
    geometry_msgs::PoseWithCovariance pcl_point;
//    pcl_point.header.stamp = ros::Time::now();
//    pcl_point.header.frame_id = "submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame";
    for(pcl::PointXYZ point: ping){
        pcl_point.pose.position.x = point.x;
        pcl_point.pose.position.y = point.y;
        pcl_point.pose.position.z = point.z;

        for(unsigned int i = 0; i < 3; i++) {
          for(unsigned int j = 0; j < 3; j++) {
            pcl_point.covariance[i*3 + j] = ping_covs.at(point_cnt)(i,j);
          }
        }

        point_cnt += 1;
        pcl_poses_pub_.publish(pcl_point);
    }
    std::cout << "Number of points in ping " << point_cnt << std::endl;



//    // Construct msg header
//    nav_msgs::Odometry pcl_pose_msg;
//    pcl_pose_msg.header.stamp = ros::Time::now();
//    pcl_pose_msg.header.frame_id = "submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame";

//    // Parse pcl points positions and covariances
//    int point_cnt = 0;
//    tf::Quaternion q_auv_t = tf::createQuaternionFromRPY(0, -M_PI/2, 0).normalize();
//    geometry_msgs::Quaternion odom_quat;
//    tf::quaternionTFToMsg(q_auv_t, odom_quat);

//    for(pcl::PointXYZ point: ping){
//        pcl_pose_msg.pose.pose.position.x = point.x;
//        pcl_pose_msg.pose.pose.position.y = point.y;
//        pcl_pose_msg.pose.pose.position.z = point.z;
//        pcl_pose_msg.pose.pose.orientation = odom_quat;
//        for(unsigned int i = 0; i < 3; i++) {
//          for(unsigned int j = 0; j < 3; j++) {
//            pcl_pose_msg.pose.covariance[i*3 + j] = ping_covs.at(point_cnt)(i,j);
//          }
//        }

//        point_cnt += 1;
//        pcl_poses_pub_.publish(pcl_pose_msg);
//    }

}

void MBESReceptor::pclFuser(){

    // Build and store frame of new submap meas
    ROS_INFO("PCL fuser called");
    tf::Transform tf_submap_map = mbes_swath_.at((submap_size_-1)/2).tf_base_map_;
    tf_map_meas_vec_.push_back(tf_submap_map.inverse());

    // Broadcast all meas frames (for testing?)
    bcMapSubmapsTF(tf_map_meas_vec_);

    // Transform and concatenate the PCLs to form the swath
    PointCloud tfed_pcl;
    PointCloud submap_pcl;

    // For each ping
    for(MbesPing ping: mbes_swath_){
        // Transform from base at t to meas frame
        pcl_ros::transformPointCloud(ping.mbes_pcl_, tfed_pcl, tf_submap_map * ping.tf_base_map_.inverse());
        // Add to submap pcl
        submap_pcl += tfed_pcl;
//        pubPCLPosesWithCov(tfed_pcl, ping.points_cov_vec_);
    }

    // Create ROS msg and publish
    sensor_msgs::PointCloud2 submap_msg;
    pcl::toROSMsg(submap_pcl, submap_msg);
    submap_msg.header.frame_id = "submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame";
    submap_msg.header.stamp = ros::Time::now();

    pcl_pub_.publish(submap_msg);

    // Save PCL to external file for testing
//    savePointCloud(submap_pcl, "submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame.xyz");
}


void MBESReceptor::MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in){

    // LaserScan to PCL
    ros::Time ping_acq_t = scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment);
    if(!tf_listener_.waitForTransform(mbes_frame_,
                                      base_frame_,
                                      ping_acq_t,
                                      ros::Duration(0.1))){
        ROS_INFO("Skipping iteration");
        // TODO: handle this somehow?
        return;
    }
    sensor_msgs::PointCloud2 scan_cloud;
    PointCloud pcl_cloud;
    projector_.transformLaserScanToPointCloud(base_frame_, *scan_in, scan_cloud, tf_listener_);

    // Convert from PointCloud2 to PCL pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(scan_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

    // Store ping acquisition time
    pcl_cloud.header.stamp = scan_in->header.stamp.sec;

    // Listen to tf map --> base pose
    try {
        // TODO: can this be done faster?
        tf_listener_.waitForTransform(base_frame_, map_frame_, ping_acq_t, ros::Duration(0.1));
        tf_listener_.lookupTransform(base_frame_, map_frame_, ping_acq_t, tf_base_map_);
        ROS_DEBUG_STREAM(node_name_ << ": locked transform map --> base at t");

        // Store MBES swath info: PCL, tf and points covariances at t_meas
        mbes_swath_.emplace_back(MbesPing(pcl_cloud,
                                          tf::Transform(tf_base_map_.getRotation().normalize(), tf_base_map_.getOrigin()),
                                          Omega_mbes_.inverse(),    // TODO: remove the inverse()!! Just for testing
                                          auv_poses_));
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
    }

    // When desired num of pings per swath is reached, merge them and clear buffer
    if(mbes_swath_.size() == submap_size_){
        this->pclFuser();
        mbes_swath_.clear();
    }
}

MBESReceptor::~MBESReceptor(){

}




