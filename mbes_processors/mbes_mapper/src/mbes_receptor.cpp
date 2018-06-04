#include "mbes_mapper/mbes_receptor.hpp"


MBESReceptor::MBESReceptor(ros::NodeHandle &nh, std::string node_name):
    node_name_(node_name), nh_(&nh){

    std::string mbes_topic;
    std::string pcl_pub_topic;
    std::string auv_pose_topic;
    std::vector<double> Q_mbes_diag;

    nh_->param<std::string>((node_name_ + "/base_frame"), base_frame_, "/base_link");
    nh_->param<std::string>((node_name_ + "/mbes_frame"), mbes_frame_, "/mbes_link");
    nh_->param<std::string>((node_name_ + "/map_frame"), map_frame_, "/map_link");
    nh_->param<int>((node_name_ + "/submap_size"), meas_size_, 5);
    nh_->param<std::string>((node_name_ + "/mbes_laser_topic"), mbes_topic, "/mbes_laser_topic");
    nh_->param<std::string>((node_name_ + "/pcl_pub_topic"), pcl_pub_topic, "/pcl_pub_topic");
    nh_->param<std::string>((node_name_ + "/pose_estimate_topic"), auv_pose_topic, "/pose_estimate_topic");
    nh_->param("meas_mbes_noise_cov_diag", Q_mbes_diag, std::vector<double>());

    // RVIZ pcl output for testing
    mbes_laser_sub_ = nh_->subscribe(mbes_topic, 10, &MBESReceptor::MBESLaserCB, this);
//    auv_pose_sub_ = nh_->subscribe(auv_pose_topic, 10, )
    pcl_pub_ = nh_->advertise<sensor_msgs::PointCloud2> (pcl_pub_topic, 2);

    this->init(Q_mbes_diag);
    ros::spin();
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
    ROS_INFO_STREAM(node_name_ << ": launched");

    // Meas noise model covariance
    Q_mbes_ = Eigen::Matrix3f::Identity();
    for(unsigned int i=0; i<q_mbes_diag.size(); i++){
        Q_mbes_(i,i) = q_mbes_diag.at(i);
    }

}

void MBESReceptor::pclFuser(){

    ROS_INFO("PCL fuser called");

    // Check time stamps for max distance covered in the new swath

    // Build and store frame of new submap meas
    tf::Transform tf_submap_map = std::get<1>(mbes_swath_.at((meas_size_-1)/2));
    tf_map_meas_vec_.push_back(tf_submap_map.inverse());

    // Broadcast all meas frames (for testing?)
    bcMapSubmapsTF(tf_map_meas_vec_);


    // Transform and concatenate the PCLs to form the swath
    PointCloud tfed_pcl;
    PointCloud submap_pcl;

    // For each ping
    for(std::tuple<PointCloud, tf::Transform, std::vector<Eigen::Matrix3f>> ping: mbes_swath_){
        // Transform from base at t to meas frame
        pcl_ros::transformPointCloud(std::get<0>(ping), tfed_pcl, tf_submap_map * std::get<1>(ping).inverse());
        // Add to submap pcl
        submap_pcl += tfed_pcl;
    }

    // Create ROS msg and publish
    sensor_msgs::PointCloud2 submap_msg;
    pcl::toROSMsg(submap_pcl, submap_msg);
    submap_msg.header.frame_id = "submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame";
    submap_msg.header.stamp = ros::Time::now();

    pcl_pub_.publish(submap_msg);
    savePointCloud(submap_pcl, "submap_" + std::to_string(tf_map_meas_vec_.size()-1) + "_frame.xyz");
}


void MBESReceptor::savePointCloud(PointCloud submap_pcl, std::string file_name){

    std::ofstream myfile ("/home/nacho/" + file_name);
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

void MBESReceptor::MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in){

    // LaserScan to PCL
    if(!tf_listener_.waitForTransform(mbes_frame_,
                                      base_frame_,
                                      scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
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

    // Listen to tf map --> base pose
    try {
        // TODO: can this made faster?
        tf_listener_.waitForTransform(base_frame_, map_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(0.1));
        tf_listener_.lookupTransform(base_frame_, map_frame_, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), tf_base_map_);
        ROS_DEBUG_STREAM(node_name_ << ": locked transform map --> base at t");

        // Store MBES swath info: PCL, tf and points covariances at t_meas
        std::vector<Eigen::Matrix3f> points_cov_vec(pcl_cloud.size(), Q_mbes_);
        mbes_swath_.emplace_back(pcl_cloud, tf::Transform(tf_base_map_.getRotation().normalize(), tf_base_map_.getOrigin()), points_cov_vec);
    }
    catch(tf::TransformException &exception) {
        ROS_ERROR("%s", exception.what());
    }

    // When desired num of pings per swath is reached, merge them and clear buffer
    if(mbes_swath_.size() == meas_size_){
        this->pclFuser();
        mbes_swath_.clear();
    }
}

MBESReceptor::~MBESReceptor(){

}




