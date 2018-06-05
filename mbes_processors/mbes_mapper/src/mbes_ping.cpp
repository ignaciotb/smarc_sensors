#include <mbes_ping/mbes_ping.hpp>

MbesPing::MbesPing(PointCloud &mbes_pcl,
                   const tf::Transform &tf_base_map,
                   const Eigen::Matrix3f& Omega_mbes,
                   const std::deque<nav_msgs::Odometry> &auv_poses): mbes_pcl_(mbes_pcl), tf_base_map_(tf_base_map){

    // Extract cov of AUV pose at ping time t
    auvPoseCovariance(auv_poses, mbes_pcl.header.stamp);

    // TODO: make meas noise model dependent on points pose along the ping
    points_cov_vec_ = std::vector<Eigen::Matrix3f> (mbes_pcl_.size(), Omega_mbes);

}

MbesPing::~MbesPing(){}


void MbesPing::auvPoseCovariance(const std::deque<nav_msgs::Odometry>& auv_poses, double time_t){
    // Find AUV pose at MBES ping time t (or closes time)
    nav_msgs::Odometry auv_pose_now;
    auv_pose_now.header.stamp.sec = time_t*10;
    for(nav_msgs::Odometry auv_pose_t: auv_poses){
        if(std::abs(auv_pose_t.header.stamp.sec  - time_t) < std::abs(auv_pose_now.header.stamp.sec - time_t)){
            auv_pose_now = auv_pose_t;
        }
    }

    auv_base_cov_ = Eigen::MatrixXf(6,6);
    // Extract the pose cov of the AUV at ping time t
    for(unsigned int i = 0; i < 6; i++) {
      for(unsigned int j = 0; j < 6; j++) {
        auv_base_cov_(i,j) = auv_pose_now.pose.covariance[i*6 + j];
      }
    }
}
