#ifndef MBES_PING_HPP
#define MBES_PING_HPP

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <eigen3/Eigen/Core>

#include <nav_msgs/Odometry.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MbesPing{

public:

    MbesPing(PointCloud& mbes_pcl, const tf::Transform& tf_base_map,
             const Eigen::Matrix3d &Omega_mbes, const std::deque<nav_msgs::Odometry>& auv_poses);
    ~MbesPing();

    PointCloud mbes_pcl_;
    tf::Transform tf_base_map_;
    Eigen::MatrixXd auv_base_cov_;
    std::vector<Eigen::Matrix3d> points_omega_vec_;
    std::vector<Eigen::Matrix3d> points_cov_vec_;

    void auvPoseCovariance(const std::deque<nav_msgs::Odometry> &auv_poses, double time_t);

};

#endif // MBES_PING_HPP
