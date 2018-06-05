#ifndef MBES_RECEPTOR_HPP
#define MBES_RECEPTOR_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <tuple>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <queue>
#include <fstream>
#include <iostream>

#include <mbes_ping/mbes_ping.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class MBESReceptor{

public:
    MBESReceptor(ros::NodeHandle &nh, std::string node_name);
    ~MBESReceptor();

private:
    std::string node_name_;
    ros::NodeHandle* nh_;

    ros::Publisher pcl_pub_;
    ros::Publisher landmark_pub_;
    ros::Subscriber mbes_laser_sub_;
    ros::Subscriber auv_pose_sub_;

    std::string mbes_frame_;
    std::string base_frame_;
    std::string map_frame_;

    tf::StampedTransform tf_base_mbes_;
    tf::StampedTransform tf_base_map_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster submaps_bc_;

    PointCloud::Ptr pcl_msg_;
    laser_geometry::LaserProjection projector_;

    // Aux
    unsigned int pcl_cnt_;
    int submap_size_;
    Eigen::Matrix3f Omega_mbes_;    // Information matrix of meas noise model
    std::deque<nav_msgs::Odometry> auv_poses_;

    // Submaps
    std::vector<MbesPing> mbes_swath_;
    std::vector<tf::Transform> tf_map_meas_vec_;

    void MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);

    void bcMapSubmapsTF(std::vector<tf::Transform> tfs_meas_map);

    void pclFuser();

    // Aux methods
    void savePointCloud(PointCloud submap_pcl, std::string file_name);

    void auvPoseCB(const nav_msgs::Odometry auv_pose_t);

    void init(std::vector<double> q_mbes_diag);

};

#endif // MBES_RECEPTOR_HPP
