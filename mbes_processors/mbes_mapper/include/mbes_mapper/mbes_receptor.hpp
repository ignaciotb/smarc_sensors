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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mbes_mapper/PoseArrayWithCovariances.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

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
    ros::Publisher pcl_poses_pub_;
    ros::Publisher landmark_pub_;
    ros::Publisher vis_pub_;
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
    Eigen::Matrix3d Omega_mbes_;    // Information matrix of meas noise model
    std::deque<nav_msgs::Odometry> auv_poses_;

    // Submaps
    std::vector<MbesPing> mbes_swath_;
    std::vector<tf::Transform> tf_map_meas_vec_;

    void MBESLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);

    void bcMapSubmapsTF(std::vector<tf::Transform> tfs_meas_map);

    void transformPCLCovariances(MbesPing &ping_i, const tf::Transform &tf_submap_baset);

    void submapBuilder(std::vector<MbesPing> mbes_swath);

    // Aux methods
    void savePointCloud(PointCloud submap_pcl, std::string file_name);

    void auvPoseCB(const nav_msgs::Odometry auv_pose_t);

    void pubPCLPosesWithCov(std::string meas_frame,
                            const PointCloud& ping_pcl_transformed,
                            const MbesPing& ping,
                            visualization_msgs::MarkerArray& marker_array, unsigned int pings_cnt);

    void init(std::vector<double> q_mbes_diag);

};

#endif // MBES_RECEPTOR_HPP
