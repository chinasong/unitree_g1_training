#ifndef LOCATE_NODE_H
#define LOCATE_NODE_H

#include <string>
#include <thread>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <pcl/registration/ndt.h>

#include <Eigen/src/Core/Matrix.h>

#include "livox_ros_driver/CustomMsg.h"
#include "fast_gicp/gicp/fast_gicp.hpp" 
#include "ros/console.h"
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <tf2_eigen/tf2_eigen.h>

// 全局常量
constexpr float distance2_min_thr = 0.25f;
constexpr float distance2_local_map_thr = 625.f;

// 全局变量
extern bool inited;
extern Eigen::Matrix4f veh_pose;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr points_map;
extern sensor_msgs::PointCloud2 cloud_msg;
extern nav_msgs::Path path_msg;
extern ros::Publisher points_pub;
extern ros::Publisher pose_pub;
extern ros::Publisher path_pub;
extern ros::Time t_msg;
extern bool received_initial_pose;

// 函数声明
pcl::PointCloud<pcl::PointXYZ>::Ptr normalSpaceSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int target_points);
void initializeMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& points_map, const std::string& map_file_path, float map_yaw, float map_pitch, float map_roll);
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void pub_path(Eigen::Matrix4f &pose, bool write_file);
pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_map, pcl::PointXYZ &center_point, float thr);
void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map);
void pointCloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr &points_filtered);
void livox_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg);
void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
void pointCloudPublisherThread(ros::Publisher &pub);

#endif // LOCATE_NODE_H 