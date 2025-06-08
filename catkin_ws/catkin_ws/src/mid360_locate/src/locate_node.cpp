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
#include <pcl/features/normal_3d.h>    // 法线估计依赖
#include <tf2_eigen/tf2_eigen.h>

#include "locate_node.h"

// 全局变量定义
bool inited = false;
Eigen::Matrix4f veh_pose = Eigen::Matrix4f::Identity();
pcl::PointCloud<pcl::PointXYZ>::Ptr points_map(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 cloud_msg;
nav_msgs::Path path_msg;
ros::Publisher points_pub;
ros::Publisher pose_pub;
ros::Publisher path_pub;
ros::Time t_msg;
bool received_initial_pose = false;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!received_initial_pose) {
        // 从消息中提取位置
        veh_pose(0, 3) = msg->pose.pose.position.x;
        veh_pose(1, 3) = msg->pose.pose.position.y;
        veh_pose(2, 3) = 1.3;

        // 从消息中提取姿态（四元数）
        Eigen::Quaterniond q;
        tf2::fromMsg(msg->pose.pose.orientation, q);

        // 将两个旋转组合起来
        Eigen::Quaternionf q_final = q.cast<float>();

        // 将四元数转换为旋转矩阵
        veh_pose.topLeftCorner(3, 3) = q_final.toRotationMatrix();

        received_initial_pose = true;
        ROS_INFO("Received initial pose from RViz");
    }
}

void pub_path(Eigen::Matrix4f &pose) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp    = t_msg;
    pose_msg.header.frame_id = "map";
    
    // 转换位置
    Eigen::Vector3d position = pose.block<3, 1>(0, 3).cast<double>();
    pose_msg.pose.position = tf2::toMsg(position);
    
    // 转换姿态
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0).cast<double>());
    pose_msg.pose.orientation = tf2::toMsg(quat);
    
    pose_pub.publish(pose_msg);

    // 发布机器人轨迹
    path_msg.header.stamp    = t_msg;
    path_msg.header.frame_id = "map";
    path_msg.poses.push_back(pose_msg);
    path_pub.publish(path_msg);

}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_map, pcl::PointXYZ &center_point, float thr) {
    // 创建一个新的点云，用于存储剪裁后的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>);

    // 遍历所有点，将距离小于25米的点添加到剪裁后的点云中
    for (const auto &point : *pcl_map) {
        double distance = pcl::euclideanDistance(center_point, point);
        if (distance <= thr) {
            local_map->push_back(point);
        }
    }
    return local_map;
};

void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    if (!inited) {
        inited = true;

        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

        // 设置要匹配的目标点云
        ndt.setInputSource(points);
        ndt.setInputTarget(local_map);

        ndt.setTransformationEpsilon(1e-8);
        ndt.setStepSize(0.1);
        ndt.setMaximumIterations(30);
        ndt.setResolution(1.0);

        ndt.align(*aligned, veh_pose);

        veh_pose = ndt.getFinalTransformation();
        ROS_INFO(
            "veh pose init finish: iter: %d x: %f y: %f!", ndt.getFinalNumIteration(), veh_pose(0, 3), veh_pose(1, 3));

    } else {
        fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(points);
        gicp.setInputTarget(local_map);
        gicp.setNumThreads(8);
        gicp.setMaxCorrespondenceDistance(0.5);
        gicp.align(*aligned, veh_pose);
        Eigen::Matrix4f gicp_tf_matrix;
        veh_pose = gicp.getFinalTransformation();
    }

    sensor_msgs::PointCloud2 points_msg; 
    pcl::toROSMsg(*aligned, points_msg);
    points_msg.header.stamp    = t_msg;
    points_msg.header.frame_id = "map";

    points_pub.publish(points_msg);

    pub_path(veh_pose);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr normalSpaceSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int target_points) {
    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);

    // 创建带法向量的点云
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // 法向量空间采样
    pcl::PointCloud<pcl::PointNormal>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
    nss.setInputCloud(cloud_with_normals);
    nss.setNormals(cloud_with_normals);
    nss.setBins(8, 8, 8);  // 法向量空间的分箱数
    nss.setSample(target_points);
    nss.filter(*sampled_cloud);

    // 转换回普通点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*sampled_cloud, *result);
    
    return result;
}

void pointCloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr &points_filtered) {
    auto start = std::chrono::high_resolution_clock::now();

    points_filtered = normalSpaceSampling(points_filtered, 5000);

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>);
    auto center = pcl::PointXYZ(veh_pose(0, 3), veh_pose(1, 3), veh_pose(2, 3));

    local_map   = get_local_map(points_map, center, 25.f);

    auto end0 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed0 = end0 - start;

    registration(points_filtered, local_map);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO("Localization Time1:  %.2f[ms], %.2f[ms], %ld points, %ld local map points", 
             elapsed.count() * 1e3, elapsed0.count() * 1e3, 
             points_filtered->size(), local_map->points.size());
}

void livox_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (uint i = 1; i < msg->point_num; i++) {
        auto &pt_i = msg->points.at(i);
        double distance2 = pt_i.x * pt_i.x + pt_i.y * pt_i.y;
        if (distance2 < distance2_min_thr || distance2 > distance2_local_map_thr) {
            continue;
        }
        pcl::PointXYZ pt(pt_i.x, -pt_i.y, -pt_i.z);
        points_filtered->emplace_back(pt);
    }
    t_msg = msg->header.stamp;
    pointCloudProcess(points_filtered);
}

void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &pt : *points) {
        double distance2 = pt.x * pt.x + pt.y * pt.y;
        if (distance2 < distance2_min_thr || distance2 > distance2_local_map_thr) {
            continue;
        }
        points_filtered->push_back(pt);
    }
    t_msg = msg->header.stamp;
    pointCloudProcess(points_filtered);
}

void pointCloudPublisherThread(ros::Publisher &pub) {
    // 发布点云消息
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pub.publish(cloud_msg);

        loop_rate.sleep();
    }
}

void initializeMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& points_map, const std::string& map_file_path, float map_yaw, float map_pitch, float map_roll) {
    ROS_INFO("Reading map file: %s.", map_file_path.c_str());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_path, *points_map) == -1) {
        PCL_ERROR("Couldn't read file.\n");
        return;
    }

    ROS_INFO("Original point cloud size: %zu points.", points_map->size());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(points_map);
    sor.setLeafSize(0.2f, 0.2f, 0.2f); // 设置体素大小
    sor.filter(*points_map);           // 执行滤波
    
    ROS_INFO("Filtered point cloud size: %zu points.", points_map->size());

    ROS_INFO("Find the plane...");
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.1);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setInputCloud(points_map); 
    seg.segment(inliers, coefficients);

    ROS_INFO(
        "Plane coefficients: %f %f %f %f",
        coefficients.values[0],
        coefficients.values[1],
        coefficients.values[2],
        coefficients.values[3]);

    ROS_INFO("Rotate map to z plane.");
    // 已知平面的法线和水平面的法线
    Eigen::Vector3f plane_normal(
        coefficients.values[0], coefficients.values[1], coefficients.values[2]); // 假设平面的法线向量
    plane_normal.normalize();                                                    // 需要对法线向量进行归一化
    Eigen::Vector3f horizontal_normal(0.0, 0.0, 1.0); // 假设水平面的法线向量为(0, 0, 1)
    // 计算旋转矩阵
    Eigen::Quaternionf rotation     = Eigen::Quaternionf::FromTwoVectors(plane_normal, horizontal_normal);
    Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix4f T_to_plane       = Eigen::Matrix4f::Identity();
    T_to_plane(2, 3)                 = coefficients.values[3];
    T_to_plane.topLeftCorner<3, 3>() = rotation_matrix;
    int target_points = points_map->points.size();
    points_map = normalSpaceSampling(points_map, target_points/2);
    pcl::transformPointCloud(*points_map, *points_map, T_to_plane);

    ROS_INFO("Rotate map's yaw.");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(
        Eigen::AngleAxisf(map_yaw, Eigen::Vector3f::UnitZ()) *   //
        Eigen::AngleAxisf(map_pitch, Eigen::Vector3f::UnitY()) * //
        Eigen::AngleAxisf(map_roll, Eigen::Vector3f::UnitX()));

    // 应用旋转变换到点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*points_map, *points_map, transform);

    ROS_INFO("Map init finish.");
}
