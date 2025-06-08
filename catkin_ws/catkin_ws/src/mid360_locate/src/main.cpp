#include "locate_node.h"
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "locate_node");
    ros::NodeHandle nh("~");

    int points_type;
    std::string map_file_path, topic_name;
    nh.param<std::string>("map_file_path", map_file_path, ros::package::getPath("fast_lio") + "/PCD/map.pcd");
    nh.param<int>("points_type", points_type, 0); // 0: livox, 1:pointcloud2
    nh.param<std::string>("topic_name", topic_name, "/livox/lidar");

    float map_yaw, map_roll, map_pitch;
    nh.param<float>("map_roll", map_roll, 0.0f);
    nh.param<float>("map_pitch", map_pitch, 0.0f);
    nh.param<float>("map_yaw", map_yaw, 0.0f);

    // 订阅 initialpose 话题
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 10, initialPoseCallback);
    ROS_INFO("Waiting for initial pose from RViz...");

    // 初始化地图
    initializeMap(points_map, map_file_path, map_yaw, map_pitch, map_roll);

    // 将pcl点云转换为ROS消息
    pcl::toROSMsg(*points_map, cloud_msg);
    // 设置消息的元数据
    cloud_msg.header.stamp    = t_msg;
    cloud_msg.header.frame_id = "map";

    ros::Subscriber points_sub;
    if (points_type == 0) {
        points_sub = nh.subscribe(topic_name, 1, livox_callback);
    } else {
        points_sub = nh.subscribe(topic_name, 1, pcl_callback);
    }
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned", 1);
    pose_pub   = nh.advertise<geometry_msgs::PoseStamped>("/veh_pose", 1);
    path_pub   = nh.advertise<nav_msgs::Path>("/path", 1);
    
    // 创建TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster;
    
    // 创建odometry发布器
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/map", 1);

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_map", 1);
    std::thread publisher_thread(pointCloudPublisherThread, std::ref(map_pub));

    // 等待直到收到初始位姿
    while (ros::ok() && !received_initial_pose) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    while (ros::ok()) {
        // 发布从map到camera_init的TF变换
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        
        // 使用实际位姿作为变换
        transform.transform.translation.x = veh_pose(0, 3);
        transform.transform.translation.y = veh_pose(1, 3);
        transform.transform.translation.z = veh_pose(2, 3);
        
        // 将旋转矩阵转换为四元数
        Eigen::Quaterniond rotation_quat(veh_pose.block<3, 3>(0, 0).cast<double>());
        Eigen::Quaterniond x_180_deg = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond final_quat = rotation_quat * x_180_deg;
        transform.transform.rotation = tf2::toMsg(final_quat);
        
        // 发布TF变换
        tf_broadcaster.sendTransform(transform);
        
        // 发布odometry消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        // 设置位置
        odom_msg.pose.pose.position.x = veh_pose(0, 3);
        odom_msg.pose.pose.position.y = veh_pose(1, 3);
        odom_msg.pose.pose.position.z = veh_pose(2, 3);
        
        // 设置方向
        odom_msg.pose.pose.orientation = tf2::toMsg(final_quat);
        
        // 发布odometry消息
        odom_pub.publish(odom_msg);
        
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ros::spin();

    return 0;
} 