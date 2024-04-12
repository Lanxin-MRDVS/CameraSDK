//
// Created by root on 3/14/23.
//

#ifndef _LX_CAMERA_ROS_H_
#define _LX_CAMERA_ROS_H_

#include "../DynamicLink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "image_transport/image_transport.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


class Location : public rclcpp::Node {
public:
    Location(DcLib* lib);
    ~Location();
    void Run();
    void ErrorCallBack(const std_msgs::msg::String::SharedPtr msg);
    void MessageCallBack(const std_msgs::msg::String::SharedPtr msg);
    void MappingCallBack(const std_msgs::msg::String::SharedPtr msg);
    void LocationCallBack(const std_msgs::msg::String::SharedPtr msg);
    void SetParamCallBack(const std_msgs::msg::String::SharedPtr msg);
    void SwitchMapCallBack(const std_msgs::msg::String::SharedPtr msg);
    void UploadMapCallBack(const std_msgs::msg::String::SharedPtr msg);
    void DownloadMapCallBack(const std_msgs::msg::String::SharedPtr msg);
    void UploadOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void UploadScanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void UploadLaserPoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void UploadRelocCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
	int Check(std::string command, LX_STATE state);

private:
    //publish
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_error;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_message;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_rgb_info;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr app_info_publisher_;  //输出定位信息
	
    //subscribe
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lsg_subsciber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr err_subsciber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapping_subsciber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr location_subsciber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_param_subsciber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_map_subsciber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr upload_map_subsciber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr download_map_subsciber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr upload_odom_subsciber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr upload_scan_subsciber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr upload_laserpose_subsciber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr upload_reloc_subsciber_;
    
	DcLib* lib;
    char* log_path_;
    DcHandle handle;
	rmw_qos_profile_t qos;
    sensor_msgs::msg::CameraInfo rgb_camera_info_;
    int print_level_ = 0;  //0:info, 1:warn, 2:error
    bool enable_screen_print_ = true;
    int auto_exposure_value_ = 50;
    bool mapping_mode_ = false;
    bool localization_mode_ = false;
    std::string map_name_ = "example_map1";
    std::vector<double> camera_extrinsic_param_{0, 0, 0, 0, 0, 0}; // [x, y, z, yaw, pitch, roll]
    std::vector<double> laser_extrinsic_param_{0, 0, 0};  // [x, y, yaw]
};

#endif //_LX_CAMERA_ROS_H_
