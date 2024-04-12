//
// Created by root on 3/14/23.
//

#ifndef _LX_CAMERA_ROS_H_
#define _LX_CAMERA_ROS_H_

#include <tf/tf.h>
#include <ros/ros.h>
#include "lx_camera_api.h"
#include "lx_camera_define.h"
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "lx_camera_application.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>


class LxCamera {
public:
    LxCamera();
    ~LxCamera();
    void Run();

private:
    void MappingCallBack(const std_msgs::String::ConstPtr& msg);
    void LocationCallBack(const std_msgs::String::ConstPtr& msg);
    void SetParamCallBack(const std_msgs::String::ConstPtr& msg);
    void SwitchMapCallBack(const std_msgs::String::ConstPtr& msg);
    void UploadMapCallBack(const std_msgs::String::ConstPtr& msg);
    void DownloadMapCallBack(const std_msgs::String::ConstPtr& msg);

    void UploadOdomCallBack(const nav_msgs::Odometry& msg);
    void UploadScanCallBack(const sensor_msgs::LaserScan& msg);
    void UploadLaserPoseCallBack(const geometry_msgs::PoseStamped& msg);
    void UploadRelocCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg);

private:
    //publish
	ros::Publisher rgb_info_publisher_;
	ros::Publisher app_info_publisher_;  //输出定位信息
	image_transport::Publisher rgb_publisher_;
	
    //subscribe
    ros::Subscriber mapping_subsciber_;
    ros::Subscriber location_subsciber_;
    ros::Subscriber set_param_subsciber_;
    ros::Subscriber switch_map_subsciber_;
    ros::Subscriber upload_map_subsciber_;
    ros::Subscriber upload_scan_subsciber_;
    ros::Subscriber upload_laserpose_subsciber_;
    ros::Subscriber download_map_subsciber_;
    ros::Subscriber upload_odom_subsciber_;
    ros::Subscriber upload_reloc_subsciber_;
    
    bool is_show_;
    char* log_path_;
    sensor_msgs::CameraInfo rgb_camera_info_;
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
