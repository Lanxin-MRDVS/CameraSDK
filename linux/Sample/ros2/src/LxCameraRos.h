//
// Created by root on 3/14/23.
//

#ifndef _LX_CAMERA_ROS_H_
#define _LX_CAMERA_ROS_H_

#include "lx_camera_api.h"
#include "lx_camera_application.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <image_transport/image_transport.h>
typedef rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Pub_Str;
typedef rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr Pub_CInfo;
typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr Pub_PC;
typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Pub_PC2;
typedef rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr Pub_Img;
typedef rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Sub_Img;
typedef rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Sub_Str;

class LxCamera : public rclcpp::Node {
public:
	LxCamera();
    ~LxCamera();
    void run();
	int Check(std::string command, LX_STATE state);
	void LxCamera_Error(const std_msgs::msg::String::SharedPtr msg);
	void LxCamera_Command(const std_msgs::msg::String::SharedPtr msg);
	void LxCamera_rgb(sensor_msgs::msg::Image::SharedPtr msg);
	void LxCamera_dep(sensor_msgs::msg::Image::SharedPtr msg);
	void LxCamera_amp(sensor_msgs::msg::Image::SharedPtr msg);

private:
	static void LxCamera_Command(const std_msgs::msg::String::SharedPtr& msg);
private:
	Pub_Str pub_app_info;
	Pub_CInfo pub_tof_info;
	Pub_CInfo pub_rgb_info;
    Pub_PC pub_cloud;

    Pub_Img pub_rgb;
    Pub_Img pub_depth;
    Pub_Img pub_amp;
    Pub_PC2 pub_xyz;

	sensor_msgs::msg::CameraInfo tof_camera_info;
	sensor_msgs::msg::CameraInfo rgb_camera_info;
	
	int is_xyz = 1;
	int is_rgb = 0;
	int is_amp = 0;
	int is_show = 0;
	int is_depth = 0;
	int	inside_app = 0;

	rmw_qos_profile_t qos;
};

#endif //_LX_CAMERA_ROS_H_

