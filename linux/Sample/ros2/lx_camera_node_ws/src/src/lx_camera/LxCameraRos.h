//
// Created by root on 3/14/23.
//

#ifndef _LX_CAMERAM_ROS_H_
#define _LX_CAMERAM_ROS_H_

#include "../DynamicLink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "lx_camera_ros/msg/frame_rate.hpp"
#include "lx_camera_ros/msg/obstacle.hpp"
#include "lx_camera_ros/msg/pallet.hpp"
#include "lx_camera_ros/msg/result.hpp"

#include "lx_camera_ros/srv/lx_string.hpp"
#include "lx_camera_ros/srv/lx_float.hpp"
#include "lx_camera_ros/srv/lx_bool.hpp"
#include "lx_camera_ros/srv/lx_cmd.hpp"
#include "lx_camera_ros/srv/lx_int.hpp"

class LxCamera : public rclcpp::Node {
public:
    LxCamera(DcLib* _lib);
    ~LxCamera();
    void run();
private:
	void SearchAndOpenDevice();
	int Check(std::string command, LX_STATE state);

private:
	void SetParam();
	void ReadParam();

private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_error;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_amp;
	rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_tf;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_tof_info;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_rgb_info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pub_cloud;
    rclcpp::Publisher<lx_camera_ros::msg::FrameRate>::SharedPtr pub_temper;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_location;
    rclcpp::Publisher<lx_camera_ros::msg::Obstacle>::SharedPtr pub_obstacle;
    rclcpp::Publisher<lx_camera_ros::msg::Pallet>::SharedPtr pub_pallet;
	
	rmw_qos_profile_t qos;
    std::string log_path;
    std::string ip = "0";
	int raw_param = 0;

	int lx_2d_binning;
	int lx_2d_undistort;
	int lx_2d_undistort_scale;
	int lx_2d_auto_exposure;
	int lx_2d_auto_exposure_value;
	int lx_2d_exposure;
	int lx_2d_gain;

	int lx_rgb_to_tof;
	int lx_3d_binning;
	int lx_mulit_mode;
	int lx_3d_undistort;
	int lx_3d_undistort_scale;
	int lx_hdr;
	int lx_3d_auto_exposure;
	int lx_3d_auto_exposure_value;
	int lx_3d_first_exposure;
	int lx_3d_second_exposure;
	int lx_3d_gain;

	int lx_tof_unit;
	int lx_min_depth;
	int lx_max_depth;
	int lx_work_mode;
};

#endif //_LX_CAMERA_ROS_H_
