//
// Created by root on 3/14/23.
//

#ifndef _LX_CAMERA_ROS_H_
#define _LX_CAMERA_ROS_H_

#include <ros/ros.h>
#include "lx_camera_api.h"
#include <std_msgs/Bool.h>
#include "lx_camera_define.h"
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "lx_camera_application.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>

#include "lx_camera_ros/FrameRate.h"
#include "lx_camera_ros/Obstacle.h"
#include "lx_camera_ros/Pallet.h"
#include "lx_camera_ros/Result.h"

#include "lx_camera_ros/LxString.h"
#include "lx_camera_ros/LxFloat.h"
#include "lx_camera_ros/LxBool.h"
#include "lx_camera_ros/LxCmd.h"
#include "lx_camera_ros/LxInt.h"

class LxCamera {
public:
    LxCamera();
    ~LxCamera();
    void run();
private:
	void SearchAndOpenDevice();
	int Check(std::string Command, LX_STATE state);
	bool LxString(lx_camera_ros::LxString::Request& req, lx_camera_ros::LxString::Response& res);
	bool LxFloat(lx_camera_ros::LxFloat::Request& req, lx_camera_ros::LxFloat::Response& res);
	bool LxBool(lx_camera_ros::LxBool::Request& req, lx_camera_ros::LxBool::Response& res);
	bool LxCmd(lx_camera_ros::LxCmd::Request& req, lx_camera_ros::LxCmd::Response& res);
	bool LxInt(lx_camera_ros::LxInt::Request& req, lx_camera_ros::LxInt::Response& res);

private:
	void SetParam();
	void ReadParam();

private:
    image_transport::Publisher pub_rgb;
    image_transport::Publisher pub_depth;
    image_transport::Publisher pub_amp;
	ros::Publisher pub_tof_info;
	ros::Publisher pub_rgb_info;
    ros::Publisher pub_cloud;
    ros::Publisher pub_temper;
    ros::Publisher pub_location;
    ros::Publisher pub_obstacle;
    ros::Publisher pub_pallet;
	ros::Publisher pub_app_info;
	ros::Publisher pub_error;

	ros::NodeHandle* nh;
    std::string log_path;
	int raw_param = 0;
    std::string ip;

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

	float x,y,z,yaw,roll,pitch;
};

#endif //_LX_CAMERA_ROS_H_
