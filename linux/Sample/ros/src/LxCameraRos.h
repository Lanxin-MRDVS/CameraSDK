//
// Created by root on 3/14/23.
//

#ifndef _LX_CAMERA_ROS_H_
#define _LX_CAMERA_ROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
class LxCamera {
public:
    LxCamera();
    ~LxCamera();
    void run();
private:
	static void LxCamera_Command(const std_msgs::String::ConstPtr& msg);
private:
	ros::Publisher pub_tof_info;
	ros::Publisher pub_rgb_info;
	ros::Publisher pub_app_info;
    ros::Publisher pub_cloud;

    image_transport::Publisher pub_rgb;
    image_transport::Publisher pub_depth;
    image_transport::Publisher pub_amp;
    image_transport::Publisher pub_xyz;

	sensor_msgs::CameraInfo tof_camera_info;
	sensor_msgs::CameraInfo rgb_camera_info;
	
	int is_xyz = 1;
	int is_depth = 0;
	int is_amp = 0;
	int is_rgb = 0;
	int	inside_app = 0;
};

#endif //_LX_CAMERA_ROS_H_
