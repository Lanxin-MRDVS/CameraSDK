#ifndef _LX_LOCALIZATION_H_
#define _LX_LOCALIZATION_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "lx_camera_api.h"
#include "lx_camera_application.h"
#include "lx_camera_define.h"

class LxLocalization {
public:
  LxLocalization();
  ~LxLocalization();
  void Run();

private:
  void ErrorCallBack(const std_msgs::String::ConstPtr &msg);
  void MessageCallBack(const std_msgs::String::ConstPtr &msg);
  void CommandCallBack(const std_msgs::String::ConstPtr &msg);
  void MappingCallBack(const std_msgs::String::ConstPtr &msg);
  void LocationCallBack(const std_msgs::String::ConstPtr &msg);
  void SetParamCallBack(const std_msgs::String::ConstPtr &msg);
  void SwitchMapCallBack(const std_msgs::String::ConstPtr &msg);
  void UploadMapCallBack(const std_msgs::String::ConstPtr &msg);
  void DownloadMapCallBack(const std_msgs::String::ConstPtr &msg);
  void UploadOdomCallBack(const nav_msgs::Odometry &msg);
  void UploadScanCallBack(const sensor_msgs::LaserScan &msg);
  void UploadLaserPoseCallBack(const geometry_msgs::PoseStamped &msg);
  void UploadRelocCallBack(const geometry_msgs::PoseWithCovarianceStamped &msg);
  int Check(std::string command, int state);

private:
  // publish
  ros::Publisher error_publisher_;
  ros::Publisher message_publisher_;
  ros::Publisher rgb_info_publisher_;
  ros::Publisher app_info_publisher_; //输出定位信息
  image_transport::Publisher rgb_publisher_;

  // subscribe
  ros::Subscriber lsg_subscriber_;
  ros::Subscriber err_subscriber_;
  ros::Subscriber com_subscriber_;
  ros::Subscriber mapping_subscriber_;
  ros::Subscriber location_subscriber_;
  ros::Subscriber set_param_subscriber_;
  ros::Subscriber switch_map_subscriber_;
  ros::Subscriber upload_map_subscriber_;
  ros::Subscriber upload_scan_subscriber_;
  ros::Subscriber upload_laserpose_subscriber_;
  ros::Subscriber download_map_subscriber_;
  ros::Subscriber upload_odom_subscriber_;
  ros::Subscriber upload_reloc_subscriber_;

  bool is_show_ = false;
  DcHandle handle_ = 0;
  std::string ip_ = "";
  std::string log_path_ = "";
  sensor_msgs::CameraInfo rgb_camera_info_;
  int print_level_ = 0; // 0:info, 1:warn, 2:error
  bool enable_screen_print_ = true;
  int auto_exposure_value_ = 50;
  bool mapping_mode_ = false;
  bool localization_mode_ = false;
  std::string map_name_ = "example_map1";
  std::vector<double> camera_extrinsic_param_{
      0, 0, 0, 0, 0, 0}; // [x, y, z, yaw, pitch, roll]
  std::vector<double> laser_extrinsic_param_{0, 0, 0}; // [x, y, yaw]
  std::string algo_ver_ = "0.0.0";
};

#endif //_LX_LOCALIZATION_H_
