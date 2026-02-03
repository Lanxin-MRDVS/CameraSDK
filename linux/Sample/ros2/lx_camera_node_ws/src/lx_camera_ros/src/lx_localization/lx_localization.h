#ifndef _LX_LOCALIZATION_ROS_H_
#define _LX_LOCALIZATION_ROS_H_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "utils/dynamic_link.h"

class LxLocalization : public rclcpp::Node {
public:
  LxLocalization(DcLib *dynamic_lib);
  ~LxLocalization();
  void Run();

private:
  void ErrorCallBack(const std_msgs::msg::String::SharedPtr msg);
  void MessageCallBack(const std_msgs::msg::String::SharedPtr msg);
  void CommandCallBack(const std_msgs::msg::String::SharedPtr &msg);

  void MappingCallBack(const std_msgs::msg::String::SharedPtr msg);
  void LocationCallBack(const std_msgs::msg::String::SharedPtr msg);
  void SetParamCallBack(const std_msgs::msg::String::SharedPtr msg);
  void SwitchMapCallBack(const std_msgs::msg::String::SharedPtr msg);
  void UploadMapCallBack(const std_msgs::msg::String::SharedPtr msg);
  void DownloadMapCallBack(const std_msgs::msg::String::SharedPtr msg);
  void UploadOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
  void UploadScanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void
  UploadLaserPoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void UploadRelocCallBack(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  int Check(std::string command, int state);

private:
  // publish
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr message_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      rgb_info_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      app_info_publisher_; // Publish localization info

  // subscribe
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lsg_subsciber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr err_subsciber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapping_subsciber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr location_subsciber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_param_subsciber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr switch_map_subsciber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr upload_map_subsciber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr download_map_subsciber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr upload_odom_subsciber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      upload_scan_subsciber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      upload_laserpose_subsciber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      upload_reloc_subsciber_;

  DcHandle handle_;
  bool is_show_ = false;
  rmw_qos_profile_t qos_;
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  int print_level_ = 0; // 0:info, 1:warn, 2:error
  bool enable_screen_print_ = true;
  std::string log_path_ = "";
  int auto_exposure_value_ = 50;
  bool mapping_mode_ = false;
  bool localization_mode_ = false;
  std::string map_name_ = "example_map1";
  std::vector<double> camera_extrinsic_param_{
      0, 0, 0, 0, 0, 0}; // [x, y, z, yaw, pitch, roll]
  std::vector<double> laser_extrinsic_param_{0, 0, 0}; // [x, y, yaw]
  std::string algo_ver_ = "0.0.0";
};

#endif //_LX_LOCALIZATION_ROS_H_
