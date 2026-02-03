#ifndef _LX_CAMERAM_ROS_H_
#define _LX_CAMERAM_ROS_H_

#include <pcl/common/common_headers.h>
#include "utils/dynamic_link.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "lx_camera_ros/msg/frame_rate.hpp"
#include "lx_camera_ros/msg/obstacle.hpp"
#include "lx_camera_ros/msg/pallet.hpp"
#include "lx_camera_ros/msg/result.hpp"

#include "lx_camera_ros/srv/lx_bool.hpp"
#include "lx_camera_ros/srv/lx_cmd.hpp"
#include "lx_camera_ros/srv/lx_float.hpp"
#include "lx_camera_ros/srv/lx_int.hpp"
#include "lx_camera_ros/srv/lx_string.hpp"

class LxCamera : public rclcpp::Node {
public:
  explicit LxCamera(DcLib *dynamic_lib);
  ~LxCamera();
  bool SearchAndOpenDevice();
  int Start();
  int Stop();
  void Run();

private:
  int Check(std::string command, int state);
  void SetParam();  // Write configuration parameters to the camera
  void ReadParam(); // Read configuration parameters
  bool LxString(const lx_camera_ros::srv::LxString::Request::SharedPtr req,
                const lx_camera_ros::srv::LxString::Response::SharedPtr res);
  bool LxFloat(const lx_camera_ros::srv::LxFloat::Request::SharedPtr req,
               const lx_camera_ros::srv::LxFloat::Response::SharedPtr res);
  bool LxBool(const lx_camera_ros::srv::LxBool::Request::SharedPtr req,
              const lx_camera_ros::srv::LxBool::Response::SharedPtr res);
  bool LxCmd(const lx_camera_ros::srv::LxCmd::Request::SharedPtr req,
             const lx_camera_ros::srv::LxCmd::Response::SharedPtr res);
  bool LxInt(const lx_camera_ros::srv::LxInt::Request::SharedPtr req,
             const lx_camera_ros::srv::LxInt::Response::SharedPtr res);
  void PubTf(const geometry_msgs::msg::TransformStamped &transform_stamped);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_error_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_amp_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_tf_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_tof_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_rgb_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<lx_camera_ros::msg::FrameRate>::SharedPtr pub_temper_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_location_;
  rclcpp::Publisher<lx_camera_ros::msg::Obstacle>::SharedPtr pub_obstacle_;
  rclcpp::Publisher<lx_camera_ros::msg::Pallet>::SharedPtr pub_pallet_;

  geometry_msgs::msg::TransformStamped tf_;
  sensor_msgs::msg::CameraInfo tof_info_;
  sensor_msgs::msg::CameraInfo rgb_info_;

  DcHandle handle_;
  bool is_start_ = 0;

  rmw_qos_profile_t qos_;
  std::string log_path_;
  std::string ip_ = "0";
  int raw_param_ = 0;

  int lx_2d_binning_;
  int lx_2d_undistort_;
  int lx_2d_undistort_scale_;
  int lx_2d_auto_exposure_;
  int lx_2d_auto_exposure_value_;
  int lx_2d_exposure_;
  int lx_2d_gain_;

  int lx_rgb_to_tof_;
  int lx_3d_binning_;
  int lx_mulit_mode_;
  int lx_3d_undistort_;
  int lx_3d_undistort_scale_;
  int lx_hdr_;
  int lx_3d_auto_exposure_;
  int lx_3d_auto_exposure_value_;
  int lx_3d_first_exposure_;
  int lx_3d_second_exposure_;
  int lx_3d_gain_;

  int lx_tof_unit_;
  int lx_min_depth_;
  int lx_max_depth_;
  int lx_work_mode_;
  int is_depth_ = 0;
  int is_amp_ = 0;
  int is_rgb_ = 0;
  int is_xyz_ = 1;
  int rgb_type_ = 0;
  int inside_app_ = 0;
  int rgb_channel_ = 0;
  float install_x_ = 0.0, install_y_ = 0.0, install_z_ = 0.0,
        install_yaw_ = 0.0, install_roll_ = 0.0, install_pitch_ = 0.0;
};

#endif //_LX_CAMERA_ROS_H_
