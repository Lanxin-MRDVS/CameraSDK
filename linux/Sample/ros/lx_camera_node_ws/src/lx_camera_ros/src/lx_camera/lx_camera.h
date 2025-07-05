#ifndef _LX_CAMERA_H_
#define _LX_CAMERA_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "lx_camera_ros/FrameRate.h"
#include "lx_camera_ros/Obstacle.h"
#include "lx_camera_ros/Pallet.h"
#include "lx_camera_ros/Result.h"

#include "lx_camera_ros/LxBool.h"
#include "lx_camera_ros/LxCmd.h"
#include "lx_camera_ros/LxFloat.h"
#include "lx_camera_ros/LxInt.h"
#include "lx_camera_ros/LxString.h"

#include "lx_camera_api.h"
#include "lx_camera_application.h"
#include "lx_camera_define.h"

class LxCamera {
public:
  LxCamera();
  ~LxCamera();
  bool SearchAndOpenDevice();
  int Start();
  int Stop();
  void Run();

private:
  int Check(std::string Command, int state);
  void SetParam();  //向相机写入配置参数
  void ReadParam(); //读取配置参数
  bool LxString(lx_camera_ros::LxString::Request &req,
                lx_camera_ros::LxString::Response &res);
  bool LxFloat(lx_camera_ros::LxFloat::Request &req,
               lx_camera_ros::LxFloat::Response &res);
  bool LxBool(lx_camera_ros::LxBool::Request &req,
              lx_camera_ros::LxBool::Response &res);
  bool LxCmd(lx_camera_ros::LxCmd::Request &req,
             lx_camera_ros::LxCmd::Response &res);
  bool LxInt(lx_camera_ros::LxInt::Request &req,
             lx_camera_ros::LxInt::Response &res);
  void PubTf(const tf::Transform &transform, const std::string &frame_id, const std::string &child_frame_id);

private:
  image_transport::Publisher pub_rgb_;
  image_transport::Publisher pub_depth_;
  image_transport::Publisher pub_amp_;
  ros::Publisher pub_tof_info_;
  ros::Publisher pub_rgb_info_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_temper_;
  ros::Publisher pub_location_;
  ros::Publisher pub_obstacle_;
  ros::Publisher pub_pallet_;
  ros::Publisher pub_app_info_;
  ros::Publisher pub_error_;

  sensor_msgs::CameraInfo tof_camera_info_;
  sensor_msgs::CameraInfo rgb_camera_info_;

  DcHandle handle_ = 0;
  bool is_start_ = 0;
  ros::NodeHandle *nh_;
  std::string log_path_;
  int raw_param_ = 0;
  std::string ip_ = "";

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

  std::string tof_frame_id_ = ""; // Configurable frame_id for TOF data (set via ROS param)
  std::string rgb_frame_id_ = ""; // Configurable frame_id for RGB data (set via ROS param)
  std::string intrinsic_depth_frame_id_ = ""; // Configurable intrinsic frame_id for depth (set via ROS param)
  std::string intrinsic_rgb_frame_id_ = ""; // Configurable intrinsic frame_id for RGB (set via ROS param)
};

#endif //_LX_CAMERA_H_
