#ifndef _LX_CAMERA_H_
#define _LX_CAMERA_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common_headers.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 自定义包含时间戳的点云类型（与ROS2实现保持一致）
struct PointXYZIT
{
  PCL_ADD_POINT4D
  uint32_t intensity;
  double timestamp;
  uint16_t row_pos;
  uint16_t col_pos;
  PointXYZIT() : intensity(0), timestamp(0.0), row_pos(0), col_pos(0) {}
  PointXYZIT(float x, float y, float z, uint32_t i, double t)
      : intensity(i), timestamp(t), row_pos(0), col_pos(0)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  PointXYZIT(float x, float y, float z, uint32_t i, double t, uint16_t r, uint16_t c)
      : intensity(i), timestamp(t), row_pos(r), col_pos(c)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 注册自定义点云类型
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint32_t, intensity, intensity)
  (double, timestamp, timestamp)
  (std::uint16_t, row_pos, row_pos)
  (std::uint16_t, col_pos, col_pos)
)

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

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <atomic>
#include <chrono>


class LxCamera {
public:
  LxCamera();
  ~LxCamera();
  bool SearchAndOpenDevice(std::string ip);
  int Start();
  int Stop();
  void Run();

private:
  int Check(std::string Command, int state);
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
  void PubTf(const tf::Transform &transform, const std::string &frame_id,
             const std::string &child_frame_id);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr BuildPointCloudXYZRGB(
      float *xyz_data, uint8_t *rgb_data, long buff_len);
  pcl::PointCloud<pcl::PointXYZ>::Ptr BuildPointCloudXYZ(
      float *xyz_data, long buff_len);
  pcl::PointCloud<PointXYZIT>::Ptr BuildPointCloudXYZIT(
      const LxPointCloudData* data);

private:
  image_transport::Publisher pub_rgb_;
  image_transport::Publisher pub_depth_;
  image_transport::Publisher pub_amp_;
  ros::Publisher pub_tof_info_;
  ros::Publisher pub_rgb_info_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_lidarCloud_;
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

  int is_depth_ = 0;
  int is_amp_ = 0;
  int is_rgb_ = 0;
  int is_xyz_ = 1;
  int rgb_type_ = 0;
  int inside_app_ = 0;
  int rgb_channel_ = 0;
  int lx_rgbd_align = 0;
  float install_x_ = 0.0, install_y_ = 0.0, install_z_ = 0.0,
        install_yaw_ = 0.0, install_roll_ = 0.0, install_pitch_ = 0.0;
};

#endif //_LX_CAMERA_H_
