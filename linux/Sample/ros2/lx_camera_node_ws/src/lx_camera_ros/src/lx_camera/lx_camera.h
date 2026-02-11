#ifndef _LX_CAMERAM_ROS_H_
#define _LX_CAMERAM_ROS_H_

#include "utils/dynamic_link.h"
#include <pcl/common/common_headers.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
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

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 自定义包含时间戳的点云类型
struct PointXYZIT
{
    PCL_ADD_POINT4D
    uint32_t intensity;
    double timestamp;
    uint16_t row_pos;
    uint16_t col_pos;
    PointXYZIT() : intensity(0), timestamp(0.0), row_pos(0), col_pos(0) {}
    PointXYZIT(float x, float y, float z, uint32_t i, double t) 
        : intensity(i), timestamp(t), row_pos(0), col_pos(0) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    PointXYZIT(float x, float y, float z, uint32_t i, double t, uint16_t r, uint16_t c)
        : intensity(i), timestamp(t), row_pos(r), col_pos(c) {
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

typedef PointXYZIT PointType;


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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidarCloud_;
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

#endif //_LX_CAMERA_ROS_H_
