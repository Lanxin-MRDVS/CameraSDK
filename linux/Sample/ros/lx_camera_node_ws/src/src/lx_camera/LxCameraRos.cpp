#include "LxCameraRos.h"
#include "../json.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <tf/tf.h>
using namespace std;

#include <pcl/common/common_headers.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
Eigen::Matrix4f cloud_trans;
DcHandle handle = 0;
bool is_start = 0;

sensor_msgs::CameraInfo tof_camera_info;
sensor_msgs::CameraInfo rgb_camera_info;
int is_depth = 0;
int is_amp = 0;
int is_rgb = 0;
int is_xyz = 1;
int rgb_type = 0;
int inside_app = 0;
int rgb_channel = 0;

int LxCamera::Check(string Command, LX_STATE state) {
  if (LX_SUCCESS == state)
    return state;

  const char *m = DcGetErrorString(state); //获取错误信息
  std_msgs::String msg;
  setlocale(LC_ALL, "");
  msg.data = "#Command: " + Command + " #error code: " + to_string(state) +
             " #report: " + m;
  pub_error.publish(msg); //推送错误信息
  ROS_ERROR("%s", msg.data.c_str());
  return state;
}

LX_STATE Start() {
  LxIntValueInfo int_value;
  DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value);
  tof_camera_info.width = int_value.cur_value;
  DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value);
  tof_camera_info.height = int_value.cur_value;
  DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value);
  rgb_camera_info.width = int_value.cur_value;
  DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value);
  rgb_camera_info.height = int_value.cur_value;
  DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
  rgb_type = int_value.cur_value;
  DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value);
  rgb_channel = int_value.cur_value;

  DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, (bool *)&is_depth);
  DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, (bool *)&is_amp);
  DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, (bool *)&is_rgb);
  DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &int_value);
  inside_app = int_value.cur_value;

  // get image params
  if (is_depth || is_amp || is_xyz) {
    float *intr = nullptr;
    DcGetPtrValue(handle, LX_PTR_3D_INTRIC_PARAM, (void **)&intr);
    tof_camera_info.D =
        vector<double>{intr[4], intr[5], intr[6], intr[7], intr[8]};
    tof_camera_info.K = boost::array<double, 9>{intr[0], 0, intr[2], 0, intr[1],
                                                intr[3], 0, 0,       1};
    tof_camera_info.header.frame_id = "intrinsic_depth";
  }

  if (is_rgb) {
    float *intr = nullptr;
    DcGetPtrValue(handle, LX_PTR_2D_INTRIC_PARAM, (void **)&intr);
    rgb_camera_info.D =
        vector<double>{intr[4], intr[5], intr[6], intr[7], intr[8]};
    rgb_camera_info.K = boost::array<double, 9>{intr[0], 0, intr[2], 0, intr[1],
                                                intr[3], 0, 0,       1};
    rgb_camera_info.header.frame_id = "intrinsic_rgb";
  }

  auto ret = DcStartStream(handle);
  is_start = !ret;
  return ret;
}
LX_STATE Stop() {
  auto ret = DcStopStream(handle);
  is_start = ret;
  return ret;
}

bool LxCamera::LxString(lx_camera_ros::LxString::Request &req,
                        lx_camera_ros::LxString::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetStringValue(handle, req.cmd, req.val.c_str());

  char *buf = nullptr;
  if (!req.is_set)
    res.result.ret = DcGetStringValue(handle, req.cmd, &buf);
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  if (buf)
    res.val = string(buf);
  return true;
}
bool LxCamera::LxFloat(lx_camera_ros::LxFloat::Request &req,
                       lx_camera_ros::LxFloat::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetFloatValue(handle, req.cmd, req.val);

  LxFloatValueInfo float_value;
  auto ret = DcGetFloatValue(handle, req.cmd, &float_value);
  if (!req.is_set)
    res.result.ret = ret;
  res.cur_value = float_value.cur_value;
  res.max_value = float_value.max_value;
  res.min_value = float_value.min_value;
  res.available = float_value.set_available;
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  return true;
}
bool LxCamera::LxBool(lx_camera_ros::LxBool::Request &req,
                      lx_camera_ros::LxBool::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetBoolValue(handle, req.cmd, req.val);

  bool val;
  auto ret = DcGetBoolValue(handle, req.cmd, &val);
  res.val = val;
  if (!req.is_set)
    res.result.ret = ret;
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  return true;
}
bool LxCamera::LxCmd(lx_camera_ros::LxCmd::Request &req,
                     lx_camera_ros::LxCmd::Response &res) {
  auto Pub = [&](string msg, int ret) {
    lx_camera_ros::Result result;
    result.ret = ret;
    result.msg = msg;
    res.result.push_back(result);
  };

  if (req.cmd == 1) {
    auto ret = Start();
    Pub(DcGetErrorString(ret), ret);
  } else if (req.cmd == 2) {
    auto ret = Stop();
    Pub(DcGetErrorString(ret), ret);
  } else if (req.cmd) {
    auto ret = DcSetCmd(handle, req.cmd);
    Pub(DcGetErrorString(ret), ret);
  } else {
    vector<pair<string, int>> cmd_vec;
    auto add = [&](string str, int cmd) {
      pair<string, int> val(str, cmd);
      cmd_vec.push_back(val);
    };
    add("INT      FIRST_EXPOSURE", 1001);
    add("INT      SECOND_EXPOSURE", 1002);
    add("INT      THIRD_EXPOSURE", 1003);
    add("INT      FOURTH_EXPOSURE", 1004);
    add("INT      GAIN", 1005);
    add("INT      MIN_DEPTH", 1011);
    add("INT      MAX_DEPTH", 1012);
    add("INT      MIN_AMPLITUDE", 1013);
    add("INT      MAX_AMPLITUDE", 1014);
    add("INT      CODE_MODE", 1016);
    add("INT      WORK_MODE", 1018);
    add("INT      LINK_SPEED", 1019);
    add("INT      3D_IMAGE_WIDTH", 1021);
    add("INT      3D_IMAGE_HEIGHT", 1022);
    add("INT      3D_IMAGE_OFFSET_X", 1023);
    add("INT      3D_IMAGE_OFFSET_Y", 1024);
    add("INT      3D_BINNING_MODE", 1025);
    add("INT      3D_DEPTH_DATA_TYPE", 1026);
    add("INT      3D_AMPLITUDE_CHANNEL", 1031);
    add("INT      3D_AMPLITUDE_GET_TYPE", 1032);
    add("INT      3D_AMPLITUDE_EXPOSURE", 1033);
    add("INT      3D_AMPLITUDE_INTENSITY", 1034);
    add("INT      3D_AMPLITUDE_DATA_TYPE", 1035);
    add("INT      3D_AUTO_EXPOSURE_LEVEL", 1036);
    add("INT      3D_AUTO_EXPOSURE_MAX", 1037);
    add("INT      3D_AUTO_EXPOSURE_MIN", 1038);
    add("INT      2D_IMAGE_WIDTH", 1041);
    add("INT      2D_IMAGE_HEIGHT", 1042);
    add("INT      2D_IMAGE_OFFSET_X", 1043);
    add("INT      2D_IMAGE_OFFSET_Y", 1044);
    add("INT      2D_BINNING_MODE", 1045);
    add("INT      2D_IMAGE_CHANNEL", 1046);
    add("INT      2D_IMAGE_DATA_TYPE", 1047);
    add("INT      2D_MANUAL_EXPOSURE", 1051);
    add("INT      2D_MANUAL_GAIN", 1052);
    add("INT      2D_ENCODE_TYPE", 1053);
    add("INT      2D_AUTO_EXPOSURE_LEVEL", 1054);
    add("INT      TOF_GLOBAL_OFFSET", 1061);
    add("INT      3D_UNDISTORT_SCALE", 1062);
    add("INT      ALGORITHM_MODE", 1065);
    add("INT      MODBUS_ADDR", 1066);
    add("INT      HEART_TIME", 1067);
    add("INT      GVSP_PACKET_SIZE", 1068);
    add("INT      TRIGGER_MODE", 1069);
    add("INT      CALCULATE_UP", 1070);
    add("INT      CAN_BAUD_RATE", 1072);
    add("INT      CUSTOM_PARAM_GROUP", 1075);
    add("FLOAT    FILTER_LEVEL", 2001);
    add("FLOAT    EST_OUT_EXPOSURE", 2002);
    add("FLOAT    LIGHT_INTENSITY", 2003);
    add("FLOAT    3D_DEPTH_FPS", 2004);
    add("FLOAT    3D_AMPLITUDE_FPS", 2005);
    add("FLOAT    2D_IMAGE_FPS", 2006);
    add("FLOAT    DEVICE_TEMPERATURE", 2007);
    add("BOOL     CONNECT_STATE", 3001);
    add("BOOL     ENABLE_3D_DEPTH_STREAM", 3002);
    add("BOOL     ENABLE_3D_AMP_STREAM", 3003);
    add("BOOL     ENABLE_3D_AUTO_EXPOSURE", 3006);
    add("BOOL     ENABLE_3D_UNDISTORT", 3007);
    add("BOOL     ENABLE_ANTI_FLICKER", 3008);
    add("BOOL     ENABLE_2D_STREAM", 3011);
    add("BOOL     ENABLE_2D_AUTO_EXPOSURE", 3012);
    add("BOOL     ENABLE_2D_UNDISTORT", 3015);
    add("BOOL     ENABLE_2D_TO_DEPTH", 3016);
    add("BOOL     ENABLE_BACKGROUND_AMP", 3017);
    add("BOOL     ENABLE_MULTI_MACHINE", 3018);
    add("BOOL     ENABLE_MULTI_EXPOSURE_HDR", 3019);
    add("BOOL     ENABLE_SYNC_FRAME", 3020);
    add("STRING   DEVICE_VERSION", 4001);
    add("STRING   DEVICE_LOG_NAME", 4002);
    add("STRING   FIRMWARE_NAME", 4003);
    add("STRING   FILTER_PARAMS", 4004);
    add("STRING   ALGORITHM_PARAMS", 4005);
    add("STRING   ALGORITHM_VERSION", 4006);
    add("STRING   DEVICE_OS_VERSION", 4007);
    add("CMD      GET_PARAM_LIST", 0);
    add("CMD      START_STREAM", 1);
    add("CMD      STOP_STREAM", 2);
    add("CMD      GET_NEW_FRAME", 5001);
    add("CMD      RETURN_VERSION", 5002);
    add("CMD      RESTART_DEVICE", 5003);
    add("CMD      WHITE_BALANCE", 5004);
    add("CMD      RESET_PARAM", 5007);
    add("CMD      CALIB_EXTRIC", 5008);

    for (auto &i : cmd_vec)
      while (i.first.length() < 40)
        i.first.push_back(' ');
    for (auto &i : cmd_vec)
      Pub(i.first, i.second);
  }

  return true;
}
bool LxCamera::LxInt(lx_camera_ros::LxInt::Request &req,
                     lx_camera_ros::LxInt::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetIntValue(handle, req.cmd, req.val);

  LxIntValueInfo int_value;
  auto ret = DcGetIntValue(handle, req.cmd, &int_value);
  if (!req.is_set)
    res.result.ret = ret;
  res.cur_value = int_value.cur_value;
  res.max_value = int_value.max_value;
  res.min_value = int_value.min_value;
  res.available = int_value.set_available;
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  return true;
}

//读取配置参数
void LxCamera::ReadParam() {
  nh->param<string>("ip", ip, "");
  nh->param<string>("log_path", log_path, "./");
  ROS_INFO("ip: %s", ip.c_str());
  ROS_INFO("Log file path: %s", log_path.c_str());
  nh->param<int>("is_xyz", is_xyz, 1);
  nh->param<int>("is_depth", is_depth, 0);
  nh->param<int>("is_amp", is_amp, 0);
  nh->param<int>("is_rgb", is_rgb, 0);
  nh->param<int>("raw_param", raw_param, 0);
  ROS_INFO("publish xyz: %d", is_xyz);
  ROS_INFO("publish depth: %d", is_depth);
  ROS_INFO("publish amp: %d", is_amp);
  ROS_INFO("publish rgb: %d", is_rgb);
  ROS_INFO("raw_param: %d", raw_param);

  nh->param<int>("lx_2d_binning", lx_2d_binning, 0);
  nh->param<int>("lx_2d_undistort", lx_2d_undistort, 1);
  nh->param<int>("lx_2d_undistort_scale", lx_2d_undistort_scale, 0);
  nh->param<int>("lx_2d_auto_exposure", lx_2d_auto_exposure, 1);
  nh->param<int>("lx_2d_auto_exposure_value", lx_2d_auto_exposure_value, 50);
  nh->param<int>("lx_2d_exposure", lx_2d_exposure, 11000);
  nh->param<int>("lx_2d_gain", lx_2d_gain, 114);
  ROS_INFO("lx_2d_binning: %d", lx_2d_binning);
  ROS_INFO("lx_2d_undistort: %d", lx_2d_undistort);
  ROS_INFO("lx_2d_undistort_scale: %d", lx_2d_undistort_scale);
  ROS_INFO("lx_2d_auto_exposure: %d", lx_2d_auto_exposure);
  ROS_INFO("lx_2d_auto_exposure_value: %d", lx_2d_auto_exposure_value);
  ROS_INFO("lx_2d_exposure: %d", lx_2d_exposure);
  ROS_INFO("lx_2d_gain: %d", lx_2d_gain);

  nh->param<int>("lx_rgb_to_tof", lx_rgb_to_tof, 0);
  nh->param<int>("lx_3d_binning", lx_3d_binning, 0);
  nh->param<int>("lx_mulit_mode", lx_mulit_mode, 0);
  nh->param<int>("lx_3d_undistort", lx_3d_undistort, 0);
  nh->param<int>("lx_3d_undistort_scale", lx_3d_undistort_scale, 0);
  nh->param<int>("lx_hdr", lx_hdr, 0);
  nh->param<int>("lx_3d_auto_exposure", lx_3d_auto_exposure, 1);
  nh->param<int>("lx_3d_auto_exposure_value", lx_3d_auto_exposure_value, 50);
  nh->param<int>("lx_3d_first_exposure", lx_3d_first_exposure, 1600);
  nh->param<int>("lx_3d_second_exposure", lx_3d_second_exposure, 500);
  nh->param<int>("lx_3d_gain", lx_3d_gain, 600);
  ROS_INFO("lx_rgb_to_tof: %d", lx_rgb_to_tof);
  ROS_INFO("lx_3d_binning: %d", lx_3d_binning);
  ROS_INFO("lx_mulit_mode: %d", lx_mulit_mode);
  ROS_INFO("lx_3d_undistort: %d", lx_3d_undistort);
  ROS_INFO("lx_3d_undistort_scale: %d", lx_3d_undistort_scale);
  ROS_INFO("lx_hdr: %d", lx_hdr);
  ROS_INFO("lx_3d_auto_exposure: %d", lx_3d_auto_exposure);
  ROS_INFO("lx_3d_auto_exposure_value: %d", lx_3d_auto_exposure_value);
  ROS_INFO("lx_3d_first_exposure: %d", lx_3d_first_exposure);
  ROS_INFO("lx_3d_second_exposure: %d", lx_3d_second_exposure);
  ROS_INFO("lx_3d_gain: %d", lx_3d_gain);

  nh->param<int>("lx_tof_unit", lx_tof_unit, 0);
  nh->param<int>("lx_min_depth", lx_min_depth, 0);
  nh->param<int>("lx_max_depth", lx_max_depth, 3000);
  nh->param<int>("lx_work_mode", lx_work_mode, 0);
  nh->param<int>("lx_application", inside_app, 0);
  ROS_INFO("lx_tof_unit: %d", lx_tof_unit);
  ROS_INFO("lx_min_depth: %d", lx_min_depth);
  ROS_INFO("lx_max_depth: %d", lx_max_depth);
  ROS_INFO("lx_work_mode: %d", lx_work_mode);
  ROS_INFO("lx_application mode: %d", inside_app);

  nh->param<float>("x", x, 0);
  nh->param<float>("y", y, 0);
  nh->param<float>("z", z, 0);
  nh->param<float>("yaw", yaw, 0);
  nh->param<float>("roll", roll, 0);
  nh->param<float>("pitch", pitch, 0);
  ROS_INFO("x: %f", x);
  ROS_INFO("y: %f", y);
  ROS_INFO("z: %f", z);
  ROS_INFO("yaw: %f", yaw);
  ROS_INFO("roll: %f", roll);
  ROS_INFO("pitch: %f", pitch);
}
//向相机写入配置参数
void LxCamera::SetParam() {
  ROS_WARN("SetParam now!");
  Check("LX_INT_WORK_MODE", DcSetIntValue(handle, LX_INT_WORK_MODE, 0));
  Check("LX_BOOL_ENABLE_2D_TO_DEPTH",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_TO_DEPTH, 0));
  Check("LX_INT_2D_BINNING_MODE",
        DcSetIntValue(handle, LX_INT_2D_BINNING_MODE, lx_2d_binning));
  Check("LX_BOOL_ENABLE_2D_UNDISTORT",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_UNDISTORT, lx_2d_undistort));
  Check(
      "LX_INT_2D_UNDISTORT_SCALE",
      DcSetIntValue(handle, LX_INT_2D_UNDISTORT_SCALE, lx_2d_undistort_scale));
  Check("LX_BOOL_ENABLE_2D_AUTO_EXPOSURE",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE,
                       lx_2d_auto_exposure));
  if (!lx_2d_auto_exposure) {
    Check("LX_INT_2D_MANUAL_EXPOSURE",
          DcSetIntValue(handle, LX_INT_2D_MANUAL_EXPOSURE, lx_2d_exposure));
    Check("LX_INT_2D_MANUAL_GAIN",
          DcSetIntValue(handle, LX_INT_2D_MANUAL_GAIN, lx_2d_gain));
  } else
    Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL",
          DcSetIntValue(handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL,
                        lx_2d_auto_exposure_value));

  Check("LX_BOOL_ENABLE_2D_TO_DEPTH",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_TO_DEPTH, lx_rgb_to_tof));
  Check("LX_INT_3D_BINNING_MODE",
        DcSetIntValue(handle, LX_INT_3D_BINNING_MODE, lx_3d_binning));
  Check("LX_BOOL_ENABLE_MULTI_MACHINE",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_MULTI_MACHINE, lx_mulit_mode));
  Check("LX_BOOL_ENABLE_3D_UNDISTORT",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_UNDISTORT, lx_3d_undistort));
  Check(
      "LX_INT_3D_UNDISTORT_SCALE",
      DcSetIntValue(handle, LX_INT_3D_UNDISTORT_SCALE, lx_3d_undistort_scale));
  if (!lx_3d_auto_exposure) {
    Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE",
          DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE,
                         lx_3d_auto_exposure));
    Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR",
          DcSetBoolValue(handle, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, lx_hdr));
    Check("LX_INT_FIRST_EXPOSURE",
          DcSetIntValue(handle, LX_INT_FIRST_EXPOSURE, lx_3d_first_exposure));
    Check("LX_INT_SECOND_EXPOSURE",
          DcSetIntValue(handle, LX_INT_SECOND_EXPOSURE, lx_3d_second_exposure));
    Check("LX_INT_GAIN", DcSetIntValue(handle, LX_INT_GAIN, lx_3d_gain));
  } else {
    Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR",
          DcSetBoolValue(handle, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, 0));
    Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE",
          DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE,
                         lx_3d_auto_exposure));
    Check("LX_INT_3D_AUTO_EXPOSURE_LEVEL",
          DcSetIntValue(handle, LX_INT_3D_AUTO_EXPOSURE_LEVEL,
                        lx_3d_auto_exposure_value));
  }

  Check("LX_INT_MIN_DEPTH",
        DcSetIntValue(handle, LX_INT_MIN_DEPTH, lx_min_depth));
  Check("LX_INT_MAX_DEPTH",
        DcSetIntValue(handle, LX_INT_MAX_DEPTH, lx_max_depth));
  ROS_WARN("SetParam done!");
}

//构造函数
LxCamera::LxCamera() {
  nh = new ros::NodeHandle("~");
  ROS_INFO("Api version: %s", DcGetApiVersion());
  DcSetInfoOutput(1, true, log_path.c_str());
  ReadParam();

  // advertise
  image_transport::ImageTransport it(*nh);
  pub_rgb = it.advertise("LxCamera_Rgb", 1);
  pub_amp = it.advertise("LxCamera_Amp", 1);
  pub_depth = it.advertise("LxCamera_Depth", 1);
  pub_error = nh->advertise<std_msgs::String>("LxCamera_Error", 1);
  pub_pallet = nh->advertise<lx_camera_ros::Pallet>("LxCamera_Pallet", 1);
  pub_tof_info = nh->advertise<sensor_msgs::CameraInfo>("LxCamera_TofInfo", 1);
  pub_rgb_info = nh->advertise<sensor_msgs::CameraInfo>("LxCamera_RgbInfo", 1);
  pub_temper = nh->advertise<lx_camera_ros::FrameRate>("LxCamera_FrameRate", 1);
  pub_obstacle = nh->advertise<lx_camera_ros::Obstacle>("LxCamera_Obstacle", 1);
  pub_cloud = nh->advertise<sensor_msgs::PointCloud2>("LxCamera_Cloud", 1);

  ros::ServiceServer cmd =
      nh->advertiseService("LxCamera_LxCmd", &LxCamera::LxCmd, this);
  ros::ServiceServer lxi =
      nh->advertiseService("LxCamera_LxInt", &LxCamera::LxInt, this);
  ros::ServiceServer lxb =
      nh->advertiseService("LxCamera_LxBool", &LxCamera::LxBool, this);
  ros::ServiceServer lxf =
      nh->advertiseService("LxCamera_LxFloat", &LxCamera::LxFloat, this);
  ros::ServiceServer lxs =
      nh->advertiseService("LxCamera_LxString", &LxCamera::LxString, this);

  Eigen::Matrix3f R =
      (Eigen::AngleAxisf(yaw / 180.f * M_PI, Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(pitch / 180.f * M_PI, Eigen::Vector3f::UnitX()) *
       Eigen::AngleAxisf(roll / 180.f * M_PI, Eigen::Vector3f::UnitZ()))
          .toRotationMatrix();
  Eigen::Vector3f t{x, y, z};
  Eigen::Matrix4f _trans = Eigen::Matrix4f::Identity();
  _trans.block(0, 0, 3, 3) = R;
  _trans.block(0, 3, 3, 1) = t;
  cloud_trans = _trans;

  SearchAndOpenDevice();
  if (raw_param)
    SetParam();

  Check("LX_INT_ALGORITHM_MODE",
        DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, inside_app));
  Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM,
                       is_xyz || is_depth));
  Check("LX_BOOL_ENABLE_3D_AMP_STREAM",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, is_amp));
  Check("LX_BOOL_ENABLE_2D_STREAM",
        DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, is_rgb));
  Check("LX_INT_WORK_MODE",
        DcSetIntValue(handle, LX_INT_WORK_MODE, lx_work_mode));

  run();
}

void LxCamera::SearchAndOpenDevice() {
  // find device
  int devnum = 0;
  LxDeviceInfo *devlist = nullptr;
  while (!devnum) {
    Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
    ROS_ERROR("Not Found Device. Retry...");
    sleep(1);
  }

  // open device
  int is_open = -1;
  LxDeviceInfo info;
  if (ip.empty())
    ip = "0";
  auto _mode =
      ip.size() < 8 ? LX_OPEN_MODE::OPEN_BY_INDEX : LX_OPEN_MODE::OPEN_BY_IP;
  while (is_open)
    is_open = DcOpenDevice(_mode, ip.c_str(), &handle, &info);

  ROS_INFO("Open device success:"
           "\ndevice handle:             %lld"
           "\ndevice name:               %s"
           "\ndevice id:                 %s"
           "\ndevice ip:                 %s"
           "\ndevice sn:                 %s"
           "\ndevice mac:                %s"
           "\ndevice firmware version:   %s"
           "\ndevice algorithm version:  %s",
           handle, info.name, info.id, info.ip, info.sn, info.mac,
           info.firmware_ver, info.algor_ver);
}

LxCamera::~LxCamera() {
  DcStopStream(handle);
  DcCloseDevice(handle);
}

void LxCamera::run() {
  while (!is_start)
    Start();
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    FrameInfo *one_frame = nullptr;
    if (!is_start) {
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }
    if (Check("LX_CMD_GET_NEW_FRAME", DcSetCmd(handle, LX_CMD_GET_NEW_FRAME)))
      continue;
    if (Check("LX_PTR_FRAME_DATA",
              DcGetPtrValue(handle, LX_PTR_FRAME_DATA, (void **)&one_frame)))
      continue;
    float dep(0), amp(0), rgb(0), temp(0);
    auto now_time = ros::Time::now();
    LxFloatValueInfo f_val;

    if (is_xyz) {
      float *xyz_data = nullptr;
      if (DcGetPtrValue(handle, LX_PTR_XYZ_DATA, (void **)&xyz_data) ==
          LX_SUCCESS) {
        sensor_msgs::PointCloud2 pcl2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto buff_len = tof_camera_info.width * tof_camera_info.height;
        cloud_->points.resize(buff_len);
        if (lx_tof_unit) {
          for (long i = 0; i < buff_len; i++) {
            long index = 3 * i;
            cloud_->points[i].x = xyz_data[index] / 1000.f;
            cloud_->points[i].y = xyz_data[index + 1] / 1000.f;
            cloud_->points[i].z = xyz_data[index + 2] / 1000.f;
          }
        } else
          memcpy(cloud_->points.data(), xyz_data, buff_len * 3 * 4);
        cloud_->width = tof_camera_info.width;
        cloud_->height = tof_camera_info.height;
        if (x || y || z || yaw || roll || pitch) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cg(
              new pcl::PointCloud<pcl::PointXYZ>);
          pcl::transformPointCloud(*cloud_, *pt_cg, cloud_trans);
          pcl::toROSMsg(*pt_cg, pcl2);
        } else
          pcl::toROSMsg(*cloud_, pcl2);
        pcl2.header.stamp = ros::Time().fromSec(
            one_frame->depth_data.sensor_timestamp / 1000000.0);
        pcl2.header.frame_id = "mrdvs";
        pub_cloud.publish(pcl2);
      } else
        ROS_WARN("%s", string("Cloud point data is empty!").c_str());
    }

    if (is_depth) {
      void *dep_data = one_frame->depth_data.frame_data;
      if (dep_data) {
        cv::Mat dep_img(one_frame->depth_data.frame_height,
                        one_frame->depth_data.frame_width, CV_16UC1, dep_data);
        auto msg_depth =
            cv_bridge::CvImage(std_msgs::Header(), "16UC1", dep_img)
                .toImageMsg();
        msg_depth->header.stamp = ros::Time().fromSec(
            one_frame->depth_data.sensor_timestamp / 1000000.0);
        msg_depth->header.frame_id = "mrdvs";
        pub_depth.publish(msg_depth);
      } else
        ROS_WARN("%s", string("Depth image is empty!").c_str());
      Check("LX_FLOAT_3D_DEPTH_FPS",
            DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &f_val));
      dep = f_val.cur_value;
    }

    if (is_amp) {
      void *amp_data = one_frame->amp_data.frame_data;
      if (amp_data) {
        cv::Mat amp_img(one_frame->amp_data.frame_height,
                        one_frame->amp_data.frame_width, CV_16UC1, amp_data);
        auto msg_amp = cv_bridge::CvImage(std_msgs::Header(), "16UC1", amp_img)
                           .toImageMsg();
        msg_amp->header.stamp = ros::Time().fromSec(
            one_frame->amp_data.sensor_timestamp / 1000000.0);
        msg_amp->header.frame_id = "mrdvs";
        pub_amp.publish(msg_amp);
      } else
        ROS_WARN("%s", string("Amplitude image is empty!").c_str());
      Check("LX_FLOAT_3D_AMPLITUDE_FPS",
            DcGetFloatValue(handle, LX_FLOAT_3D_AMPLITUDE_FPS, &f_val));
      amp = f_val.cur_value;
    }

    if (is_rgb) {
      void *rgb_data = one_frame->rgb_data.frame_data;
      if (rgb_data) {
        cv::Mat rgb_pub;
        auto _type = CV_MAKETYPE(rgb_type, rgb_channel);
        cv::Mat rgb_img(one_frame->rgb_data.frame_height,
                        one_frame->rgb_data.frame_width, _type, rgb_data);
        rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);

        std::string type = rgb_channel == 3 ? "bgr8" : "mono8";
        sensor_msgs::ImagePtr msg_rgb =
            cv_bridge::CvImage(std_msgs::Header(), type, rgb_pub).toImageMsg();
        msg_rgb->header.stamp = ros::Time().fromSec(
            one_frame->rgb_data.sensor_timestamp / 1000000.0);
        msg_rgb->header.frame_id = "mrdvs";
        pub_rgb.publish(msg_rgb);
      } else
        ROS_WARN("%s", string("RGB image is empty!").c_str());
      Check("LX_FLOAT_2D_IMAGE_FPS",
            DcGetFloatValue(handle, LX_FLOAT_2D_IMAGE_FPS, &f_val));
      rgb = f_val.cur_value;
    }
    if (is_amp || is_depth || is_xyz)
      pub_tof_info.publish(tof_camera_info);
    if (is_rgb)
      pub_rgb_info.publish(rgb_camera_info);

    Check("LX_FLOAT_DEVICE_TEMPERATURE",
          DcGetFloatValue(handle, LX_FLOAT_DEVICE_TEMPERATURE, &f_val));
    temp = f_val.cur_value;
    lx_camera_ros::FrameRate fr;
    fr.header.frame_id = "mrdvs";
    fr.header.stamp = now_time;
    fr.amp = amp;
    fr.rgb = rgb;
    fr.depth = dep;
    fr.temperature = temp;
    pub_temper.publish(fr);

    int ret = 0;
    void *app_ptr = one_frame->app_data.frame_data;

    switch (inside_app) {
    case MODE_AVOID_OBSTACLE: {
      lx_camera_ros::Obstacle result;
      result.header.frame_id = "mrdvs";
      result.header.stamp =
          ros::Time().fromSec(one_frame->app_data.sensor_timestamp / 1000000.0);
      Check("GetObstacleIO", DcSpecialControl(handle, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle.publish(result);
        break;
      }
      LxAvoidanceOutput *lao = (LxAvoidanceOutput *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle.publish(result);
      break;
    }
    case MODE_PALLET_LOCATE: {
      if (ret || !app_ptr)
        break;
      lx_camera_ros::Pallet result;
      result.header.frame_id = "mrdvs";
      result.header.stamp =
          ros::Time().fromSec(one_frame->app_data.sensor_timestamp / 1000000.0);
      LxPalletPose *lao = (LxPalletPose *)app_ptr;
      result.status = lao->return_val;
      result.x = lao->x;
      result.y = lao->y;
      result.yaw = lao->yaw;
      pub_pallet.publish(result);

      break;
    }
    case MODE_VISION_LOCATION: {
      if (ret || !app_ptr)
        break;
      LxLocation *__val = (LxLocation *)app_ptr;
      if (!__val->status) {
        geometry_msgs::PoseStamped alg_val;
        alg_val.header.frame_id = "mrdvs";
        alg_val.header.stamp = ros::Time().fromSec(
            one_frame->app_data.sensor_timestamp / 1000000.0);

        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, __val->theta);
        alg_val.pose.position.x = __val->x;
        alg_val.pose.position.y = __val->y;
        alg_val.pose.position.z = 0;
        alg_val.pose.orientation.x = q.x;
        alg_val.pose.orientation.y = q.y;
        alg_val.pose.orientation.z = q.z;
        alg_val.pose.orientation.w = q.w;
        pub_location.publish(alg_val);
      }
      break;
    }
    case MODE_AVOID_OBSTACLE2: {
      lx_camera_ros::Obstacle result;
      result.header.frame_id = "mrdvs";
      result.header.stamp =
          ros::Time().fromSec(one_frame->app_data.sensor_timestamp / 1000000.0);
      Check("GetObstacleIO", DcSpecialControl(handle, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle.publish(result);
        break;
      }
      LxAvoidanceOutputN *lao = (LxAvoidanceOutputN *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle.publish(result);
      break;
    }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
