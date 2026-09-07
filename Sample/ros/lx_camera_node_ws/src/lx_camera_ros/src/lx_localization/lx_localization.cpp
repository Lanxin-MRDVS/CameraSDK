#include "lx_localization/lx_localization.h"
#include "utils/json.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>
#include <array>

// 定位算法时间戳ms
static long GetTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  long t = tv.tv_sec * 1000L + tv.tv_usec / 1000L;
  return t;
}

struct CameraCalibMatrices {
  std::array<double, 9> K{};
  std::array<double, 14> D{};
};

static bool GetCameraCalibration(DcHandle handle, int type,
    CameraCalibMatrices& out) {
    LxIntrinsicParameters* calib = nullptr;
    if (LX_SUCCESS != DcGetPtrValue(handle,
        0 == type ? LX_PTR_2D_INTRINSIC_PARAMETERS : LX_PTR_3D_INTRINSIC_PARAMETERS,
        (void**)&calib))
        return false;

    out.K = { static_cast<double>(calib->intrinsics[0]), 0.0,
             static_cast<double>(calib->intrinsics[4]), 0.0,
             static_cast<double>(calib->intrinsics[2]),
             static_cast<double>(calib->intrinsics[5]), 0.0, 0.0, 1.0 };

    out.D.fill(0.0);
    if (calib->distortion_coeffs && calib->num_distortion_coeffs > 0) {
        const size_t copy =
            std::min<size_t>(calib->num_distortion_coeffs, out.D.size());
        for (size_t i = 0; i < copy; ++i) {
            out.D[i] = static_cast<double>(calib->distortion_coeffs[i]);
        }
    }
    return true;
}

// 版本号比较
// 0: version1 == version2; 1：version1 > version2; -1：version1 < version2
int CompareVersion(const std::string &version1, const std::string &version2) {
  std::vector<int> v1, v2;
  std::stringstream ss1(version1), ss2(version2);
  std::string token;
  while (std::getline(ss1, token, '.')) {
    v1.push_back(std::stoi(token));
  }
  while (std::getline(ss2, token, '.')) {
    v2.push_back(std::stoi(token));
  }
  size_t i = 0;
  while (i < v1.size() && i < v2.size()) {
    if (v1[i] > v2[i]) {
      return 1;
    } else if (v1[i] < v2[i]) {
      return -1;
    }
    i++;
  }
  if (v1.size() > v2.size()) {
    return 1;
  } else if (v1.size() < v2.size()) {
    return -1;
  } else {
    return 0;
  }
}

// 获取相机定位结果附加内容：当前地图、定位置信度
// 算法模块版本不匹配，可能产生错误
int GetExtentsData(int32_t *extents, const std::string &algo_ver,
                   std::string &map_name, float &pose_confidence,
                   int8_t &reloc_errorcode) {
  // std::cout << "GetData start" << std::endl;
  uint32_t num_byte = 4;
  uint32_t data_byte = 0;
  uint32_t filed_byte = 0;
  char *ex = (char *)extents;

  memcpy(&filed_byte, ex, num_byte);
  ex += filed_byte;
  // map name
  memcpy(&filed_byte, ex, num_byte);
  data_byte = filed_byte - num_byte;
  ex += num_byte;
  for (int i = 0; i < data_byte; i++) {
    char cur_val;
    memcpy(&cur_val, ex, 1);
    ex++;
    map_name.push_back(cur_val);
  }

  if (CompareVersion(algo_ver, "1.0.4") >= 0) {
    // pose confidence
    memcpy(&filed_byte, ex, num_byte);
    ex += filed_byte;
    memcpy(&filed_byte, ex, num_byte);
    data_byte = filed_byte - num_byte;
    ex += num_byte;
    memcpy(&pose_confidence, ex, data_byte);
    ex += data_byte;
    if (CompareVersion(algo_ver, "1.0.7") >= 0) {
      // reloc errorcode
      memcpy(&filed_byte, ex, num_byte);
      data_byte = filed_byte - num_byte;
      ex += num_byte;
      memcpy(&reloc_errorcode, ex, data_byte);
    }
  }
  return 0;
}

void ShowRgb(const sensor_msgs::ImagePtr &msg) {
  cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
  if (img.empty()) {
    return;
  }
  cv::imshow("RGB", img);
  cv::waitKey(1);
}

LxLocalization::LxLocalization() {
  ros::NodeHandle nh("~");
  std::string serr, scmd, smap, sloc, spar, supmap, sswmap, supscan,
      sublaserpose, sdmap, sodm, sapp, slpr;
  nh.param<std::string>("log_path", log_path_, "./");
  nh.param<bool>("is_show", is_show_, false);
  nh.param<std::string>("ip", ip_, "");
  nh.param<std::string>("LxCamera_Error", serr, "LxCamera_Error");
  nh.param<std::string>("LxCamera_Command", scmd, "LxCamera_Command");
  nh.param<std::string>("LxCamera_Mapping", smap, "LxCamera_Mapping");
  nh.param<std::string>("LxCamera_Location", sloc, "LxCamera_Location");
  nh.param<std::string>("LxCamera_SetParam", spar, "LxCamera_SetParam");
  nh.param<std::string>("LxCamera_SwitchMap", sswmap, "LxCamera_SwitchMap");
  nh.param<std::string>("LxCamera_UploadScan", supscan, "/scan");
  nh.param<std::string>("LxCamera_UploadLaserPose", sublaserpose, "/scan_pose");
  nh.param<std::string>("LxCamera_DownloadMap", sdmap, "LxCamera_DownloadMap");
  nh.param<std::string>("LxCamera_UploadMap", supmap, "LxCamera_UploadMap");
  nh.param<std::string>("LxCamera_UploadOdom", sodm, "/odom");
  nh.param<std::string>("LxCamera_UploadReloc", slpr, "/initialpose_reloc");
  nh.param<std::string>("LxCamera_LocationResult", sapp,
                        "LxCamera_LocationResult");
  nh.param<int>("auto_exposure_value", auto_exposure_value_, 50);
  nh.param<std::string>("map_name", map_name_, "example_map1");
  nh.param<bool>("mapping_mode", mapping_mode_, false);
  nh.param<bool>("localization_mode", localization_mode_, false);
  nh.getParam("camera_extrinsic_param", camera_extrinsic_param_);
  if (camera_extrinsic_param_.size() != 6) {
    ROS_ERROR("no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!");
    throw(std::invalid_argument(
        "no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!"));
  }
  nh.getParam("laser_extrinsic_param", laser_extrinsic_param_);
  if (laser_extrinsic_param_.size() != 3) {
    ROS_ERROR("no laser_extrinsic_param(x, y, yaw) found!");
    throw(std::invalid_argument("no laser_extrinsic_param(x, y, yaw) found!"));
  }
  ROS_INFO_STREAM("ip: " << ip_);
  ROS_INFO_STREAM("is_show: " << is_show_);
  ROS_INFO_STREAM("auto_exposure_value: " << auto_exposure_value_);
  ROS_INFO_STREAM("mapping_mode: " << mapping_mode_);
  ROS_INFO_STREAM("map_name: " << map_name_);
  ROS_INFO_STREAM("localization_mode: " << localization_mode_);
  ROS_INFO_STREAM(
      "camera_extrinsic_param: ["
      << camera_extrinsic_param_[0] << ", " << camera_extrinsic_param_[1]
      << ", " << camera_extrinsic_param_[2] << ", "
      << camera_extrinsic_param_[3] << ", " << camera_extrinsic_param_[4]
      << ", " << camera_extrinsic_param_[5] << "]");
  ROS_INFO_STREAM("laser_extrinsic_param: ["
                  << laser_extrinsic_param_[0] << ", "
                  << laser_extrinsic_param_[1] << ", "
                  << laser_extrinsic_param_[2] << "]");

  image_transport::ImageTransport it(nh);
  rgb_publisher_ = it.advertise("LxCamera_Rgb", 1);
  rgb_info_publisher_ =
      nh.advertise<sensor_msgs::CameraInfo>("LxCamera_RgbInfo", 1);
  message_publisher_ = nh.advertise<std_msgs::String>("LxCamera_Message", 1);
  error_publisher_ = nh.advertise<std_msgs::String>("LxCamera_Error", 1);
  app_info_publisher_ = nh.advertise<geometry_msgs::PoseStamped>(sapp, 1);

  // subscriber
  lsg_subscriber_ = nh.subscribe("LxCamera_Message", 10,
                                 &LxLocalization::MessageCallBack, this);
  err_subscriber_ =
      nh.subscribe(serr, 10, &LxLocalization::ErrorCallBack, this);
  com_subscriber_ =
      nh.subscribe(scmd, 10, &LxLocalization::CommandCallBack, this);
  mapping_subscriber_ =
      nh.subscribe(smap, 10, &LxLocalization::MappingCallBack, this);
  location_subscriber_ =
      nh.subscribe(sloc, 10, &LxLocalization::LocationCallBack, this);
  set_param_subscriber_ =
      nh.subscribe(spar, 10, &LxLocalization::SetParamCallBack, this);
  switch_map_subscriber_ =
      nh.subscribe(sswmap, 10, &LxLocalization::SwitchMapCallBack, this);
  upload_map_subscriber_ =
      nh.subscribe(supmap, 10, &LxLocalization::UploadMapCallBack, this);
  upload_scan_subscriber_ =
      nh.subscribe(supscan, 10, &LxLocalization::UploadScanCallBack, this);
  upload_laserpose_subscriber_ = nh.subscribe(
      sublaserpose, 10, &LxLocalization::UploadLaserPoseCallBack, this);
  download_map_subscriber_ =
      nh.subscribe(sdmap, 10, &LxLocalization::DownloadMapCallBack, this);
  upload_odom_subscriber_ =
      nh.subscribe(sodm, 10, &LxLocalization::UploadOdomCallBack, this);
  upload_reloc_subscriber_ =
      nh.subscribe(slpr, 10, &LxLocalization::UploadRelocCallBack, this);

  // 以下为相机SDK操作
  // 设置日志
  ROS_INFO("Api version: %s", DcGetApiVersion());
  Check("SET_LOG", DcSetInfoOutput(print_level_, enable_screen_print_,
                                   log_path_.c_str())); // 设置日志等级及路径
  ROS_INFO("Log file path: %s", log_path_.c_str());

  // 寻找设备
  LxDeviceInfo *devlist = nullptr;
  int devnum = 0;
  int search_num = 5;
  while (true) {
    Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
    if (devnum) {
      break;
    }
    if (!search_num) {
      break;
    }
    ROS_ERROR("Not Found Device. Retry...");
    usleep(1000 * 1000);
    search_num--;
  }

  // 打开设备
  std::string ip;
  LX_OPEN_MODE open_mode =
      ip == "" ? LX_OPEN_MODE::OPEN_BY_INDEX : LX_OPEN_MODE::OPEN_BY_IP;

  LxDeviceInfo info;
  if (LX_SUCCESS != Check("OPEN_DEVICE", DcOpenDevice(open_mode, ip.c_str(),
                                                      &handle_, &info))) {
    ROS_ERROR_STREAM("open device failed, open_mode:" << open_mode
                                                      << " open_param:" << ip);
    abort();
  }

  ROS_INFO("Open device success:"
           "\ndevice handle:             %lld"
           "\ndevice name:               %s"
           "\ndevice id:                 %s"
           "\ndevice ip:                 %s"
           "\ndevice sn:                 %s"
           "\ndevice mac:                %s"
           "\ndevice firmware version:   %s"
           "\ndevice algorithm version:  %s",
           handle_, info.name, info.id, info.ip, info.sn, info.mac,
           info.firmware_ver, info.algor_ver);

  Check("LX_BOOL_ENABLE_2D_STREAM",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_STREAM, 1)); // 开启RGB流
  Check("LX_INT_ALGORITHM_MODE",
        DcSetIntValue(handle_, LX_INT_ALGORITHM_MODE,
                      MODE_VISION_LOCATION)); // 设置算法模式为视觉定位

  Check("LX_INT_WORK_MODE",
        DcSetIntValue(handle_, LX_INT_WORK_MODE, 0)); // 设置工作模式为关闭
  Check("LX_BOOL_ENABLE_2D_AUTO_EXPOSURE",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE,
                       1)); // 开启2D自动曝光
  Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL",
        DcSetIntValue(handle_, LX_INT_2D_AUTO_EXPOSURE_LEVEL,
                      auto_exposure_value_)); // 设置自动曝光值为50
  Check("LX_INT_WORK_MODE",
        DcSetIntValue(handle_, LX_INT_WORK_MODE, 1)); // 设置算法模式为常开

  bool is_enable_rgb = false, is_enable_exp = false;
  LxIntValueInfo int_value, exp_val;
  Check(
      "LX_BOOL_ENABLE_2D_AUTO_EXPOSURE",
      DcGetBoolValue(handle_, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, &is_enable_exp));
  Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL",
        DcGetIntValue(handle_, LX_INT_2D_AUTO_EXPOSURE_LEVEL, &exp_val));
  Check("LX_BOOL_ENABLE_2D_STREAM",
        DcGetBoolValue(handle_, LX_BOOL_ENABLE_2D_STREAM, &is_enable_rgb));
  Check("LX_INT_ALGORITHM_MODE",
        DcGetIntValue(handle_, LX_INT_ALGORITHM_MODE, &int_value));

  char *algo_ver = nullptr;
  Check("LX_INT_ALGORITHM_MODE",
        DcGetStringValue(handle_, LX_STRING_ALGORITHM_VERSION,
                         &algo_ver)); // 获取当前应用算法版本
  algo_ver_ = algo_ver;

  ROS_INFO("rgb:%d,algor:%d,auto_exp:%d,exp_val:%d,visloc_algo_ver:%s",
           is_enable_rgb, int_value.cur_value, is_enable_exp, exp_val.cur_value,
           algo_ver_.c_str());
  if (!is_enable_rgb || int_value.cur_value != MODE_VISION_LOCATION ||
      !is_enable_exp || exp_val.cur_value != auto_exposure_value_) {
    ROS_ERROR_STREAM("camera setting failed...");
    abort();
  } // 判断值是否正确

  // 遍历已有地图，判断地图名是否正确
  // 设置地图、相机外参、激光外参
  if (mapping_mode_) {
    map_name_ = "example_map1";
    localization_mode_ = false;
  }
  if (localization_mode_) {
    mapping_mode_ = false;
  }
  auto message = nlohmann::json();
  nlohmann::json raw_json;
  raw_json["map_name"] = map_name_;                     // key值为约定值
  raw_json["external_param"] = camera_extrinsic_param_; // key值为约定值
  raw_json["laser_external_param"] = laser_extrinsic_param_; // key值为约定值
  Check("LxCamera_SwitchMap",
        DcSetStringValue(handle_, LX_STRING_ALGORITHM_PARAMS,
                         raw_json.dump().c_str()));
  // 设置定位模式or建图模式，二者不能同时设置为使能
  Check("LxCamera_Mapping", DcSetBoolValue(handle_, 3113, mapping_mode_));
  Check("LxCamera_Location", DcSetBoolValue(handle_, 3114, localization_mode_));

  rgb_camera_info_.header.frame_id = "mrdvs";
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_WIDTH, &int_value);
  rgb_camera_info_.width = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_HEIGHT, &int_value);
  rgb_camera_info_.height = int_value.cur_value;

  CameraCalibMatrices rgb_calib;
  if (GetCameraCalibration(handle_, 0, rgb_calib)) {
    rgb_camera_info_.D.assign(rgb_calib.D.begin(), rgb_calib.D.end());
    std::copy(rgb_calib.K.begin(), rgb_calib.K.end(), rgb_camera_info_.K.begin());
  } else {
    rgb_camera_info_.D.clear();
    rgb_camera_info_.K.assign(0.0);
  }
  rgb_info_publisher_.publish(rgb_camera_info_);

  if (LX_SUCCESS != Check("START_STREAM", DcStartStream(handle_))) {
    ROS_ERROR_STREAM("start stream failed...");
    abort();
  }
}

LxLocalization::~LxLocalization() {
  DcSetBoolValue(handle_, 3113, false); // 关闭定位模式
  DcSetBoolValue(handle_, 3114, false); // 关闭建图模式
  DcStopStream(handle_);                // 相机启流关闭
  DcCloseDevice(handle_);               // 关闭相机
}

void LxLocalization::Run() {
  LxIntValueInfo int_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
  int rgb_type = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_CHANNEL, &int_value);
  int rgb_channel = int_value.cur_value;

  tf::Transform tf_base_camera;
  tf::Quaternion base_camera_quat;
  tf_base_camera.setOrigin(tf::Vector3(camera_extrinsic_param_[0],
                                       camera_extrinsic_param_[1],
                                       camera_extrinsic_param_[2]));
  geometry_msgs::Quaternion base_camera_q;
  base_camera_q = tf::createQuaternionMsgFromRollPitchYaw(
      camera_extrinsic_param_[5] / 180.0 * M_PI,
      camera_extrinsic_param_[4] / 180.0 * M_PI,
      camera_extrinsic_param_[3] / 180.0 * M_PI);
  base_camera_quat.setW(base_camera_q.w);
  base_camera_quat.setX(base_camera_q.x);
  base_camera_quat.setY(base_camera_q.y);
  base_camera_quat.setZ(base_camera_q.z);
  tf_base_camera.setRotation(base_camera_quat);

  ros::Rate loop_rate(15);
  while (ros::ok()) {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------"
              << std::endl;
    // 获取定位算法结果
    void *app_ptr = nullptr;
    DcGetPtrValue(handle_, LX_PTR_ALGORITHM_OUTPUT, &app_ptr);
    LxLocation *loc_res = (LxLocation *)app_ptr;
    if (loc_res == nullptr) {
      continue;
    }
    std::string current_map_name = "";
    float pose_confidence = -1;
    int8_t reloc_errorcode = -1;

    try {
      GetExtentsData(loc_res->extents, algo_ver_, current_map_name,
                     pose_confidence, reloc_errorcode);
      if (CompareVersion(algo_ver_, "1.0.7") >= 0) {
        // reloc errorcode
        ROS_INFO_STREAM("current map:"
                        << current_map_name
                        << ", pose confidence:" << pose_confidence
                        << ", reloc errorcode:" << int(reloc_errorcode));
      } else if (CompareVersion(algo_ver_, "1.0.4") >= 0) {
        // pose confidence
        ROS_INFO_STREAM("current map:" << current_map_name
                                       << ", pose confidence:"
                                       << pose_confidence);
      } else {
        ROS_INFO_STREAM("current map:" << current_map_name);
      }
    } catch (const std::exception &e) {
      ROS_WARN_STREAM(
          "get extents data failed, please check sdk version and update...");
    } catch (...) {
      ROS_WARN_STREAM(
          "get extents data failed, please check sdk version and update...");
    }

    // std::cout << "status: " << loc_res->status << std::endl;
    // status
    // 算法返回值: 错误码
    // Unknown           = -1,  //!< 重定位中
    // Tracking          = 0,   //!< 正常定位中
    // RelocFailed       = 1,   //!< 重定位失败
    // Slipping          = 2,   //!< 打滑
    // Relocating        = 6,   //!< 重定位中
    // NoTopImage        = 7,   //!< 相机图片输入异常
    // NoOdom            = 8,   //!< 里程计超时异常
    // Mapping = 20,  //!<
    // 正常建图中，建图需要odom,图像以及激光数据（激光or激光位姿至少满足一个）
    // MappingException = 21,  //!建图异常
    // NoLaser       = 22,  //!< 激光超时异常，建图时使用
    // NoLaserPose   = 23,  //!< // 激光位姿数据超时异常，建图时使用
    switch (loc_res->status) {
    case 0: {
      geometry_msgs::PoseStamped alg_val;
      alg_val.header.stamp = ros::Time().fromSec(loc_res->timestamp / 1000.0);
      alg_val.header.frame_id = "map";
      geometry_msgs::Quaternion q;
      q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, loc_res->theta);
      alg_val.pose.position.x = loc_res->x;
      alg_val.pose.position.y = loc_res->y;
      alg_val.pose.position.z = 0;
      alg_val.pose.orientation.x = q.x;
      alg_val.pose.orientation.y = q.y;
      alg_val.pose.orientation.z = q.z;
      alg_val.pose.orientation.w = q.w;
      app_info_publisher_.publish(alg_val);
      ROS_INFO_STREAM("[VISLOC:STATUS] 0 Tracking: (" << loc_res->x << ", "
                                                      << loc_res->y << ", "
                                                      << loc_res->theta << ")");
      // pub tf
      {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion quat;
        transform.setOrigin(tf::Vector3(loc_res->x, loc_res->y, 0.0));
        quat.setW(q.w);
        quat.setX(q.x);
        quat.setY(q.y);
        quat.setZ(q.z);
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, alg_val.header.stamp,
                                              "map", "base_link"));
        br.sendTransform(tf::StampedTransform(
            tf_base_camera, alg_val.header.stamp, "base_link", "mrdvs"));
      }

      break;
    }
    case -1:
      ROS_INFO_STREAM("[VISLOC:STATUS] -1 Unknown...");
      break;
    case 1:
      ROS_INFO_STREAM("[VISLOC:STATUS] 1 RelocFailed...");
      break;
    case 2:
      ROS_INFO_STREAM("[VISLOC:STATUS] 2 Slipping...");
      break;
    case 6:
      ROS_INFO_STREAM("[VISLOC:STATUS] 6 Relocating...");
      break;
    case 7:
      ROS_INFO_STREAM("[VISLOC:STATUS] 7 NoImage...");
      break;
    case 8:
      ROS_INFO_STREAM("[VISLOC:STATUS] 8 NoOdom...");
      break;
    case 20:
      ROS_INFO_STREAM("[VISLOC:STATUS] 20 Mapping...");
      break;
    case 21:
      ROS_INFO_STREAM("[VISLOC:STATUS] 21 MappingException...");
      break;
    case 22:
      ROS_INFO_STREAM("[VISLOC:STATUS] 22 NoLaser...");
      break;
    case 23:
      ROS_INFO_STREAM("[VISLOC:STATUS] 23 NoLaserPose...");
      break;
    default:
      break;
    }

    // 获取图像数据
    if (LX_SUCCESS != Check("LX_CMD_GET_NEW_FRAME",
                            DcSetCmd(handle_, LX_CMD_GET_NEW_FRAME))) {
      ROS_WARN("%s", std::string("get new frame failed").c_str());
      continue;
    }
    void *rgb_data = nullptr;
    if (DcGetPtrValue(handle_, LX_PTR_2D_IMAGE_DATA, &rgb_data) == LX_SUCCESS) {
      cv::Mat rgb_img(rgb_camera_info_.height, rgb_camera_info_.width,
                      CV_MAKETYPE(rgb_type, rgb_channel), rgb_data);
      cv::Mat rgb_pub;
      rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);

      std::string type = rgb_channel == 3 ? "bgr8" : "mono8";
      sensor_msgs::ImagePtr msg_rgb =
          cv_bridge::CvImage(std_msgs::Header(), type, rgb_pub).toImageMsg();
      msg_rgb->header.stamp = ros::Time::now();
      msg_rgb->header.frame_id = "mrdvs";
      rgb_publisher_.publish(msg_rgb);
      if (is_show_) {
        ShowRgb(msg_rgb);
      }
    } else {
      ROS_WARN("%s", std::string("RGB image is empty!").c_str());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void LxLocalization::ErrorCallBack(const std_msgs::String::ConstPtr &msg) {
  ROS_ERROR("%s", msg->data.c_str());
}

void LxLocalization::MessageCallBack(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("%s", msg->data.c_str());
}

void LxLocalization::CommandCallBack(const std_msgs::String::ConstPtr &msg) {
  ROS_WARN("%s", msg->data.c_str());
  auto message = nlohmann::json::parse(msg->data);
  std::string execute = message["execute"].get<std::string>();
  if (execute == "GET") {
    int command = message["command"].get<int>();
    if (command / 1000 == 1) {
      LxIntValueInfo int_value;
      DcGetIntValue(handle_, (LX_CAMERA_FEATURE)command, &int_value);

      nlohmann::json json_data;
      json_data["cur_value"] = int_value.cur_value;
      json_data["max_value"] = int_value.max_value;
      json_data["min_value"] = int_value.min_value;
      json_data["set_available"] = int_value.set_available;
      message["value"] = json_data;
    } else if (command / 1000 == 2) {
      LxFloatValueInfo float_value;
      DcGetFloatValue(handle_, (LX_CAMERA_FEATURE)command, &float_value);

      nlohmann::json json_data;
      json_data["cur_value"] = float_value.cur_value;
      json_data["max_value"] = float_value.max_value;
      json_data["min_value"] = float_value.min_value;
      json_data["set_available"] = float_value.set_available;
      message["value"] = json_data;
    } else if (command / 1000 == 3) {
      bool bool_value = false;
      DcGetBoolValue(handle_, (LX_CAMERA_FEATURE)command, &bool_value);
      message["value"] = bool_value;
    } else if (command / 1000 == 4) {
      char *string_value = nullptr;
      DcGetStringValue(handle_, (LX_CAMERA_FEATURE)command, &string_value);
      std::string str = string_value == nullptr ? "NONE" : string_value;
      message["value"] = str;
    } else if (command / 1000 == 6) {
      float *ptr_val = nullptr;
      DcGetPtrValue(handle_, (LX_CAMERA_FEATURE)command, (void **)&ptr_val);
      for (int i = 0; i < 12; i++)
        message["value"][i] = ptr_val[i];
    }
    std_msgs::String str;
    str.data = message.dump();
    message_publisher_.publish(str);
  } else if (execute == "SET") {
    int command = message["command"].get<int>();
    if (command / 1000 == 1) {
      int val = message["value"].get<int>();
      message["result"] =
          DcSetIntValue(handle_, (LX_CAMERA_FEATURE)command, val);
    } else if (command / 1000 == 2) {
      float val = message["value"].get<float>();
      message["result"] =
          DcSetFloatValue(handle_, (LX_CAMERA_FEATURE)command, val);
    } else if (command / 1000 == 3) {
      bool val = message["value"].get<bool>();
      message["result"] =
          DcSetBoolValue(handle_, (LX_CAMERA_FEATURE)command, val);
    } else if (command / 1000 == 4) {
      std::string val = message["value"].get<std::string>();
      message["result"] =
          DcSetStringValue(handle_, (LX_CAMERA_FEATURE)command, val.c_str());
    } else if (command / 1000 == 5)
      message["result"] = DcSetCmd(handle_, (LX_CAMERA_FEATURE)command);
    std_msgs::String str;
    str.data = message.dump();
    message_publisher_.publish(str);
  }
}

int LxLocalization::Check(std::string command, int state) {
  LX_STATE lx_state = static_cast<LX_STATE>(state);
  if (LX_SUCCESS != lx_state) {
    const char *m = DcGetErrorString(lx_state);
    std_msgs::String msg;
    setlocale(LC_ALL, "");
    msg.data = "#command: " + command +
               " #error code: " + std::to_string(lx_state) + " #report: " + m;
    error_publisher_.publish(msg);
  }
  return state;
}

void LxLocalization::MappingCallBack(const std_msgs::String::ConstPtr &msg) {
  bool status = msg->data[0] == '1';
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_Mapping";
  message["result"] = DcSetBoolValue(handle_, 3113, status);
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::LocationCallBack(const std_msgs::String::ConstPtr &msg) {
  bool status = msg->data[0] == '1';
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_Location";
  message["result"] = DcSetBoolValue(handle_, 3114, status);
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::SetParamCallBack(const std_msgs::String::ConstPtr &msg) {
  char *msg_data = nullptr;
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_SwitchMap";
  DcGetStringValue(handle_, LX_STRING_ALGORITHM_PARAMS, &msg_data);
  if (msg_data) {
    std::string raw_val = msg_data;
    auto raw_json = nlohmann::json::parse(raw_val.c_str());
    nlohmann::json json_array = nlohmann::json::array();
    json_array.push_back(*(float *)msg->data.c_str());
    json_array.push_back(*(float *)(msg->data.c_str() + 4));
    json_array.push_back(*(float *)(msg->data.c_str() + 8));
    json_array.push_back(*(float *)(msg->data.c_str() + 12));
    json_array.push_back(*(float *)(msg->data.c_str() + 16));
    json_array.push_back(*(float *)(msg->data.c_str() + 20));
    raw_json["external_param"] = json_array;

    nlohmann::json json_array2 = nlohmann::json::array();
    json_array2.push_back(*(float *)(msg->data.c_str() + 24));
    json_array2.push_back(*(float *)(msg->data.c_str() + 28));
    json_array2.push_back(*(float *)(msg->data.c_str() + 32));
    raw_json["laser_external_param"] = json_array2;
    message["result"] = DcSetStringValue(handle_, LX_STRING_ALGORITHM_PARAMS,
                                         raw_json.dump().c_str());
    ROS_INFO("%s", raw_json.dump().c_str());
  }
  message["result"] = -1;
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::SwitchMapCallBack(const std_msgs::String::ConstPtr &msg) {
  char *msg_data_data = nullptr;
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_SwitchMap";
  DcGetStringValue(handle_, LX_STRING_ALGORITHM_PARAMS, &msg_data_data);
  std::string raw_val = msg_data_data;
  auto raw_json = nlohmann::json::parse(raw_val.c_str());
  raw_json["map_name"] = msg->data;
  message["result"] = DcSetStringValue(handle_, LX_STRING_ALGORITHM_PARAMS,
                                       raw_json.dump().c_str());
  std_msgs::String str;
  str.data = raw_json.dump();
  message_publisher_.publish(str);
}

void LxLocalization::UploadMapCallBack(const std_msgs::String::ConstPtr &msg) {
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadMap";
  message["result"] = DcSpecialControl(handle_, "ImportLocationMapFile",
                                       const_cast<char *>(msg->data.c_str()));
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::DownloadMapCallBack(
    const std_msgs::String::ConstPtr &msg) {
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_DownloadMap";
  message["result"] = DcSetStringValue(handle_, 4105, msg->data.c_str());
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::UploadScanCallBack(const sensor_msgs::LaserScan &msg) {
  LxLaser laser_data;
  laser_data.timestamp = msg.header.stamp.toNSec() * 1e-6;
  laser_data.time_increment = msg.time_increment;
  laser_data.angle_min = msg.angle_min;
  laser_data.angle_max = msg.angle_max;
  laser_data.angle_increment = msg.angle_increment;
  laser_data.range_min = msg.range_min;
  laser_data.range_max = msg.range_max;
  std::vector<float> vec_ranges;
  vec_ranges.reserve(msg.ranges.size());
  auto cur_angle = msg.angle_min;
  int i = 0;
  while (cur_angle < msg.angle_max) {
    vec_ranges.push_back(msg.ranges[i]);
    cur_angle += msg.angle_increment;
    i++;
  }
  laser_data.ranges = vec_ranges.data();
  laser_data.range_size = vec_ranges.size();

  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadScan";
  message["result"] = DcSpecialControl(handle_, "SetLaserData", &laser_data);
  message["scan"] = laser_data.range_size;
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::UploadLaserPoseCallBack(
    const geometry_msgs::PoseStamped &msg) {
  LxLaserPose data;
  data.x = msg.pose.position.x;
  data.y = msg.pose.position.y;
  data.theta = tf::getYaw(msg.pose.orientation);
  data.timestamp = msg.header.stamp.toNSec() * 1e-6;
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadLaserPose";
  message["result"] = DcSpecialControl(handle_, "SetLaserPose", &data);
  message["laserpose"] = std::vector<double>{data.x, data.y, data.theta};
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::UploadOdomCallBack(const nav_msgs::Odometry &msg) {
  LxOdomData data;
  data.x = msg.pose.pose.position.x;
  data.y = msg.pose.pose.position.y;
  data.theta = tf::getYaw(msg.pose.pose.orientation);
  // 定位模块接收时间戳单位ms
  data.timestamp = msg.header.stamp.toNSec() * 1e-6;
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadOdom";
  message["result"] = DcSpecialControl(handle_, "SetOdomData", &data);
  message["odom"] = std::vector<double>{data.x, data.y, data.theta};
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}

void LxLocalization::UploadRelocCallBack(
    const geometry_msgs::PoseWithCovarianceStamped &msg) {
  struct PoiT {
    double x, y, theta;
  } data;
  data.x = msg.pose.pose.position.x;
  data.y = msg.pose.pose.position.y;
  data.theta = tf::getYaw(msg.pose.pose.orientation);
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadReloc";
  message["result"] = DcSpecialControl(handle_, "SetRelocPose", &data);
  std_msgs::String str;
  str.data = message.dump();
  message_publisher_.publish(str);
}
