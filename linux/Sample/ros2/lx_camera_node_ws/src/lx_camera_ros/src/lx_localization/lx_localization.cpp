#include "lx_localization/lx_localization.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "utils/json.hpp"
#include <array>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

static DcLib *LX_DYNAMIC_LIB = nullptr;

// 定位算法时间戳 ms
static long GetTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  long t = tv.tv_sec * 1000L + tv.tv_usec / 1000L;
  return t;
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

void ShowRgb(const sensor_msgs::msg::Image::SharedPtr &msg) {
  cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
  if (img.empty()) {
    return;
  }
  cv::imshow("RGB", img);
  cv::waitKey(1);
}

// check接口调用返回值
int LxLocalization::Check(std::string command, int state) {
  LX_STATE lx_state = LX_STATE(state);
  if (LX_SUCCESS != lx_state) {
    const char *m = DcGetErrorString(lx_state);
    std_msgs::msg::String msg;
    setlocale(LC_ALL, "");
    msg.data = "#command: " + command +
               " #error code: " + std::to_string(state) + " #report: " + m;
    error_publisher_->publish(msg);
  }
  return state;
}

void LxLocalization::ErrorCallBack(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_ERROR(this->get_logger(), "%s", msg->data.c_str());
}

void LxLocalization::MessageCallBack(
    const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
}

void LxLocalization::MappingCallBack(
    const std_msgs::msg::String::SharedPtr msg) {
  bool status = msg->data[0] == '1';
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_Mapping";
  message["result"] = DcSetBoolValue(handle_, 3113, status);
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}
void LxLocalization::LocationCallBack(
    const std_msgs::msg::String::SharedPtr msg) {
  bool status = msg->data[0] == '1';
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_Location";
  message["result"] = DcSetBoolValue(handle_, 3114, status);
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}
void LxLocalization::SetParamCallBack(
    const std_msgs::msg::String::SharedPtr msg) {
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
    RCLCPP_INFO(this->get_logger(), "%s", raw_json.dump().c_str());
  }
  message["result"] = -1;

  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}

void LxLocalization::SwitchMapCallBack(
    const std_msgs::msg::String::SharedPtr msg) {
  char *msg_data = nullptr;
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_SwitchMap";
  DcGetStringValue(handle_, LX_STRING_ALGORITHM_PARAMS, &msg_data);
  std::string raw_val = msg_data;
  auto raw_json = nlohmann::json::parse(raw_val.c_str());
  raw_json["map_name"] = msg->data;
  message["result"] = DcSetStringValue(handle_, LX_STRING_ALGORITHM_PARAMS,
                                       raw_json.dump().c_str());

  std_msgs::msg::String str;
  str.data = raw_json.dump();
  message_publisher_->publish(str);
}

void LxLocalization::UploadMapCallBack(
    const std_msgs::msg::String::SharedPtr msg) {
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadMap";
  message["result"] = DcSpecialControl(handle_, "ImportLocationMapFile",
                                       const_cast<char *>(msg->data.c_str()));
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}

void LxLocalization::DownloadMapCallBack(
    const std_msgs::msg::String::SharedPtr msg) {
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_DownloadMap";
  message["result"] = DcSetStringValue(handle_, 4105, msg->data.c_str());
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}

void LxLocalization::UploadScanCallBack(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  LxLaser laser_data;
  laser_data.timestamp =
      msg->header.stamp.sec * 1e3 + msg->header.stamp.nanosec * 1e-6;
  laser_data.time_increment = msg->time_increment;
  laser_data.angle_min = msg->angle_min;
  laser_data.angle_max = msg->angle_max;
  laser_data.angle_increment = msg->angle_increment;
  laser_data.range_min = msg->range_min;
  laser_data.range_max = msg->range_max;
  std::vector<float> vec_ranges;
  vec_ranges.reserve(msg->ranges.size());
  auto cur_angle = msg->angle_min;
  int i = 0;
  while (cur_angle < msg->angle_max) {
    vec_ranges.push_back(msg->ranges[i]);
    cur_angle += msg->angle_increment;
    i++;
  }
  laser_data.ranges = vec_ranges.data();
  laser_data.range_size = vec_ranges.size();

  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadScan";
  message["result"] = DcSpecialControl(handle_, "SetLaserData", &laser_data);
  message["scan"] = laser_data.range_size;
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}

void LxLocalization::UploadOdomCallBack(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  LxOdomData data;
  data.x = msg->pose.pose.position.x;
  data.y = msg->pose.pose.position.y;
  Quaternion q;
  q.x = msg->pose.pose.orientation.x;
  q.y = msg->pose.pose.orientation.y;
  q.z = msg->pose.pose.orientation.z;
  q.w = msg->pose.pose.orientation.w;
  data.theta = ToEulerAngles(q).yaw;
  // 定位模块接收时间戳单位ms
  data.timestamp =
      msg->header.stamp.sec * 1e3 + msg->header.stamp.nanosec * 1e-6;
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadOdom";
  message["result"] = DcSpecialControl(handle_, "SetOdomData", &data);
  message["odom"] = std::vector<double>{data.x, data.y, data.theta};
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}

void LxLocalization::UploadLaserPoseCallBack(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  LxLaserPose data;
  data.x = msg->pose.position.x;
  data.y = msg->pose.position.y;
  Quaternion q;
  q.x = msg->pose.orientation.x;
  q.y = msg->pose.orientation.y;
  q.z = msg->pose.orientation.z;
  q.w = msg->pose.orientation.w;
  data.theta = ToEulerAngles(q).yaw;
  data.timestamp =
      msg->header.stamp.sec * 1e3 + msg->header.stamp.nanosec * 1e-6;
  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadLaserPose";
  message["result"] = DcSpecialControl(handle_, "SetLaserPose", &data);
  message["laserpose"] = std::vector<double>{data.x, data.y, data.theta};
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}

void LxLocalization::UploadRelocCallBack(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  struct PoiT {
    double x, y, theta;
  } data;

  data.x = msg->pose.pose.position.x;
  data.y = msg->pose.pose.position.y;
  Quaternion q;
  q.x = msg->pose.pose.orientation.x;
  q.y = msg->pose.pose.orientation.y;
  q.z = msg->pose.pose.orientation.z;
  q.w = msg->pose.pose.orientation.w;
  data.theta = ToEulerAngles(q).yaw;

  auto message = nlohmann::json();
  message["cmd"] = "LxCamera_UploadReloc";
  message["result"] = DcSpecialControl(handle_, "SetRelocPose", &data);
  std_msgs::msg::String str;
  str.data = message.dump();
  message_publisher_->publish(str);
}

void LxLocalization::CommandCallBack(
    const std_msgs::msg::String::SharedPtr &msg) {
  RCLCPP_INFO(this->get_logger(), msg->data.c_str());
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
    std_msgs::msg::String str;
    str.data = message.dump();
    message_publisher_->publish(str);
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
    std_msgs::msg::String str;
    str.data = message.dump();
    message_publisher_->publish(str);
  }
}

LxLocalization::LxLocalization(DcLib *dynamic_lib)
    : Node("lx_localization_node") {
  LX_DYNAMIC_LIB = dynamic_lib;
  qos_ = rmw_qos_profile_default;

  std::string serr, scmd, smap, sloc, spar, supmap, sswmap, supscan,
      sublaserpose, sdmap, sodm, sapp, slpr, ip;
  this->declare_parameter<std::string>("ip", "");
  this->declare_parameter<bool>("is_show", false);
  this->declare_parameter<std::string>("LxCamera_Error", "LxCamera_Error");
  this->declare_parameter<std::string>("LxCamera_Command", "LxCamera_Command");
  this->declare_parameter<std::string>("LxCamera_Mapping", "LxCamera_Mapping");
  this->declare_parameter<std::string>("LxCamera_Location",
                                       "LxCamera_Location");
  this->declare_parameter<std::string>("LxCamera_SetParam",
                                       "LxCamera_SetParam");
  this->declare_parameter<std::string>("LxCamera_SwitchMap",
                                       "LxCamera_SwitchMap");
  this->declare_parameter<std::string>("LxCamera_UploadScan", "/scan");
  this->declare_parameter<std::string>("LxCamera_UploadLaserPose",
                                       "/scan_pose");
  this->declare_parameter<std::string>("LxCamera_DownloadMap",
                                       "LxCamera_DownloadMap");
  this->declare_parameter<std::string>("LxCamera_UploadMap",
                                       "LxCamera_UploadMap");
  this->declare_parameter<std::string>("LxCamera_UploadOdom", "/odom");
  this->declare_parameter<std::string>("LxCamera_UploadReloc",
                                       "/initialpose_reloc");
  this->declare_parameter<std::string>("LxCamera_LocationResult",
                                       "LxCamera_LocationResult");
  this->declare_parameter<int>("auto_exposure_value", 50);
  this->declare_parameter<std::string>("map_name", "example_map1");
  this->declare_parameter<bool>("mapping_mode", true);
  this->declare_parameter<bool>("localization_mode", false);
  this->declare_parameter<std::vector<double>>("camera_extrinsic_param",
                                               std::vector<double>(6, 0.0));
  this->declare_parameter<std::vector<double>>("laser_extrinsic_param",
                                               std::vector<double>(3, 0.0));

  this->get_parameter<std::string>("ip", ip);
  this->get_parameter<bool>("is_show", is_show_);
  this->get_parameter<std::string>("LxCamera_Error", serr);
  this->get_parameter<std::string>("LxCamera_Command", scmd);
  this->get_parameter<std::string>("LxCamera_Mapping", smap);
  this->get_parameter<std::string>("LxCamera_Location", sloc);
  this->get_parameter<std::string>("LxCamera_SetParam", spar);
  this->get_parameter<std::string>("LxCamera_SwitchMap", sswmap);
  this->get_parameter<std::string>("LxCamera_UploadScan", supscan);
  this->get_parameter<std::string>("LxCamera_UploadLaserPose", sublaserpose);
  this->get_parameter<std::string>("LxCamera_DownloadMap", sdmap);
  this->get_parameter<std::string>("LxCamera_UploadMap", supmap);
  this->get_parameter<std::string>("LxCamera_UploadOdom", sodm);
  this->get_parameter<std::string>("LxCamera_UploadReloc", slpr);
  this->get_parameter<std::string>("LxCamera_LocationResult", sapp);
  this->get_parameter<int>("auto_exposure_value", auto_exposure_value_);
  this->get_parameter<std::string>("map_name", map_name_);
  this->get_parameter<bool>("mapping_mode", mapping_mode_);
  this->get_parameter<bool>("localization_mode", localization_mode_);
  this->get_parameter<std::vector<double>>("camera_extrinsic_param",
                                           camera_extrinsic_param_);
  this->get_parameter<std::vector<double>>("laser_extrinsic_param",
                                           laser_extrinsic_param_);
  if (camera_extrinsic_param_.size() != 6) {
    RCLCPP_ERROR(this->get_logger(),
                 "no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!");
    throw(std::invalid_argument(
        "no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!"));
  }
  if (laser_extrinsic_param_.size() != 3) {
    RCLCPP_ERROR(this->get_logger(),
                 "no laser_extrinsic_param(x, y, yaw) found!");
    throw(std::invalid_argument("no laser_extrinsic_param(x, y, yaw) found!"));
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "ip:" << ip);
  RCLCPP_INFO_STREAM(this->get_logger(), "is_show:" << is_show_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "auto_exposure_value: " << auto_exposure_value_);
  RCLCPP_INFO_STREAM(this->get_logger(), "mapping_mode: " << mapping_mode_);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "localization_mode: " << localization_mode_);
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "camera_extrinsic_param: ["
          << camera_extrinsic_param_[0] << ", " << camera_extrinsic_param_[1]
          << ", " << camera_extrinsic_param_[2] << ", "
          << camera_extrinsic_param_[3] << ", " << camera_extrinsic_param_[4]
          << ", " << camera_extrinsic_param_[5] << "]");
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "laser_extrinsic_param: ["
                         << laser_extrinsic_param_[0] << ", "
                         << laser_extrinsic_param_[1] << ", "
                         << laser_extrinsic_param_[2] << "]");

  // publisher
  app_info_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(sapp, 1);
  rgb_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Rgb", 1);
  rgb_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "LxCamera_RgbInfo", 1);
  message_publisher_ =
      this->create_publisher<std_msgs::msg::String>("LxCamera_Message", 10);
  error_publisher_ =
      this->create_publisher<std_msgs::msg::String>("LxCamera_Error", 10);

  // subscriber
  using std::placeholders::_1;
  std::string lxm = "LxCamera_Message";
#define cre_sub this->create_subscription
#define BIND(A) 10, std::bind(&LxLocalization::A, this, _1)
  static auto s1 = cre_sub<std_msgs::msg::String>(serr, BIND(ErrorCallBack));
  static auto s2 = cre_sub<std_msgs::msg::String>(lxm, BIND(MessageCallBack));
  static auto s3 = cre_sub<std_msgs::msg::String>(sloc, BIND(LocationCallBack));
  static auto s4 = cre_sub<std_msgs::msg::String>(spar, BIND(SetParamCallBack));
  static auto s5 = cre_sub<std_msgs::msg::String>(smap, BIND(MappingCallBack));
  static auto s6 =
      cre_sub<std_msgs::msg::String>(sswmap, BIND(SwitchMapCallBack));
  static auto s7 =
      cre_sub<std_msgs::msg::String>(supmap, BIND(UploadMapCallBack));
  static auto s8 =
      cre_sub<std_msgs::msg::String>(sdmap, BIND(DownloadMapCallBack));
  static auto s9 =
      cre_sub<nav_msgs::msg::Odometry>(sodm, BIND(UploadOdomCallBack));
  static auto s0 =
      cre_sub<sensor_msgs::msg::LaserScan>(supscan, BIND(UploadScanCallBack));
  static auto s10 = cre_sub<geometry_msgs::msg::PoseStamped>(
      sublaserpose, BIND(UploadLaserPoseCallBack));
  static auto s11 = cre_sub<geometry_msgs::msg::PoseWithCovarianceStamped>(
      slpr, BIND(UploadRelocCallBack));

  // 以下为相机SDK操作
  // set sdk log
  RCLCPP_INFO(this->get_logger(), "Api version: %s", DcGetApiVersion());
  Check("SET_LOG", DcSetInfoOutput(print_level_, enable_screen_print_,
                                   log_path_.c_str())); // 设置日志等级及路径
  RCLCPP_INFO(this->get_logger(), "Log file path: %s", log_path_.c_str());

  // find device
  LxDeviceInfo *devlist = nullptr;
  int devnum = 0;
  int search_num = 5;
  while (true) {
    Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
    if (devnum)
      break;
    if (!search_num)
      break;
    RCLCPP_ERROR(this->get_logger(), "Found device failed. retry...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    search_num--;
  }

  // open device
  LX_OPEN_MODE open_mode =
      ip == "" ? LX_OPEN_MODE::OPEN_BY_INDEX : LX_OPEN_MODE::OPEN_BY_IP;

  LxDeviceInfo info;
  if (LX_SUCCESS != Check("OPEN_DEVICE",
                          DcOpenDevice(open_mode, ip.c_str(), &handle_, &info)))
    abort();

  RCLCPP_INFO(this->get_logger(),
              "Open device success:"
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
  const char *algo_ver = "0.0.0";
  char *algo_ver = "0.0.0";
  Check("LX_INT_ALGORITHM_MODE",
        DcGetStringValue(handle_, LX_STRING_ALGORITHM_VERSION,
                         (char**)&algo_ver)); // 获取当前应用算法版本
  algo_ver_ = algo_ver;
  RCLCPP_INFO(this->get_logger(),
              "rgb:%d,algor:%d,auto_exp:%d,exp_val:%d,visloc_algo_ver:%s",
              is_enable_rgb, int_value.cur_value, is_enable_exp,
              exp_val.cur_value, algo_ver_.c_str());
  if (!is_enable_rgb || int_value.cur_value != MODE_VISION_LOCATION ||
      !is_enable_exp || exp_val.cur_value != auto_exposure_value_) {
    RCLCPP_ERROR(this->get_logger(), "camera setting failed...");
    abort(); // 判断值是否正确
  }

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

  float *intr = nullptr;
  DcGetPtrValue(handle_, LX_PTR_3D_NEW_INTRIC_PARAM, (void **)&intr);
  rgb_camera_info_.d =
      std::vector<double>{intr[4], intr[5], intr[6], intr[7], intr[8], intr[9], intr[10], intr[11], intr[12], intr[13], intr[14], intr[15],intr[16],intr[17]};
  rgb_camera_info_.k =
      std::array<double, 9>{intr[0], 0, intr[2], 0, intr[1], intr[3], 0, 0, 1};
  rgb_info_publisher_->publish(rgb_camera_info_);

  if (LX_SUCCESS != Check("START_STREAM", DcStartStream(handle_))) {
    RCLCPP_ERROR(this->get_logger(), "Start stream failed...");
    abort();
  }

  Run();
}

LxLocalization::~LxLocalization() {
  DcSetBoolValue(handle_, 3113, false); // 关闭建图使能
  DcSetBoolValue(handle_, 3114, false); // 关闭定位使能
  DcStopStream(handle_);                // 相机启流关闭
  DcCloseDevice(handle_);               // 关闭相机
}

void LxLocalization::Run() {
  LxIntValueInfo int_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
  int rgb_type = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_CHANNEL, &int_value);
  int rgb_channel = int_value.cur_value;

  auto base_camera_quat =
      ToQuaternion(camera_extrinsic_param_[3] / 180.0 * M_PI,
                   camera_extrinsic_param_[4] / 180.0 * M_PI,
                   camera_extrinsic_param_[5] / 180.0 * M_PI);
  geometry_msgs::msg::TransformStamped tf_base_camera;
  tf_base_camera.transform.translation.x = camera_extrinsic_param_[0];
  tf_base_camera.transform.translation.y = camera_extrinsic_param_[1];
  tf_base_camera.transform.translation.z = camera_extrinsic_param_[2];
  tf_base_camera.transform.rotation.x = base_camera_quat.x;
  tf_base_camera.transform.rotation.y = base_camera_quat.y;
  tf_base_camera.transform.rotation.z = base_camera_quat.z;
  tf_base_camera.transform.rotation.w = base_camera_quat.w;

  rclcpp::Node::SharedPtr node(this);
  rclcpp::Rate rate(15);
  while (rclcpp::ok()) {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------"
              << std::endl;
    // 获取定位算法结果
    void *app_ptr = nullptr;
    DcGetPtrValue(handle_, LX_PTR_ALGORITHM_OUTPUT, &app_ptr);
    LxLocation *loc_res = (LxLocation *)app_ptr;
    if (loc_res == nullptr) {
      rate.sleep();
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
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "current map:"
                               << current_map_name
                               << ", pose confidence:" << pose_confidence
                               << ", reloc errorcode:" << int(reloc_errorcode));
      } else if (CompareVersion(algo_ver_, "1.0.4") >= 0) {
        // pose confidence
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "current map:" << current_map_name
                                          << ", pose confidence:"
                                          << pose_confidence);
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "current map:" << current_map_name);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(
          this->get_logger(),
          "get extents data failed, please check sdk version and update...");
    } catch (...) {
      RCLCPP_WARN(
          this->get_logger(),
          "get extents data failed, please check sdk version and update...");
    }
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
      geometry_msgs::msg::PoseStamped alg_val;
      int64_t nanoseconds = static_cast<int64_t>(loc_res->timestamp * 1e6);
      alg_val.header.stamp.sec = nanoseconds / 1e9;
      alg_val.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      alg_val.header.frame_id = "map";
      auto qua_res = ToQuaternion(loc_res->theta, 0, 0);
      alg_val.pose.position.x = loc_res->x;
      alg_val.pose.position.y = loc_res->y;
      alg_val.pose.position.z = 0;
      alg_val.pose.orientation.x = qua_res.x;
      alg_val.pose.orientation.y = qua_res.y;
      alg_val.pose.orientation.z = qua_res.z;
      alg_val.pose.orientation.w = qua_res.w;
      app_info_publisher_->publish(alg_val);
      RCLCPP_INFO(this->get_logger(),
                  "[VISLOC:STATUS] 0 Tracking: (%lf, %lf, %lf)", loc_res->x,
                  loc_res->y, loc_res->theta);
      // pub tf
      {
        static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = rclcpp::Clock().now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = loc_res->x;
        transform_stamped.transform.translation.y = loc_res->y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = qua_res.x;
        transform_stamped.transform.rotation.y = qua_res.y;
        transform_stamped.transform.rotation.z = qua_res.z;
        transform_stamped.transform.rotation.w = qua_res.w;
        tf_broadcaster->sendTransform(transform_stamped);

        tf_base_camera.header.stamp = rclcpp::Clock().now();
        tf_base_camera.header.frame_id = "base_link";
        tf_base_camera.child_frame_id = "mrdvs";
        tf_broadcaster->sendTransform(tf_base_camera);
      }
      break;
    }
    case -1:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] -1 Unknown...");
      break;
    case 1:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 1 RelocFailed...");
      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 2 Slipping...");
      break;
    case 6:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 6 Relocating...");
      break;
    case 7:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 7 NoImage...");
      break;
    case 8:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 8 NoOdom...");
      break;
    case 20:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 20 Mapping...");
      break;
    case 21:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 21 MappingException...");
      break;
    case 22:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 22 NoLaser...");
      break;
    case 23:
      RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 23 NoLaserPose...");
      break;
    default:
      break;
    }

    // 获取图像数据
    if (LX_SUCCESS != Check("LX_CMD_GET_NEW_FRAME",
                            DcSetCmd(handle_, LX_CMD_GET_NEW_FRAME))) {
      RCLCPP_ERROR(this->get_logger(), "%s",
                   std::string("get new frame failed").c_str());
      rate.sleep();
      continue;
    }
    void *rgb_data = nullptr;
    if (DcGetPtrValue(handle_, LX_PTR_2D_IMAGE_DATA, &rgb_data) == LX_SUCCESS) {
      cv::Mat rgb_img(rgb_camera_info_.height, rgb_camera_info_.width,
                      CV_MAKETYPE(rgb_type, rgb_channel), rgb_data);
      cv::Mat rgb_pub;
      rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);
      std::string type = rgb_channel == 3 ? "bgr8" : "mono8";
      std_msgs::msg::Header rgb_header;
      rgb_header.stamp = rclcpp::Clock().now();
      rgb_header.frame_id = "mrdvs";
      sensor_msgs::msg::Image::SharedPtr msg_rgb =
          cv_bridge::CvImage(rgb_header, type, rgb_pub).toImageMsg();
      rgb_publisher_->publish(*msg_rgb);
      if (is_show_) {
        ShowRgb(msg_rgb);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s",
                   std::string("RGB image is empty!").c_str());
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }
}
