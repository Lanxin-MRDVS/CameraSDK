//
// Created by root on 3/14/23.
//

#include <string>
#include "../json.hpp"
#include "location_node.h"
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// 定位算法时间戳ms
static long GetTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  long t = tv.tv_sec * 1000L + tv.tv_usec / 1000L;
  return t;
}

// check接口调用返回值
int Location::Check(std::string command, LX_STATE state)
{
    if (LX_SUCCESS != state) {        
        const char* m = DcGetErrorString(state);
        std_msgs::msg::String msg;
        setlocale(LC_ALL, "");
        msg.data = "#command: " + command + " #error code: " + std::to_string(state) + " #report: " + m;
        pub_error->publish(msg);
    }
    return state;
}

void Location::ErrorCallBack(const std_msgs::msg::String::SharedPtr msg ){
    RCLCPP_ERROR(this->get_logger(),"%s", msg->data.c_str());
}

void Location::MessageCallBack(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(),"%s", msg->data.c_str());
}

void Location::MappingCallBack(const std_msgs::msg::String::SharedPtr msg){
    bool status = msg->data[0]=='1';
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_Mapping";
    message["result"] = DcSetBoolValue(handle, 3113, status);
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}
void Location::LocationCallBack(const std_msgs::msg::String::SharedPtr msg){
    bool status = msg->data[0]=='1';
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_Location";
    message["result"] = DcSetBoolValue(handle, 3114, status);
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}
void Location::SetParamCallBack(const std_msgs::msg::String::SharedPtr msg){
    char* __data = nullptr;
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_SwitchMap";
    DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &__data);
    if(__data){
        std::string raw_val = __data;
        auto raw_json = nlohmann::json::parse(raw_val.c_str());
        nlohmann::json json_array = nlohmann::json::array();
        json_array.push_back(*(float*)msg->data.c_str());
        json_array.push_back(*(float*)(msg->data.c_str() + 4));
        json_array.push_back(*(float*)(msg->data.c_str() + 8));
        json_array.push_back(*(float*)(msg->data.c_str() + 12));
        json_array.push_back(*(float*)(msg->data.c_str() + 16));
        json_array.push_back(*(float*)(msg->data.c_str() + 20));
        raw_json["external_param"] = json_array;

        nlohmann::json json_array2 = nlohmann::json::array();
        json_array2.push_back(*(float*)(msg->data.c_str() + 24));
        json_array2.push_back(*(float*)(msg->data.c_str() + 28));
        json_array2.push_back(*(float*)(msg->data.c_str() + 32));
        raw_json["laser_external_param"] = json_array2;
        message["result"] = DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, raw_json.dump().c_str());
        RCLCPP_INFO(this->get_logger(),"%s", raw_json.dump().c_str());
    }
    message["result"] = -1;

    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}
void Location::SwitchMapCallBack(const std_msgs::msg::String::SharedPtr msg){
    char* __data = nullptr;
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_SwitchMap";
    DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &__data);
    std::string raw_val = __data;
    auto raw_json = nlohmann::json::parse(raw_val.c_str());
    raw_json["map_name"] = msg->data;
    message["result"] = DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, raw_json.dump().c_str());

    std_msgs::msg::String str;
    str.data = raw_json.dump();
    pub_message->publish(str);
}

void Location::UploadMapCallBack(const std_msgs::msg::String::SharedPtr msg){
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadMap";
    message["result"] = DcSpecialControl(handle, "ImportLocationMapFile", const_cast <char*>(msg->data.c_str()));
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}

void Location::DownloadMapCallBack(const std_msgs::msg::String::SharedPtr msg){
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_DownloadMap";
    message["result"] = DcSetStringValue(handle, 4105, msg->data.c_str());
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}

void Location::UploadScanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    LxLaser laser_data;
    laser_data.timestamp = msg->header.stamp.nanosec * 1e-6;
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
    while(cur_angle < msg->angle_max){
        vec_ranges.push_back(msg->ranges[i]);
        cur_angle += msg->angle_increment;
        i++;
    }
    laser_data.ranges = vec_ranges.data();
    laser_data.range_size = vec_ranges.size();

    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadScan";
    message["result"] = DcSpecialControl(handle, "SetLaserData", &laser_data);
    message["scan"] = laser_data.range_size;
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}

void Location::UploadOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg){
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
    data.timestamp = msg->header.stamp.nanosec * 1e-6;
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadOdom";
    message["result"] = DcSpecialControl(handle, "SetOdomData", &data);
    message["odom"] = std::vector<double>{data.x, data.y, data.theta};
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}

void Location::UploadLaserPoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    LxLaserPose data;
    data.x = msg->pose.position.x;
    data.y = msg->pose.position.y;
    Quaternion q;
    q.x = msg->pose.orientation.x;
    q.y = msg->pose.orientation.y;
    q.z = msg->pose.orientation.z;
    q.w = msg->pose.orientation.w;
    data.theta = ToEulerAngles(q).yaw;
    data.timestamp = msg->header.stamp.nanosec * 1e-6;
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadLaserPose";
    message["result"] = DcSpecialControl(handle, "SetLaserPose", &data);
    message["laserpose"] = std::vector<double>{data.x, data.y, data.theta};
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}

void Location::UploadRelocCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    struct PoiT {double x, y, theta;}data;

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
    message["result"] = DcSpecialControl(handle, "SetRelocPose", &data);
    std_msgs::msg::String str;
    str.data = message.dump();
    pub_message->publish(str);
}

Location::Location(DcLib* _lib): Node("location_node") {
    lib = _lib;
    qos = rmw_qos_profile_default;
    DcSetInfoOutput(0, true, "/var/log/");
    RCLCPP_INFO(this->get_logger(),"Api version: %s", DcGetApiVersion());

    std::string serr,scmd,smap,sloc,spar,supmap,sswmap,supscan,sublaserpose,sdmap,sodm,sapp,slpr,ip;
    this->declare_parameter<std::string>("ip", "0");
    this->declare_parameter<std::string>("LxCamera_Error", "LxCamera_Error");
    this->declare_parameter<std::string>("LxCamera_Command", "LxCamera_Command");
    this->declare_parameter<std::string>("LxCamera_Mapping", "LxCamera_Mapping");
    this->declare_parameter<std::string>("LxCamera_Location", "LxCamera_Location");
    this->declare_parameter<std::string>("LxCamera_SetParam", "LxCamera_SetParam");
    this->declare_parameter<std::string>("LxCamera_SwitchMap", "LxCamera_SwitchMap");
    this->declare_parameter<std::string>("LxCamera_UploadScan", "/scan");
    this->declare_parameter<std::string>("LxCamera_UploadLaserPose", "/scan_pose");
    this->declare_parameter<std::string>("LxCamera_DownloadMap", "LxCamera_DownloadMap");
    this->declare_parameter<std::string>("LxCamera_UploadMap", "LxCamera_UploadMap");
    this->declare_parameter<std::string>("LxCamera_UploadOdom", "/odom");
    this->declare_parameter<std::string>("LxCamera_UploadReloc", "/initialpose_reloc");
    this->declare_parameter<std::string>("LxCamera_LocationResult", "LxCamera_LocationResult");
    this->declare_parameter<int>("auto_exposure_value", 50);
    this->declare_parameter<std::string>("map_name", "example_map1");
    this->declare_parameter<bool>("mapping_mode", true);
    this->declare_parameter<bool>("localization_mode", false);

    this->get_parameter<std::string>("ip", ip);
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

    camera_extrinsic_param_ = this->get_parameter("camera_extrinsic_param").as_double_array();
    if (camera_extrinsic_param_.size() != 6) {
      RCLCPP_ERROR(this->get_logger(),"no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!");
      throw(std::invalid_argument("no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!"));
    }
    laser_extrinsic_param_ = this->get_parameter("laser_extrinsic_param").as_double_array();
    if (laser_extrinsic_param_.size() != 3) {
      RCLCPP_ERROR(this->get_logger(),"no laser_extrinsic_param(x, y, yaw) found!");
      throw(std::invalid_argument("no camera_extrinsic_param(x, y, yaw) found!"));
    }

	app_info_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(sapp, 1);
    rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Rgb", 1);
	pub_rgb_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("LxCamera_RgbInfo", 1);
    pub_message = this->create_publisher<std_msgs::msg::String>("LxCamera_Message", 10);
    pub_error = this->create_publisher<std_msgs::msg::String>("LxCamera_Error", 10);


    //subscriber
    using std::placeholders::_1;
    std::string lxm = "LxCamera_Message";
    #define cre_sub this->create_subscription
    #define BIND(A) 10, std::bind(&Location::A, this,_1)
    static auto s1 = cre_sub<std_msgs::msg::String>(serr, BIND(ErrorCallBack));
    static auto s2 = cre_sub<std_msgs::msg::String>(lxm, BIND(MessageCallBack));
    static auto s3 = cre_sub<std_msgs::msg::String>(sloc, BIND(LocationCallBack));
    static auto s4 = cre_sub<std_msgs::msg::String>(spar, BIND(SetParamCallBack));
    static auto s5 = cre_sub<std_msgs::msg::String>(smap, BIND(MappingCallBack));
    static auto  s6 = cre_sub<std_msgs::msg::String>(sswmap, BIND(SwitchMapCallBack));
    static auto s7 = cre_sub<std_msgs::msg::String>(supmap, BIND(UploadMapCallBack));
    static auto s8 = cre_sub<std_msgs::msg::String>(sdmap, BIND(DownloadMapCallBack));
    static auto s9 = cre_sub<nav_msgs::msg::Odometry>(sodm, BIND(UploadOdomCallBack));
    static auto s0 = cre_sub<sensor_msgs::msg::LaserScan>(supscan, BIND(UploadScanCallBack));
    static auto s10 = cre_sub<geometry_msgs::msg::PoseStamped>(sublaserpose, BIND(UploadLaserPoseCallBack));
    static auto s11 = cre_sub<geometry_msgs::msg::PoseWithCovarianceStamped>(slpr, BIND(UploadRelocCallBack));

    //find device
    LxDeviceInfo* devlist = nullptr;
    int devnum = 0;
    while (true){
        Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
        if (devnum)break;
        RCLCPP_ERROR(this->get_logger(),"Not Found Device. Retry...");
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    //open device
    LX_OPEN_MODE open_mode = ip == "" ?LX_OPEN_MODE::OPEN_BY_INDEX:LX_OPEN_MODE::OPEN_BY_IP;

    LxDeviceInfo info;
    if (LX_SUCCESS != Check("OPEN_DEVICE", DcOpenDevice(open_mode, ip.c_str(), &handle, &info)))
        abort();

    RCLCPP_INFO(this->get_logger(),"Open device success:"
        "\ndevice handle:             %lld"
        "\ndevice name:               %s"
        "\ndevice id:                 %s"
        "\ndevice ip:                 %s"
        "\ndevice sn:                 %s"
        "\ndevice mac:                %s"
        "\ndevice firmware version:   %s"
        "\ndevice algorithm version:  %s"
        , handle, info.name, info.id, info.ip, info.sn, info.mac, info.firmware_ver, info.algor_ver);

    Check("LX_SetInfoOutput",DcSetInfoOutput(print_level_, enable_screen_print_, log_path_));                                     //开启RGB流

    Check("LX_BOOL_ENABLE_2D_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, 1));                                     //开启RGB流
    Check("LX_INT_ALGORITHM_MODE", DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, MODE_VISION_LOCATION));                         //设置算法模式为视觉定位
    
    Check("LX_INT_WORK_MODE", DcSetIntValue(handle, LX_INT_WORK_MODE, 0));                                                      //设置工作模式为关闭
    Check("LX_BOOL_ENABLE_2D_AUTO_EXPOSURE", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, 1));                       //开启2D自动曝光
    Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL", DcSetIntValue(handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL, auto_exposure_value_));         //设置自动曝光值为50
    Check("LX_INT_WORK_MODE", DcSetIntValue(handle, LX_INT_WORK_MODE, 1));                                                    //设置算法模式为常开
    
    bool is_enable_rgb = false, is_enable_exp = false;
    LxIntValueInfo int_value, exp_val;
    Check("LX_BOOL_ENABLE_2D_AUTO_EXPOSURE", DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, &is_enable_exp));
    Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL", DcGetIntValue(handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL, &exp_val));
    Check("LX_BOOL_ENABLE_2D_STREAM", DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &is_enable_rgb));
    Check("LX_INT_ALGORITHM_MODE", DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &int_value));

    RCLCPP_INFO(this->get_logger(),"rgb:%d,algor:%d,auto_exp:%d,exp_val:%d",is_enable_rgb,int_value.cur_value,is_enable_exp,exp_val.cur_value);
    if(!is_enable_rgb || int_value.cur_value != MODE_VISION_LOCATION || !is_enable_exp || exp_val.cur_value != auto_exposure_value_) abort();      //判断值是否正确

    // 遍历已有地图，判断地图名是否正确
    // 设置地图、相机外参、激光外参
    auto message = nlohmann::json();
    nlohmann::json raw_json;
    raw_json["map_name"] = map_name_;   // key值为约定值
    raw_json["external_param"] = camera_extrinsic_param_;   // key值为约定值
    raw_json["laser_external_param"] = laser_extrinsic_param_;   // key值为约定值
    Check("LxCamera_SwitchMap", DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, raw_json.dump().c_str()));
    // 设置定位模式or建图模式，二者不能同时设置为使能
    Check("LxCamera_Mapping", DcSetBoolValue(handle, 3113, mapping_mode_));
    Check("LxCamera_Location", DcSetBoolValue(handle, 3114, localization_mode_));

    rgb_camera_info_.header.frame_id = "mrdvs";
    DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value);
    rgb_camera_info_.width = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value);
    rgb_camera_info_.height = int_value.cur_value;
    
    float* intr = nullptr;
    DcGetPtrValue(handle, LX_PTR_2D_INTRIC_PARAM, (void**)&intr);
    double r[9]{ intr[4], intr[5], intr[6], intr[7], intr[8],0,0,0,0 };
    double k[9]{intr[0], 0, intr[2],0, intr[1], intr[3],0, 0, 1};
    for(int i=0;i<9;i++)rgb_camera_info_.k[i] = k[i], rgb_camera_info_.r[i] = r[i];
    pub_rgb_info->publish(rgb_camera_info_);

    if (LX_SUCCESS != Check("START_STREAM", DcStartStream(handle))) abort();
}

Location::~Location(){
    // DcSetBoolValue(handle, 3113, false);  //关闭定位模式
    // DcSetBoolValue(handle, 3114, false);  //关闭建图模式
    DcStopStream(handle);  //相机启流关闭
    DcCloseDevice(handle);  //关闭相机
}

void Location::Run()
{
    LxIntValueInfo int_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
    int rgb_type = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value);
    int rgb_channel = int_value.cur_value;

    rclcpp::Node::SharedPtr node_(this);
    while (rclcpp::ok())
    {
        std::cout << std::endl;
        std::cout << "-------------------------------------------------" << std::endl;
        // 获取定位算法结果
        void* app_ptr = nullptr;
        DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &app_ptr);
        LxLocation* __val = (LxLocation*)app_ptr;
        if(__val == nullptr){
            continue;
        }
        // std::cout << "status: " << __val->status << std::endl;
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
        switch (__val->status)
        {
        case 0:
            {
                geometry_msgs::msg::PoseStamped alg_val;
                alg_val.header.stamp = rclcpp::Clock().now();
                alg_val.header.frame_id = "mrdvs";
                auto qua_res = ToQuaternion(__val->theta,0,0);
                alg_val.pose.position.x = __val->x;
                alg_val.pose.position.y = __val->y;
                alg_val.pose.position.z = 0;
                alg_val.pose.orientation.x = qua_res.x;
                alg_val.pose.orientation.y = qua_res.y;
                alg_val.pose.orientation.z = qua_res.z;
                alg_val.pose.orientation.w = qua_res.w;
                app_info_publisher_->publish(alg_val);
                RCLCPP_INFO(this->get_logger(), "[VISLOC:STATUS] 0 Tracking: (%lf, %lf, %lf)", __val->x ,__val->y ,__val->theta);
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
        if (LX_SUCCESS != Check("LX_CMD_GET_NEW_FRAME", DcSetCmd(handle, LX_CMD_GET_NEW_FRAME))){
            RCLCPP_ERROR(this->get_logger(), "%s", std::string("get new frame failed").c_str());
            continue;
        }
        void* rgb_data = nullptr;
        if (DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data) == LX_SUCCESS)
        {
            cv::Mat rgb_img(rgb_camera_info_.height, rgb_camera_info_.width, CV_MAKETYPE(rgb_type, rgb_channel), rgb_data);
            cv::Mat rgb_pub;
            rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);
            
            std::string type = rgb_channel == 3 ? "bgr8" : "mono8";
            sensor_msgs::msg::Image msg_rgb;
            cv_bridge::CvImage cv_img;
            cv_img.header.stamp = rclcpp::Clock().now();
            msg_rgb.header.frame_id = "mrdvs";
            cv_img.encoding = type;
            cv_img.image = rgb_pub;
            cv_img.toImageMsg(msg_rgb);
            rgb_publisher_->publish(msg_rgb);
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "%s", std::string("RGB image is empty!").c_str());
        }

        rclcpp::spin_some(node_);
    }
}
