//
// Created by root on 3/14/23.
//

#include "location_node.h"
#include "../json.hpp"
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <sensor_msgs/PointCloud2.h>

static DcHandle handle;
static ros::Publisher pub_error;
static ros::Publisher pub_message;

// 定位算法时间戳ms
static long GetTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  long t = tv.tv_sec * 1000L + tv.tv_usec / 1000L;
  return t;
}

// check接口调用返回值
int Check(std::string command, LX_STATE state)
{
    if (LX_SUCCESS != state) {        
        const char* m = DcGetErrorString(state);
        std_msgs::String msg;
        setlocale(LC_ALL, "");
        msg.data = "#command: " + command + " #error code: " + std::to_string(state) + " #report: " + m;
        pub_error.publish(msg);
    }
    return state;
}

void LxCamera::MappingCallBack(const std_msgs::String::ConstPtr& msg){
    bool status = msg->data[0]=='1';
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_Mapping";
    message["result"] = DcSetBoolValue(handle, 3113, status);
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}
void LxCamera::LocationCallBack(const std_msgs::String::ConstPtr& msg){
    bool status = msg->data[0]=='1';
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_Location";
    message["result"] = DcSetBoolValue(handle, 3114, status);
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}
void LxCamera::SetParamCallBack(const std_msgs::String::ConstPtr& msg){
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
        ROS_INFO("%s", raw_json.dump().c_str());
    }
    message["result"] = -1;

    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}
void LxCamera::SwitchMapCallBack(const std_msgs::String::ConstPtr& msg){
    char* __data = nullptr;
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_SwitchMap";
    DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &__data);
    std::string raw_val = __data;
    auto raw_json = nlohmann::json::parse(raw_val.c_str());
    raw_json["map_name"] = msg->data;
    message["result"] = DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, raw_json.dump().c_str());

    std_msgs::String str;
    str.data = raw_json.dump();
    pub_message.publish(str);
}

void LxCamera::UploadMapCallBack(const std_msgs::String::ConstPtr& msg){
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadMap";
    message["result"] = DcSpecialControl(handle, "ImportLocationMapFile", const_cast <char*>(msg->data.c_str()));
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}

void LxCamera::DownloadMapCallBack(const std_msgs::String::ConstPtr& msg){
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_DownloadMap";
    message["result"] = DcSetStringValue(handle, 4105, msg->data.c_str());
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}

void LxCamera::UploadScanCallBack(const sensor_msgs::LaserScan& msg){
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
    while(cur_angle < msg.angle_max){
        vec_ranges.push_back(msg.ranges[i]);
        cur_angle += msg.angle_increment;
        i++;
    }
    laser_data.ranges = vec_ranges.data();
    laser_data.range_size = vec_ranges.size();

    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadScan";
    message["result"] = DcSpecialControl(handle, "SetLaserData", &laser_data);
    message["scan"] = laser_data.range_size;
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}

void LxCamera::UploadLaserPoseCallBack(const geometry_msgs::PoseStamped& msg){
    LxLaserPose data;
    data.x = msg.pose.position.x;
    data.y = msg.pose.position.y;
    data.theta = tf::getYaw(msg.pose.orientation);
    data.timestamp = msg.header.stamp.toNSec() * 1e-6;
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadLaserPose";
    message["result"] = DcSpecialControl(handle, "SetLaserPose", &data);
    message["laserpose"] = std::vector<double>{data.x, data.y, data.theta};
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}

void LxCamera::UploadOdomCallBack(const nav_msgs::Odometry& msg){
    LxOdomData data;
    data.x = msg.pose.pose.position.x;
    data.y = msg.pose.pose.position.y;
    data.theta = tf::getYaw(msg.pose.pose.orientation);
    // 定位模块接收时间戳单位ms
    data.timestamp = msg.header.stamp.toNSec() * 1e-6;
    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadOdom";
    message["result"] = DcSpecialControl(handle, "SetOdomData", &data);
    message["odom"] = std::vector<double>{data.x, data.y, data.theta};
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}

void LxCamera::UploadRelocCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg){
    struct PoiT {double x, y, theta;}data;

    data.x = msg.pose.pose.position.x;
    data.y = msg.pose.pose.position.y;
    data.theta = tf::getYaw(msg.pose.pose.orientation);

    auto message = nlohmann::json();
    message["cmd"] = "LxCamera_UploadReloc";
    message["result"] = DcSpecialControl(handle, "SetRelocPose", &data);
    std_msgs::String str;
    str.data = message.dump();
    pub_message.publish(str);
}

LxCamera::LxCamera()
{
    ros::NodeHandle nh("~");
    ROS_INFO("Api version: %s", DcGetApiVersion());

    std::string log_path;
    nh.param<std::string>("log_path", log_path, "./");
    Check("SET_LOG", DcSetInfoOutput(0, true, log_path.c_str()));
    ROS_INFO("Log file path: %s", log_path.c_str());

    std::string serr,scmd,smap,sloc,spar,supmap,sswmap,supscan,sublaserpose,sdmap,sodm,sapp,slpr;
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
    nh.param<std::string>("LxCamera_LocationResult", sapp, "LxCamera_LocationResult");

    nh.param<int>("auto_exposure_value", auto_exposure_value_, 50);
    nh.param<std::string>("map_name", map_name_, "example_map1");
    nh.param<bool>("mapping_mode", mapping_mode_, true);
    nh.param<bool>("localization_mode", localization_mode_, false);
    nh.getParam("camera_extrinsic_param", camera_extrinsic_param_);
    if (camera_extrinsic_param_.size() != 6) {
      ROS_ERROR("no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!");
      throw(std::invalid_argument("no camera_extrinsic_param(x, y, z, yaw, pitch, roll) found!"));
    }
    nh.getParam("laser_extrinsic_param", laser_extrinsic_param_);
    if (laser_extrinsic_param_.size() != 3) {
      ROS_ERROR("no laser_extrinsic_param(x, y, yaw) found!");
      throw(std::invalid_argument("no camera_extrinsic_param(x, y, yaw) found!"));
    }
    ROS_INFO_STREAM("auto_exposure_value: " << auto_exposure_value_);
    ROS_INFO_STREAM("mapping_mode: " << mapping_mode_);
    ROS_INFO_STREAM("localization_mode: " << localization_mode_);
    ROS_INFO_STREAM("camera_extrinsic_param: [" << camera_extrinsic_param_[0] << ", " 
                                                << camera_extrinsic_param_[1] << ", "
                                                << camera_extrinsic_param_[2] << ", "
                                                << camera_extrinsic_param_[3] << ", "
                                                << camera_extrinsic_param_[4] << ", "
                                                << camera_extrinsic_param_[5] << "]");
    ROS_INFO_STREAM("laser_extrinsic_param: [" << laser_extrinsic_param_[0] << ", " 
                                               << laser_extrinsic_param_[1] << ", "
                                               << laser_extrinsic_param_[2] << "]");
    nh.param<bool>("is_show", is_show_, false);
    ROS_INFO("is_show: %d", is_show_);

    image_transport::ImageTransport it(nh);
    rgb_publisher_ = it.advertise("LxCamera_Rgb", 1);
    rgb_info_publisher_ = nh.advertise<sensor_msgs::CameraInfo>("LxCamera_RgbInfo", 1);
    pub_message = nh.advertise<std_msgs::String>("LxCamera_Message", 1);
    pub_error = nh.advertise<std_msgs::String>("LxCamera_Error", 1);
    app_info_publisher_ = nh.advertise<geometry_msgs::PoseStamped>(sapp, 1);

    //subscriber
    mapping_subsciber_ = nh.subscribe(smap, 10, &LxCamera::MappingCallBack, this);
    location_subsciber_ = nh.subscribe(sloc, 10, &LxCamera::LocationCallBack, this);
    set_param_subsciber_ = nh.subscribe(spar, 10, &LxCamera::SetParamCallBack, this);
    switch_map_subsciber_ = nh.subscribe(sswmap, 10, &LxCamera::SwitchMapCallBack, this);
    upload_map_subsciber_ = nh.subscribe(supmap, 10, &LxCamera::UploadMapCallBack, this);
    upload_scan_subsciber_ = nh.subscribe(supscan, 10, &LxCamera::UploadScanCallBack, this);
    upload_laserpose_subsciber_ = nh.subscribe(sublaserpose, 10, &LxCamera::UploadLaserPoseCallBack, this);
    download_map_subsciber_ = nh.subscribe(sdmap, 10, &LxCamera::DownloadMapCallBack, this);
    upload_odom_subsciber_ = nh.subscribe(sodm, 10, &LxCamera::UploadOdomCallBack, this);
    upload_reloc_subsciber_ = nh.subscribe(slpr, 10, &LxCamera::UploadRelocCallBack, this);


    //find device
    LxDeviceInfo* devlist = nullptr;
    int devnum = 0;
    while (true){
        Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
        if (devnum)break;
        ROS_ERROR("Not Found Device. Retry...");
        sleep(1);
    }

    //open device
    std::string ip;
    nh.param<std::string>("ip", ip, "");
    LX_OPEN_MODE open_mode = ip == "" ?LX_OPEN_MODE::OPEN_BY_INDEX:LX_OPEN_MODE::OPEN_BY_IP;

    LxDeviceInfo info;
    if (LX_SUCCESS != Check("OPEN_DEVICE", DcOpenDevice(open_mode, ip.c_str(), &handle, &info)))
        abort();

    ROS_INFO("Open device success:"
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

    ROS_INFO("rgb:%d,algor:%d,auto_exp:%d,exp_val:%d",is_enable_rgb,int_value.cur_value,is_enable_exp,exp_val.cur_value);
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
    rgb_camera_info_.D = std::vector<double>{ intr[4], intr[5], intr[6], intr[7], intr[8] };
    rgb_camera_info_.K = boost::array<double, 9>{intr[0], 0, intr[2],0, intr[1], intr[3],0, 0, 1};
    rgb_info_publisher_.publish(rgb_camera_info_);

    if (LX_SUCCESS != Check("START_STREAM", DcStartStream(handle))) abort();
}

LxCamera::~LxCamera(){
    // DcSetBoolValue(handle, 3113, false);  //关闭定位模式
    // DcSetBoolValue(handle, 3114, false);  //关闭建图模式
    DcStopStream(handle);  //相机启流关闭
    DcCloseDevice(handle);  //关闭相机
}

void LxCamera::Run()
{
    LxIntValueInfo int_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
    int rgb_type = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value);
    int rgb_channel = int_value.cur_value;

    ros::Rate loop_rate(15);
    while (ros::ok())
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
                geometry_msgs::PoseStamped alg_val;
                alg_val.header.stamp = ros::Time::now();
                alg_val.header.frame_id = "mrdvs";
                geometry_msgs::Quaternion q;
                q=tf::createQuaternionMsgFromRollPitchYaw(0,0,__val->theta);
                alg_val.pose.position.x = __val->x;
                alg_val.pose.position.y = __val->y;
                alg_val.pose.position.z = 0;
                alg_val.pose.orientation.x = q.x;
                alg_val.pose.orientation.y = q.y;
                alg_val.pose.orientation.z = q.z;
                alg_val.pose.orientation.w = q.w;
                app_info_publisher_.publish(alg_val);
                ROS_INFO_STREAM("[VISLOC:STATUS] 0 Tracking: (" << __val->x << ", " 
                                                << __val->y << ", "
                                                << __val->theta << ")");
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
        if (LX_SUCCESS != Check("LX_CMD_GET_NEW_FRAME", DcSetCmd(handle, LX_CMD_GET_NEW_FRAME))){
            ROS_WARN("%s", std::string("get new frame failed").c_str());
            continue;
        }
        void* rgb_data = nullptr;
        if (DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data) == LX_SUCCESS)
        {
            cv::Mat rgb_img(rgb_camera_info_.height, rgb_camera_info_.width, CV_MAKETYPE(rgb_type, rgb_channel), rgb_data);
            cv::Mat rgb_pub;
            rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);
            
            std::string type = rgb_channel == 3 ? "bgr8" : "mono8";
            sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), type, rgb_pub).toImageMsg();
            msg_rgb->header.stamp = ros::Time::now();
            msg_rgb->header.frame_id = "mrdvs";
            rgb_publisher_.publish(msg_rgb);
        }
        else {
            ROS_WARN("%s", std::string("RGB image is empty!").c_str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
