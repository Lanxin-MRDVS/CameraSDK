#include <string>
#include "../json.hpp"
#include "LxCameraRos.h"
#include "rclcpp/callback_group.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/registration/icp.h>
Eigen::Matrix4f trans;
using namespace std;
DcHandle handle = 0;
bool is_start = 0;
DcLib* lib;

geometry_msgs::msg::TransformStamped tf;
sensor_msgs::msg::CameraInfo tof_info;
sensor_msgs::msg::CameraInfo rgb_info; 
int is_depth = 0;
int is_amp = 0;
int is_rgb = 0;
int is_xyz = 1;
int rgb_type = 0;
int	inside_app = 0;
int rgb_channel = 0;
int x,y,z,yaw,roll,pitch;

int LxCamera::Check(string command, LX_STATE state){
    if (LX_SUCCESS == state)return state;
    
    const char* m = DcGetErrorString(state);//获取错误信息
    std_msgs::msg::String msg;
    setlocale(LC_ALL, "");
    msg.data = "#command: " + command + " #error code: " + to_string(state) + " #report: " + m;
    pub_error->publish(msg);//推送错误信息
    RCLCPP_ERROR(this->get_logger(), "%s", msg.data.c_str());
    return state;
}

LX_STATE Start(){
    LxIntValueInfo int_value;
    DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value);
	tof_info.width = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value);
	tof_info.height = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value);
    rgb_info.width = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value);
    rgb_info.height = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
    rgb_type = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value);
    rgb_channel = int_value.cur_value;

    DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, (bool*)&is_depth);
    DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, (bool*)&is_amp);
    DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, (bool*)&is_rgb);
    DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &int_value);
    inside_app = int_value.cur_value;

    //get image params
    if (is_depth || is_amp || is_xyz) {
        float* intr = nullptr, *ex_intr = nullptr;
        DcGetPtrValue(handle, LX_PTR_3D_INTRIC_PARAM, (void**)&intr);
        DcGetPtrValue(handle, LX_PTR_3D_EXTRIC_PARAM, (void**)&ex_intr);
        double d[5]{ intr[4], intr[5], intr[6], intr[7], intr[8]};
        double k[9]{intr[0], 0, intr[2],0, intr[1], intr[3],0, 0, 1};
        for(int i=0;i<9;i++)tof_info.k[i] = k[i], tof_info.r[i] = ex_intr[i],tof_info.d.push_back(d[i]);

        auto q = ToQuaternion(yaw,pitch,roll);
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf.transform.rotation.x = q.x;
        tf.transform.rotation.y = q.y;
        tf.transform.rotation.z = q.z;
        tf.transform.rotation.w = q.w;

        tof_info.header.frame_id = "intrinsic_tof";
        tf.header.frame_id = "intrinsic_tof";
        tf.child_frame_id  = "mrdvs";
    }

    if (is_rgb) {
        float* intr = nullptr;
        DcGetPtrValue(handle, LX_PTR_2D_INTRIC_PARAM, (void**)&intr);
        double d[5]{ intr[4], intr[5], intr[6], intr[7], intr[8]};
        double k[9]{intr[0], 0, intr[2],0, intr[1], intr[3],0, 0, 1};
        for(int i=0;i<9;i++)tof_info.k[i] = k[i], tof_info.d.push_back(d[i]);
        rgb_info.header.frame_id = "intrinsic_rgb";
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

bool LxString(const lx_camera_ros::srv::LxString::Request::SharedPtr req, const lx_camera_ros::srv::LxString::Response::SharedPtr res){
    if (req->is_set)res->result.ret = DcSetStringValue(handle, req->cmd, req->val.c_str());

    char* buf = nullptr;
    if (!req->is_set)res->result.ret = DcGetStringValue(handle, req->cmd, &buf);
    res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
    if(buf) res->val = string(buf);
    return true;
}
bool LxFloat(const lx_camera_ros::srv::LxFloat::Request::SharedPtr req, const lx_camera_ros::srv::LxFloat::Response::SharedPtr res){
    if (req->is_set)res->result.ret = DcSetFloatValue(handle, req->cmd, req->val);

    LxFloatValueInfo float_value{0,0,0,0,0};
    auto ret = DcGetFloatValue(handle, req->cmd, &float_value);
    if (!req->is_set)res->result.ret = ret;
    res->cur_value = float_value.cur_value;
    res->max_value = float_value.max_value;
    res->min_value = float_value.min_value;
    res->available = float_value.set_available;
    res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
    return true;
}
bool LxBool(const lx_camera_ros::srv::LxBool::Request::SharedPtr req, const lx_camera_ros::srv::LxBool::Response::SharedPtr res){
    if (req->is_set)res->result.ret = DcSetBoolValue(handle, req->cmd, req->val);

    bool val;
    auto ret = DcGetBoolValue(handle, req->cmd, &val);
    res->val = val;
    if (!req->is_set)res->result.ret = ret;
    res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
    return true;
}
bool LxCmd(const lx_camera_ros::srv::LxCmd::Request::SharedPtr req, const lx_camera_ros::srv::LxCmd::Response::SharedPtr res){
    auto Pub = [&](string msg,int ret){
        lx_camera_ros::msg::Result result;
        result.ret = ret;
        result.msg = msg;
        res->result.push_back(result);
    };

    if (req->cmd == 1){
        auto ret = Start();
        Pub(DcGetErrorString(ret), ret);
    }
    else if (req->cmd == 2){
        auto ret = Stop();
        Pub(DcGetErrorString(ret), ret);
    }
    else if (req->cmd){
        auto ret = DcSetCmd(handle, req->cmd);
        Pub(DcGetErrorString(ret), ret);
    }
    else{
        vector<pair<string,int>>cmd_vec;
        auto add = [&](string str,int cmd){
            pair<string,int>val(str,cmd);
            cmd_vec.push_back(val);
        };
        add("INT      FIRST_EXPOSURE", 1001);
        add("INT      SECOND_EXPOSURE",1002);
        add("INT      THIRD_EXPOSURE",1003);
        add("INT      FOURTH_EXPOSURE",1004);
        add("INT      GAIN",1005);
        add("INT      MIN_DEPTH",1011);
        add("INT      MAX_DEPTH",1012);
        add("INT      MIN_AMPLITUDE",1013);
        add("INT      MAX_AMPLITUDE",1014);
        add("INT      CODE_MODE",1016);
        add("INT      WORK_MODE",1018);
        add("INT      LINK_SPEED",1019);
        add("INT      3D_IMAGE_WIDTH",1021);
        add("INT      3D_IMAGE_HEIGHT",1022);
        add("INT      3D_IMAGE_OFFSET_X",1023);
        add("INT      3D_IMAGE_OFFSET_Y",1024);
        add("INT      3D_BINNING_MODE",1025);
        add("INT      3D_DEPTH_DATA_TYPE",1026);
        add("INT      3D_AMPLITUDE_CHANNEL",1031);
        add("INT      3D_AMPLITUDE_GET_TYPE",1032);
        add("INT      3D_AMPLITUDE_EXPOSURE",1033);
        add("INT      3D_AMPLITUDE_INTENSITY",1034);
        add("INT      3D_AMPLITUDE_DATA_TYPE",1035);
        add("INT      3D_AUTO_EXPOSURE_LEVEL",1036);
        add("INT      3D_AUTO_EXPOSURE_MAX",1037);
        add("INT      3D_AUTO_EXPOSURE_MIN",1038);
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

        for(auto& i : cmd_vec)while(i.first.length()<40)i.first.push_back(' ');
        for(auto& i : cmd_vec)Pub(i.first, i.second);
    }


    return true;
}
bool LxInt(const lx_camera_ros::srv::LxInt::Request::SharedPtr req, const lx_camera_ros::srv::LxInt::Response::SharedPtr res){
    if (req->is_set)res->result.ret = DcSetIntValue(handle, req->cmd, req->val);

    LxIntValueInfo int_value{0,0,0,0,0};
    auto ret = DcGetIntValue(handle, req->cmd, &int_value);
    if (!req->is_set)res->result.ret = ret;
    res->cur_value = int_value.cur_value;
    res->max_value = int_value.max_value;
    res->min_value = int_value.min_value;
    res->available = int_value.set_available;
    res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
    return true;
}

//读取配置参数
void LxCamera::ReadParam(){
    this->declare_parameter<std::string>("ip", "0");
    this->declare_parameter<std::string>("log_path", "./");
    this->declare_parameter<int>("raw_param", 0);
    this->declare_parameter<int>("is_xyz", 0);
    this->declare_parameter<int>("is_rgb", 0);
    this->declare_parameter<int>("is_amp", 0);
    this->declare_parameter<int>("is_depth", 1);
    this->declare_parameter<int>("lx_2d_binning", 0);
    this->declare_parameter<int>("lx_2d_undistort", 0);
    this->declare_parameter<int>("lx_2d_undistort_scale", 0);
    this->declare_parameter<int>("lx_2d_auto_exposure", 0);
    this->declare_parameter<int>("lx_2d_auto_exposure_value", 0);
    this->declare_parameter<int>("lx_2d_exposure", 1000);
    this->declare_parameter<int>("lx_2d_gain", 100);
    this->declare_parameter<int>("lx_rgb_to_tof", 0);
    this->declare_parameter<int>("lx_3d_binning", 0);
    this->declare_parameter<int>("lx_mulit_mode", 0);
    this->declare_parameter<int>("lx_3d_undistort", 0);
    this->declare_parameter<int>("lx_3d_undistort_scale", 0);
    this->declare_parameter<int>("lx_hdr", 0);
    this->declare_parameter<int>("lx_3d_auto_exposure", 0);
    this->declare_parameter<int>("lx_3d_auto_exposure_value", 0);
    this->declare_parameter<int>("lx_3d_first_exposure", 0);
    this->declare_parameter<int>("lx_3d_second_exposure", 0);
    this->declare_parameter<int>("lx_3d_gain", 0);
    this->declare_parameter<int>("lx_tof_unit", 0);
    this->declare_parameter<int>("lx_min_depth", 0);
    this->declare_parameter<int>("lx_max_depth", 6000);
    this->declare_parameter<int>("lx_work_mode", 0);
    this->declare_parameter<int>("lx_application", 0);
    this->declare_parameter<int>("x", 0);
    this->declare_parameter<int>("y", 0);
    this->declare_parameter<int>("z", 0);
    this->declare_parameter<int>("yaw", 0);
    this->declare_parameter<int>("roll", 0);
    this->declare_parameter<int>("pitch", 0);

     this->get_parameter<string>("ip", ip);
     this->get_parameter<string>("log_path", log_path);
	RCLCPP_INFO(this->get_logger(), "ip: %s", ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Log file path: %s", log_path.c_str());

    this->get_parameter<int>("is_xyz", is_xyz);
    this->get_parameter<int>("is_depth", is_depth);
    this->get_parameter<int>("is_amp", is_amp);
    this->get_parameter<int>("is_rgb", is_rgb);
    this->get_parameter<int>("raw_param", raw_param);
	RCLCPP_INFO(this->get_logger(), "publish xyz: %d", is_xyz);
	RCLCPP_INFO(this->get_logger(), "publish depth: %d", is_depth);
	RCLCPP_INFO(this->get_logger(), "publish amp: %d", is_amp);
	RCLCPP_INFO(this->get_logger(), "publish rgb: %d", is_rgb);
	RCLCPP_INFO(this->get_logger(), "raw_param: %d", raw_param);

    this->get_parameter<int>("lx_2d_binning", lx_2d_binning);
    this->get_parameter<int>("lx_2d_undistort", lx_2d_undistort);
    this->get_parameter<int>("lx_2d_undistort_scale", lx_2d_undistort_scale);
    this->get_parameter<int>("lx_2d_auto_exposure", lx_2d_auto_exposure);
    this->get_parameter<int>("lx_2d_auto_exposure_value", lx_2d_auto_exposure_value);
    this->get_parameter<int>("lx_2d_exposure", lx_2d_exposure);
    this->get_parameter<int>("lx_2d_gain", lx_2d_gain);
	RCLCPP_INFO(this->get_logger(), "lx_2d_binning: %d", lx_2d_binning);
	RCLCPP_INFO(this->get_logger(), "lx_2d_undistort: %d", lx_2d_undistort);
	RCLCPP_INFO(this->get_logger(), "lx_2d_undistort_scale: %d", lx_2d_undistort_scale);
	RCLCPP_INFO(this->get_logger(), "lx_2d_auto_exposure: %d", lx_2d_auto_exposure);
	RCLCPP_INFO(this->get_logger(), "lx_2d_auto_exposure_value: %d", lx_2d_auto_exposure_value);
	RCLCPP_INFO(this->get_logger(), "lx_2d_exposure: %d", lx_2d_exposure);
	RCLCPP_INFO(this->get_logger(), "lx_2d_gain: %d", lx_2d_gain);

    this->get_parameter<int>("lx_rgb_to_tof", lx_rgb_to_tof);
    this->get_parameter<int>("lx_3d_binning", lx_3d_binning);
    this->get_parameter<int>("lx_mulit_mode", lx_mulit_mode);
    this->get_parameter<int>("lx_3d_undistort", lx_3d_undistort);
    this->get_parameter<int>("lx_3d_undistort_scale", lx_3d_undistort_scale);
    this->get_parameter<int>("lx_hdr", lx_hdr);
    this->get_parameter<int>("lx_3d_auto_exposure", lx_3d_auto_exposure);
    this->get_parameter<int>("lx_3d_auto_exposure_value", lx_3d_auto_exposure_value);
    this->get_parameter<int>("lx_3d_first_exposure", lx_3d_first_exposure);
    this->get_parameter<int>("lx_3d_second_exposure", lx_3d_second_exposure);
    this->get_parameter<int>("lx_3d_gain", lx_3d_gain);
	RCLCPP_INFO(this->get_logger(), "lx_rgb_to_tof: %d", lx_rgb_to_tof);
	RCLCPP_INFO(this->get_logger(), "lx_3d_binning: %d", lx_3d_binning);
	RCLCPP_INFO(this->get_logger(), "lx_mulit_mode: %d", lx_mulit_mode);
	RCLCPP_INFO(this->get_logger(), "lx_3d_undistort: %d", lx_3d_undistort);
	RCLCPP_INFO(this->get_logger(), "lx_3d_undistort_scale: %d", lx_3d_undistort_scale);
	RCLCPP_INFO(this->get_logger(), "lx_hdr: %d", lx_hdr);
	RCLCPP_INFO(this->get_logger(), "lx_3d_auto_exposure: %d", lx_3d_auto_exposure);
	RCLCPP_INFO(this->get_logger(), "lx_3d_auto_exposure_value: %d", lx_3d_auto_exposure_value);
	RCLCPP_INFO(this->get_logger(), "lx_3d_first_exposure: %d", lx_3d_first_exposure);
	RCLCPP_INFO(this->get_logger(), "lx_3d_second_exposure: %d", lx_3d_second_exposure);
	RCLCPP_INFO(this->get_logger(), "lx_3d_gain: %d", lx_3d_gain);

    this->get_parameter<int>("lx_tof_unit", lx_tof_unit);
    this->get_parameter<int>("lx_min_depth", lx_min_depth);
    this->get_parameter<int>("lx_max_depth", lx_max_depth);
    this->get_parameter<int>("lx_work_mode", lx_work_mode);
    this->get_parameter<int>("lx_application", inside_app);
	RCLCPP_INFO(this->get_logger(), "lx_tof_unit: %d", lx_tof_unit);
	RCLCPP_INFO(this->get_logger(), "lx_min_depth: %d", lx_min_depth);
	RCLCPP_INFO(this->get_logger(), "lx_max_depth: %d", lx_max_depth);
	RCLCPP_INFO(this->get_logger(), "lx_work_mode: %d", lx_work_mode);
	RCLCPP_INFO(this->get_logger(), "lx_application mode: %d", inside_app);

    this->get_parameter<int>("x", x);
    this->get_parameter<int>("y", y);
    this->get_parameter<int>("z", z);
    this->get_parameter<int>("yaw", yaw);
    this->get_parameter<int>("roll", roll);
    this->get_parameter<int>("pitch", pitch);
	RCLCPP_INFO(this->get_logger(), "x: %d", x);
	RCLCPP_INFO(this->get_logger(), "y: %d", y);
	RCLCPP_INFO(this->get_logger(), "z: %d", z);
	RCLCPP_INFO(this->get_logger(), "yaw: %d", yaw);
	RCLCPP_INFO(this->get_logger(), "roll: %d", roll);
	RCLCPP_INFO(this->get_logger(), "pitch: %d", pitch);
}
//向相机写入配置参数
void LxCamera::SetParam(){
    RCLCPP_WARN(this->get_logger(),"SetParam...");
    Check("LX_INT_WORK_MODE", DcSetIntValue(handle, LX_INT_WORK_MODE, 0));
    Check("LX_BOOL_ENABLE_2D_TO_DEPTH", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_TO_DEPTH, 0));

    Check("LX_INT_2D_BINNING_MODE", DcSetIntValue(handle, LX_INT_2D_BINNING_MODE, lx_2d_binning));
    Check("LX_BOOL_ENABLE_2D_UNDISTORT", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_UNDISTORT, lx_2d_undistort));
    Check("LX_INT_2D_UNDISTORT_SCALE", DcSetIntValue(handle, LX_INT_2D_UNDISTORT_SCALE, lx_2d_undistort_scale));
    Check("LX_BOOL_ENABLE_2D_AUTO_EXPOSURE", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE, lx_2d_auto_exposure));
    if(!lx_2d_auto_exposure){
        Check("LX_INT_2D_MANUAL_EXPOSURE", DcSetIntValue(handle, LX_INT_2D_MANUAL_EXPOSURE, lx_2d_exposure));
        Check("LX_INT_2D_MANUAL_GAIN", DcSetIntValue(handle, LX_INT_2D_MANUAL_GAIN, lx_2d_gain));
    }
    else Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL", DcSetIntValue(handle, LX_INT_2D_AUTO_EXPOSURE_LEVEL, lx_2d_auto_exposure_value));

    Check("LX_BOOL_ENABLE_2D_TO_DEPTH", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_TO_DEPTH, lx_rgb_to_tof));
    Check("LX_INT_3D_BINNING_MODE", DcSetIntValue(handle, LX_INT_3D_BINNING_MODE, lx_3d_binning));
    Check("LX_BOOL_ENABLE_MULTI_MACHINE", DcSetBoolValue(handle, LX_BOOL_ENABLE_MULTI_MACHINE, lx_mulit_mode));
    Check("LX_BOOL_ENABLE_3D_UNDISTORT", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_UNDISTORT, lx_3d_undistort));
    Check("LX_INT_3D_UNDISTORT_SCALE", DcSetIntValue(handle, LX_INT_3D_UNDISTORT_SCALE, lx_3d_undistort_scale));
    if(!lx_3d_auto_exposure){
        Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, lx_3d_auto_exposure));
        Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR", DcSetBoolValue(handle, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, lx_hdr));
        Check("LX_INT_FIRST_EXPOSURE", DcSetIntValue(handle, LX_INT_FIRST_EXPOSURE, lx_3d_first_exposure));
        Check("LX_INT_SECOND_EXPOSURE", DcSetIntValue(handle, LX_INT_SECOND_EXPOSURE, lx_3d_second_exposure));
        Check("LX_INT_GAIN", DcSetIntValue(handle, LX_INT_GAIN, lx_3d_gain));
    }
    else {
        Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR", DcSetBoolValue(handle, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, 0));
        Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE, lx_3d_auto_exposure));
        Check("LX_INT_3D_AUTO_EXPOSURE_LEVEL", DcSetIntValue(handle, LX_INT_3D_AUTO_EXPOSURE_LEVEL, lx_3d_auto_exposure_value));
    }

    Check("LX_INT_MIN_DEPTH", DcSetIntValue(handle, LX_INT_MIN_DEPTH, lx_min_depth));
    Check("LX_INT_MAX_DEPTH", DcSetIntValue(handle, LX_INT_MAX_DEPTH, lx_max_depth));
    RCLCPP_WARN(this->get_logger(),"SetParam done!");
}

//构造函数
LxCamera::LxCamera(DcLib* _lib) : Node("lx_camera_node"){
    RCLCPP_INFO(this->get_logger(),"lx_camera_node start!");

    lib = _lib;
    qos = rmw_qos_profile_default;
    RCLCPP_INFO(this->get_logger(),"Api version: %s", DcGetApiVersion());
    DcSetInfoOutput(1, true, log_path.c_str());
    ReadParam();

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    pub_rgb = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Rgb", 1);
	pub_rgb_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("LxCamera_RgbInfo", 1);
    pub_amp = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Amp", 1);
    pub_depth = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Depth", 1);
	pub_tof_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("LxCamera_TofInfo", 1);
    pub_error = this->create_publisher<std_msgs::msg::String>("LxCamera_Error", 10);
	
    pub_pallet =  this->create_publisher<lx_camera_ros::msg::Pallet>("LxCamera_Pallet", 1);
    pub_temper = this->create_publisher<lx_camera_ros::msg::FrameRate>("LxCamera_FrameRate", 1);
    pub_obstacle = this->create_publisher<lx_camera_ros::msg::Obstacle>("LxCamera_Obstacle", 1);
	pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("LxCamera_Cloud", 1);
	pub_tf = this->create_publisher<geometry_msgs::msg::TransformStamped>("LxCamera_TF", 1);

    auto cmd = this->create_service<lx_camera_ros::srv::LxCmd>("LxCamera_LxCmd",  &LxCmd);
    auto lxi = this->create_service<lx_camera_ros::srv::LxInt>("LxCamera_LxInt",  &LxInt);
    auto lxb = this->create_service<lx_camera_ros::srv::LxBool>("LxCamera_LxBool",  &LxBool);
    auto lxf = this->create_service<lx_camera_ros::srv::LxFloat>("LxCamera_LxFloat",  &LxFloat);
    auto lxs = this->create_service<lx_camera_ros::srv::LxString>("LxCamera_LxString",  &LxString);

	Eigen::Matrix3f R =
		(Eigen::AngleAxisf(yaw / 180.f * M_PI, Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(pitch / 180.f * M_PI, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf(roll / 180.f * M_PI, Eigen::Vector3f::UnitZ()))
		.toRotationMatrix();
	Eigen::Vector3f t{ (float)x, (float)y, (float)z };
	Eigen::Matrix4f _trans = Eigen::Matrix4f::Identity();
	_trans.block(0, 0, 3, 3) = R;
	_trans.block(0, 3, 3, 1) = t;
	trans = _trans;

    SearchAndOpenDevice();
    if(raw_param) SetParam();

    Check("LX_INT_ALGORITHM_MODE", DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, inside_app));
    Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, is_xyz || is_depth));
    Check("LX_BOOL_ENABLE_3D_AMP_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, is_amp));
    Check("LX_BOOL_ENABLE_2D_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, is_rgb));
    Check("LX_INT_WORK_MODE", DcSetIntValue(handle, LX_INT_WORK_MODE, lx_work_mode));

    run();
}

void LxCamera::SearchAndOpenDevice(){
    //find device
    int devnum = 0;
    LxDeviceInfo* devlist = nullptr;
    while (!devnum){
        Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
        RCLCPP_ERROR(this->get_logger(),"Not Found Device. Retry...");
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    //open device
    int is_open = -1;
    LxDeviceInfo info;
    if (ip.empty())ip = "0";
    auto _mode = ip.size()<8?LX_OPEN_MODE::OPEN_BY_INDEX:LX_OPEN_MODE::OPEN_BY_IP;
    while(is_open) is_open = DcOpenDevice(_mode, ip.c_str(), &handle, &info);

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

}

LxCamera::~LxCamera(){
    DcStopStream(handle);
    DcCloseDevice(handle);
}

void LxCamera::run(){
    while(!is_start) Start();
    rclcpp::Node::SharedPtr node_(this);
    while (rclcpp::ok()){
        if(!is_start){rclcpp::spin_some(node_);continue; }
        if (Check("LX_CMD_GET_NEW_FRAME", DcSetCmd(handle, LX_CMD_GET_NEW_FRAME))) continue;
        rclcpp::Time now = this->get_clock()->now();
        float dep(0),amp(0),rgb(0),temp(0);
        LxFloatValueInfo f_val;

		if(is_xyz){
            float* xyz_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_XYZ_DATA, (void**)&xyz_data) == LX_SUCCESS){
                sensor_msgs::msg::PointCloud2 pcl2;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
                auto buff_len = tof_info.width * tof_info.height;
				cloud_->points.resize(buff_len);
                if(lx_tof_unit){
                    for (long i = 0; i < buff_len; i++) {
                        long index = 3 * i;
                        cloud_->points[i].x = xyz_data[index]/1000.f;
                        cloud_->points[i].y = xyz_data[index + 1]/1000.f;
                        cloud_->points[i].z = xyz_data[index + 2]/1000.f;
                    }
                }
                else memcpy(cloud_->points.data(),xyz_data,buff_len*3*4);
                cloud_->height = tof_info.height;
                cloud_->width = tof_info.width;

                pcl::PointCloud<pcl::PointXYZ>::Ptr cld = cloud_;
                if(x||y||z||yaw||roll||pitch){
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cg(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::transformPointCloud(*cloud_, *pt_cg, trans);
                    cld = pt_cg;
                }
                pcl::toROSMsg(*cld, pcl2);
                pcl2.header.stamp = now;
                pcl2.header.frame_id = "mrdvs";
                pub_cloud->publish(pcl2);
            }
            else RCLCPP_WARN(this->get_logger(),"%s", string("Cloud point data is empty!").c_str());			
		}
		
        if (is_depth){
            void* dep_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &dep_data) == LX_SUCCESS){
                cv::Mat dep_img(tof_info.height, tof_info.width, CV_16UC1, dep_data);

                sensor_msgs::msg::Image msg_depth;
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = now;
                cv_img.header.frame_id = "mrdvs";
                cv_img.encoding = "mono16";
                cv_img.image = dep_img;
                cv_img.toImageMsg(msg_depth);
                pub_depth->publish(msg_depth);
            }
            else RCLCPP_WARN(this->get_logger(),"%s", string("Depth image is empty!").c_str());
            Check("LX_FLOAT_3D_DEPTH_FPS", DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &f_val));
            dep = f_val.cur_value;
        }

        if (is_amp){
            void* amp_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_3D_AMP_DATA, &amp_data) == LX_SUCCESS){
                cv::Mat amp_img(tof_info.height, tof_info.width, CV_16UC1, amp_data);

                sensor_msgs::msg::Image msg_amp;
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = now;
                cv_img.header.frame_id = "mrdvs";
                cv_img.encoding = "mono16";
                cv_img.image = amp_img;
                cv_img.toImageMsg(msg_amp);
                pub_amp->publish(msg_amp);
            }
            else RCLCPP_WARN(this->get_logger(),"%s", string("Amplitude image is empty!").c_str());
            Check("LX_FLOAT_3D_AMPLITUDE_FPS", DcGetFloatValue(handle, LX_FLOAT_3D_AMPLITUDE_FPS, &f_val));
            amp = f_val.cur_value;
        }

        if (is_rgb){
            void* rgb_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data) == LX_SUCCESS){
                cv::Mat rgb_pub;
                cv::Mat rgb_img(rgb_info.height, rgb_info.width, CV_MAKETYPE(rgb_type, rgb_channel), rgb_data);
                rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);
                std::string type = rgb_channel == 3 ? "bgr8" : "mono8";

                sensor_msgs::msg::Image msg_rgb;
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = now;
                cv_img.header.frame_id = "mrdvs";
                cv_img.encoding = type;
                cv_img.image = rgb_pub;
                cv_img.toImageMsg(msg_rgb);
                pub_rgb->publish(msg_rgb);
            }
            else RCLCPP_WARN(this->get_logger(),"%s", string("RGB image is empty!").c_str());
            Check("LX_FLOAT_2D_IMAGE_FPS", DcGetFloatValue(handle, LX_FLOAT_2D_IMAGE_FPS, &f_val));
            rgb = f_val.cur_value;
        }
        if(is_amp||is_depth||is_xyz) tof_info.header.stamp = now, pub_tof_info->publish(tof_info);
        if(is_rgb) rgb_info.header.stamp = now, pub_rgb_info->publish(rgb_info);
        if(is_xyz) tf.header.stamp = now, pub_tf->publish(tf);

        Check("LX_FLOAT_DEVICE_TEMPERATURE", DcGetFloatValue(handle, LX_FLOAT_DEVICE_TEMPERATURE, &f_val));
        temp = f_val.cur_value;
        lx_camera_ros::msg::FrameRate fr;
        fr.header.frame_id = "mrdvs";
        fr.header.stamp = now;
        fr.amp = amp;
        fr.rgb = rgb;
        fr.depth = dep;
        fr.temperature = temp;
        pub_temper->publish(fr);

        int ret = 0;
        void* app_ptr = nullptr;
        if(inside_app) ret = Check("ALGORITHM_OUTPUT", DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &app_ptr));
        if(ret || !app_ptr){
            rclcpp::spin_some(node_);
            continue;
        }

        switch(inside_app){
            case MODE_AVOID_OBSTACLE:{
                lx_camera_ros::msg::Obstacle result;
                result.header.frame_id = "mrdvs";
                result.header.stamp = now;
                Check("GetObstacleIO", DcSpecialControl(handle, "GetObstacleIO", (void*)&result.io_output));
                LxAvoidanceOutput* lao = (LxAvoidanceOutput*)app_ptr;
                result.status = lao->state;
                result.box_number = lao->number_box;
                for(int i=0;i<lao->number_box;i++){
                    auto raw_box = lao->obstacleBoxs[i];
                    lx_camera_ros::msg::ObstacleBox box;
                    box.width = raw_box.width;
                    box.depth = raw_box.depth;
                    box.height = raw_box.height;
                    for(int t = 0;t<3;t++)box.center[t] = raw_box.center[t];
                    for(int t = 0;t<9;t++)box.rotation[t] = raw_box.pose.R[t];
                    for(int t = 0;t<3;t++)box.translation[t] = raw_box.pose.T[t];
                    result.box.push_back(box);
                }
                pub_obstacle->publish(result);
                break;
            }
            case MODE_PALLET_LOCATE:{
                lx_camera_ros::msg::Pallet result;
                result.header.frame_id = "mrdvs";
                result.header.stamp = now;
                LxPalletPose* lao = (LxPalletPose*)app_ptr;
                result.status = lao->return_val;
                result.x = lao->x;
                result.y = lao->y;
                result.yaw = lao->yaw;
                pub_pallet->publish(result);   
                break;
            }
            case MODE_VISION_LOCATION:{
                LxLocation* __val = (LxLocation*)app_ptr;
                if(!__val->status){
                    geometry_msgs::msg::PoseStamped alg_val;
                    alg_val.header.stamp = now;
                    alg_val.header.frame_id = "mrdvs";
                    auto qua_res = ToQuaternion(__val->theta,0,0);
                    alg_val.pose.position.x = __val->x;
                    alg_val.pose.position.y = __val->y;
                    alg_val.pose.position.z = 0;
                    alg_val.pose.orientation.x = qua_res.x;
                    alg_val.pose.orientation.y = qua_res.y;
                    alg_val.pose.orientation.z = qua_res.z;
                    alg_val.pose.orientation.w = qua_res.w;
                    pub_location->publish(alg_val);
                }
                break;
            }
            case MODE_AVOID_OBSTACLE2:{
                lx_camera_ros::msg::Obstacle result;
                result.header.frame_id = "mrdvs";
                result.header.stamp = now;
                Check("GetObstacleIO", DcSpecialControl(handle, "GetObstacleIO", (void*)&result.io_output));
                LxAvoidanceOutputN* lao = (LxAvoidanceOutputN*)app_ptr;
                result.status = lao->state;
                result.box_number = lao->number_box;
                for(int i=0;i<lao->number_box;i++){
                    auto raw_box = lao->obstacleBoxs[i];
                    lx_camera_ros::msg::ObstacleBox box;
                    box.width = raw_box.width;
                    box.depth = raw_box.depth;
                    box.height = raw_box.height;
                    for(int t = 0;t<3;t++)box.center[t] = raw_box.center[t];
                    for(int t = 0;t<9;t++)box.rotation[t] = raw_box.pose.R[t];
                    for(int t = 0;t<3;t++)box.translation[t] = raw_box.pose.T[t];
                    result.box.push_back(box);
                }
                pub_obstacle->publish(result);
                break;
            }
        }
        rclcpp::spin_some(node_);
    }
}
