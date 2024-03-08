//
// Created by root on 3/14/23.
//

#include "LxCameraRos.h"
#include "json.hpp"
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>

static DcHandle handle;
static Pub_Str pub_error;
static Pub_Str pub_message;
#define CREATE_SUB_IMG(A,B)  this->create_subscription<sensor_msgs::msg::Image>(A, default_qos, std::bind(B, this, std::placeholders::_1))
#define CREATE_SUB_MSG(A,B)  this->create_subscription<std_msgs::msg::String>(A, qos,std::bind(B, this, std::placeholders::_1))

int LxCamera::Check(std::string command, LX_STATE state){
    if (LX_SUCCESS != state) {        
        const char* m = DcGetErrorString(state);
        std_msgs::msg::String msg;
        msg.data = "#command: " + command + " #error code: " + std::to_string(state) + " #report: " + m;
        pub_error->publish(msg);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.data.c_str());
    }
    return state;
}

void LxCamera::LxCamera_Error(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_ERROR(this->get_logger(), "%s", msg->data.c_str());
}

void LxCamera::LxCamera_rgb(sensor_msgs::msg::Image::SharedPtr msg){
    cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::imshow("RGB", img);
    cv::waitKey(1);
}

void LxCamera::LxCamera_amp(sensor_msgs::msg::Image::SharedPtr msg){
    cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::imshow("AMP", img);
    cv::waitKey(1);
}

void LxCamera::LxCamera_dep(sensor_msgs::msg::Image::SharedPtr msg){
    cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::imshow("DEPTH", img);
    cv::waitKey(1);
}

//相机交互，通过json字符串，包含键值"execute"，"command"，"value"，"result"
void LxCamera::LxCamera_Command(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_WARN(this->get_logger(), "%s", msg->data.c_str());
    auto message = nlohmann::json::parse(msg->data);
    std::string execute = message["execute"].get<std::string>();
    //获取参数
    if (execute == "GET")
    {
        int command = message["command"].get<int>();
        //int类型
        if (command / 1000 == 1)
        {
            LxIntValueInfo int_value;
            DcGetIntValue(handle, (LX_CAMERA_FEATURE)command, &int_value);

            nlohmann::json _val;
            _val["cur_value"] = int_value.cur_value;
            _val["max_value"] = int_value.max_value;
            _val["min_value"] = int_value.min_value;
            _val["set_available"] = int_value.set_available;
            message["value"] = _val;
        }
        //float类型
        else if (command / 1000 == 2)
        {
            LxFloatValueInfo float_value;
            DcGetFloatValue(handle, (LX_CAMERA_FEATURE)command, &float_value);

            nlohmann::json _val;
            _val["cur_value"] = float_value.cur_value;
            _val["max_value"] = float_value.max_value;
            _val["min_value"] = float_value.min_value;
            _val["set_available"] = float_value.set_available;
            message["value"] = _val;
        }
        //bool类型
        else if (command / 1000 == 3)
        {
            bool bool_value = false;
            DcGetBoolValue(handle, (LX_CAMERA_FEATURE)command, &bool_value);
            message["value"] = bool_value;
        }
        //字符串类型
        else if (command / 1000 == 4)
        {
            char* string_value = nullptr;
            DcGetStringValue(handle, (LX_CAMERA_FEATURE)command, &string_value);
            std::string str = string_value == nullptr ? "NONE" : string_value;
            message["value"] = str;
        }
        //指针类型，仅限内外参，其他参数的指针类型和尺寸需要根据实际情况确定
        else if (command / 1000 == 6)
        {
            float* ptr_val = nullptr;
            DcGetPtrValue(handle, (LX_CAMERA_FEATURE)command, (void**)&ptr_val);
            for (int i = 0; i < 12; i++)
                message["value"][i] = ptr_val[i];
        }
        std_msgs::msg::String str;
        str.data = message.dump();
        pub_message->publish(str);
    }
    //设置参数
    else if (execute == "SET")
    {
        int command = message["command"].get<int>();
        //int类型
        if (command / 1000 == 1)
        {
            int val = message["value"].get<int>();
            message["result"] = DcSetIntValue(handle, (LX_CAMERA_FEATURE)command, val);
        }
        //float类型
        else if (command / 1000 == 2)
        {
            float val = message["value"].get<float>();
            message["result"] = DcSetFloatValue(handle, (LX_CAMERA_FEATURE)command, val);
        }
        //bool类型
        else if (command / 1000 == 3)
        {
            bool val = message["value"].get<bool>();
            message["result"] = DcSetBoolValue(handle, (LX_CAMERA_FEATURE)command, val);
        }
        //字符串类型
        else if (command / 1000 == 4)
        {
            std::string val = message["value"].get<std::string>();
            message["result"] = DcSetStringValue(handle, (LX_CAMERA_FEATURE)command, val.c_str());
        }
        //命令类型
        else if (command / 1000 == 5)
        {
            message["result"] = DcSetCmd(handle, (LX_CAMERA_FEATURE)command);
        }
        std_msgs::msg::String str;
        str.data = message.dump();
        pub_message->publish(str);
    }
    //特殊操作，为万能通配接口，需要根据实际commond确定value
	//for example:
	//int obstacle_mode = 1;
	//DcSpecialControl(handle, "SetObstacleMode", (void*)&pobstacle_mode);
    else if (execute == "SPECAIL")
    {
		try{
			std::string command = message["command"].get<std::string>();
			int val = message["value"].get<int>();
			message["result"] = DcSpecialControl(handle, command.c_str(), (void*)&val);
			
			std_msgs::msg::String str;
			str.data = message.dump();
			pub_message->publish(str);
		}
		catch (const std::exception& e)
		{
			RCLCPP_ERROR(this->get_logger(), "%s", e.what());
			return;
		}
    }
}

LxCamera::LxCamera() : Node("lx_camera_node"){
    qos = rmw_qos_profile_default;

    std::string ip = "0";
    std::string log_path = "./";
    this->get_parameter("log_path", log_path);
    RCLCPP_INFO(this->get_logger(), "Api version: %s", DcGetApiVersion());
    Check("SET_LOG", DcSetInfoOutput(1, true, log_path.c_str()));
    RCLCPP_INFO(this->get_logger(), "Log file path: %s", log_path.c_str());

    this->declare_parameter<std::string>("ip", "0");
    this->declare_parameter<int>("is_xyz", is_xyz);
    this->declare_parameter<int>("is_rgb", is_rgb);
    this->declare_parameter<int>("is_amp", is_amp);
    this->declare_parameter<int>("is_show", is_show);
    this->declare_parameter<int>("is_depth", is_depth);
    this->declare_parameter<int>("application", inside_app);

    this->get_parameter<std::string>("ip", ip);
    this->get_parameter<int>("is_xyz", is_xyz);
    this->get_parameter<int>("is_rgb", is_rgb);
    this->get_parameter<int>("is_amp", is_amp);
    this->get_parameter<int>("is_show", is_show);
    this->get_parameter<int>("is_depth", is_depth);
    this->get_parameter<int>("application", inside_app);
	RCLCPP_INFO(this->get_logger(), "camera ip: %s", ip.c_str());
	RCLCPP_INFO(this->get_logger(), "publish xyz: %d", is_xyz);
	RCLCPP_INFO(this->get_logger(), "publish rgb: %d", is_rgb);
	RCLCPP_INFO(this->get_logger(), "publish amp: %d", is_amp);
	RCLCPP_INFO(this->get_logger(), "sublish show: %d", is_show);
	RCLCPP_INFO(this->get_logger(), "publish depth: %d", is_depth);
	RCLCPP_INFO(this->get_logger(), "application mode: %d", inside_app);

    if(!is_rgb && !is_amp) is_depth = true;
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
	if(is_rgb){
        pub_rgb = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Rgb", 1);
		pub_rgb_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("LxCamera_RgbInfo", 1);
        if (is_show) static Sub_Img rgb = CREATE_SUB_IMG("LxCamera_Rgb", &LxCamera::LxCamera_rgb);
    }
	if(is_amp){
        pub_amp = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Amp", 1);
        if (is_show) static Sub_Img amp = CREATE_SUB_IMG("LxCamera_Amp", &LxCamera::LxCamera_amp);
    }
	if(is_depth){
        pub_depth = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Depth", 1);
	    pub_tof_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("LxCamera_TofInfo", 1);
        if (is_show) static Sub_Img amp = CREATE_SUB_IMG("LxCamera_Depth", &LxCamera::LxCamera_dep);
	}
	if(is_xyz){
        pub_xyz = this->create_publisher<sensor_msgs::msg::PointCloud2>("LxCamera_xyz", 1);
		pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud>("LxCamera_Cloud", 1);
	}
    pub_message = this->create_publisher<std_msgs::msg::String>("LxCamera_Message", 10);
    pub_error = this->create_publisher<std_msgs::msg::String>("LxCamera_Error", 10);

    static Sub_Str err;
    static Sub_Str com;
    //static Sub_Str err = CREATE_SUB_MSG("LxCamera_Error", &LxCamera::LxCamera_Error );
    //static Sub_Str com = CREATE_SUB_MSG("LxCamera_Command", &LxCamera::LxCamera_Command );

    //find device
    LxDeviceInfo* devlist = nullptr;
    int devnum = 0;
    while (!devnum){
        Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
        if(!devnum) RCLCPP_ERROR(this->get_logger(), "Not Found Device. Retry... %d",devnum);
        sleep(1);
    }
    //open device
    LxDeviceInfo info;
    LX_OPEN_MODE open_mode = ip=="0"?OPEN_BY_INDEX:OPEN_BY_IP;
    if (LX_SUCCESS != Check("OPEN_DEVICE", DcOpenDevice(open_mode, ip.c_str(), &handle, &info))) abort();

    RCLCPP_INFO(this->get_logger(), "Open device success:"
        "\ndevice handle:             %lld"
        "\ndevice name:               %s"
        "\ndevice id:                 %s"
        "\ndevice ip:                 %s"
        "\ndevice sn:                 %s"
        "\ndevice mac:                %s"
        "\ndevice firmware version:   %s"
        "\ndevice algorithm version:  %s"
        , handle, info.name, info.id, info.ip, info.sn, info.mac, info.firmware_ver, info.algor_ver);

    //开启内置算法（避障和托盘对接）会自动关闭强度流和2D数据流，可以再次手动开启
    Check("LX_INT_ALGORITHM_MODE", DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, MODE_AVOID_OBSTACLE));
    //enable all stream
    Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, is_depth));
    Check("LX_BOOL_ENABLE_3D_AMP_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, is_amp));
    Check("LX_BOOL_ENABLE_2D_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, is_rgb));
    //check stream status
    Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM", DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM,  (bool*)&is_depth));
    Check("LX_BOOL_ENABLE_3D_AMP_STREAM", DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, (bool*)&is_amp));
    Check("LX_BOOL_ENABLE_2D_STREAM", DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM,  (bool*)&is_rgb));
    LxIntValueInfo int_value;
	Check("LX_INT_ALGORITHM_MODE", DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &int_value));
	inside_app = int_value.cur_value;
	if(inside_app) pub_app_info = this->create_publisher<std_msgs::msg::String>("LxCamera_App", 10);

    //get image params
    if (is_depth) {
        //binning，roi，对齐等操作会导致尺寸和内参改变，需要更新
        tof_camera_info.header.frame_id = "intrinsic_depth";

        LxIntValueInfo int_value;
        DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value);
        tof_camera_info.width = int_value.cur_value;
        DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value);
        tof_camera_info.height = int_value.cur_value;

        float* intr = nullptr;
        DcGetPtrValue(handle, LX_PTR_3D_INTRIC_PARAM, (void**)&intr);
        tof_camera_info.r [0] = intr[4];
        tof_camera_info.r [1] = intr[5];
        tof_camera_info.r [2] = intr[6];
        tof_camera_info.r [3] = intr[7];

        tof_camera_info.k [0] = intr[0];
        tof_camera_info.k [1] = 0;
        tof_camera_info.k [2] = intr[2];
        tof_camera_info.k [3] = 0;
        tof_camera_info.k [4] = intr[1];
        tof_camera_info.k [5] = intr[3];
        tof_camera_info.k [6] = 0;
        tof_camera_info.k [7] = 0;
        tof_camera_info.k [8] = 1;

        pub_tof_info->publish(tof_camera_info);
    }

    if (is_rgb) {
        rgb_camera_info.header.frame_id = "intrinsic_rgb";

        LxIntValueInfo int_value;
        DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value);
        rgb_camera_info.width = int_value.cur_value;
        DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value);
        rgb_camera_info.height = int_value.cur_value;

        float* intr = nullptr;
        DcGetPtrValue(handle, LX_PTR_2D_INTRIC_PARAM, (void**)&intr);
        tof_camera_info.r [0] = intr[4];
        tof_camera_info.r [1] = intr[5];
        tof_camera_info.r [2] = intr[6];
        tof_camera_info.r [3] = intr[7];

        tof_camera_info.k [0] = intr[0];
        tof_camera_info.k [1] = 0;
        tof_camera_info.k [2] = intr[2];
        tof_camera_info.k [3] = 0;
        tof_camera_info.k [4] = intr[1];
        tof_camera_info.k [5] = intr[3];
        tof_camera_info.k [6] = 0;
        tof_camera_info.k [7] = 0;
        tof_camera_info.k [8] = 1;

        pub_rgb_info->publish(rgb_camera_info);
    }

    if (LX_SUCCESS != Check("START_STREAM", DcStartStream(handle))) abort();
    run();
}

LxCamera::~LxCamera(){
    DcStopStream(handle);
    DcCloseDevice(handle);
}

void LxCamera::run()
{
    rclcpp::Node::SharedPtr node_(this);
    LxIntValueInfo int_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
    int rgb_type = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value);
    int rgb_channel = int_value.cur_value;

    while (rclcpp::ok())
    {
        if (LX_SUCCESS != Check("LX_CMD_GET_NEW_FRAME", DcSetCmd(handle, LX_CMD_GET_NEW_FRAME))){
            RCLCPP_WARN(this->get_logger(), "%s", std::string("get new frame failed").c_str());
            continue;
        }

        if (is_depth){
            void* dep_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &dep_data) == LX_SUCCESS){
                cv::Mat dep_img(tof_camera_info.height, tof_camera_info.width, CV_16UC1, dep_data);

                sensor_msgs::msg::Image msg_depth;
                cv_bridge::CvImage cv_img;
                cv_img.encoding = "16UC1";
                cv_img.header.stamp = this->now();
                cv_img.image = dep_img;
                cv_img.toImageMsg(msg_depth);
                pub_depth->publish(msg_depth);
            }
            else RCLCPP_WARN(this->get_logger(), "%s", std::string("Depth image is empty!").c_str());

            float* xyz_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_XYZ_DATA, (void**)&xyz_data) == LX_SUCCESS){
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (long i = 0; i < tof_camera_info.width * tof_camera_info.height; i++){
                    long index = 3 * i;
                    pcl::PointXYZ pt;
                    pt.x = xyz_data[index];
                    pt.y = xyz_data[index + 1];
                    pt.z = xyz_data[index + 2];
                    if (pt.z < 0.001) continue;

                    pcl_cloud->points.push_back(pt);
                }

                sensor_msgs::msg::PointCloud2 pcl2;
                pcl::toROSMsg(*pcl_cloud, pcl2);
                pcl2.header.stamp = this->now();
                pcl2.header.frame_id = "LxCamera_xyz";
                pub_xyz->publish(pcl2);
            }
            else RCLCPP_WARN(this->get_logger(), "%s", std::string("Cloud point data is empty!").c_str());
        }

        if (is_amp){
            void* amp_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_3D_AMP_DATA, &amp_data) == LX_SUCCESS)
            {
                cv::Mat amp_img(tof_camera_info.height, tof_camera_info.width, CV_16UC1, amp_data);

                sensor_msgs::msg::Image msg_amp;
                cv_bridge::CvImage cv_img;
                cv_img.encoding = "16UC1";
                cv_img.header.stamp = this->now();
                cv_img.image = amp_img;
                cv_img.toImageMsg(msg_amp);
                pub_amp->publish(msg_amp);
            }
            else RCLCPP_WARN(this->get_logger(), "%s", std::string("Amplitude image is empty!").c_str());
        }

        if (is_rgb){
            void* rgb_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data) == LX_SUCCESS)
            {
                cv::Mat rgb_img(rgb_camera_info.height, rgb_camera_info.width, CV_MAKETYPE(rgb_type, rgb_channel), rgb_data);
                cv::Mat rgb_pub;
                rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);

                std::string type = rgb_channel == 3 ? "8UC3" : "8UC1";
                sensor_msgs::msg::Image msg_rgb;
                cv_bridge::CvImage cv_img;
                cv_img.encoding = type;
                cv_img.header.stamp = this->now();
                cv_img.image = rgb_pub;
                cv_img.toImageMsg(msg_rgb);
                pub_rgb->publish(msg_rgb);
            }
            else RCLCPP_WARN(this->get_logger(), "%s", std::string("RGB image is empty!").c_str());
        }

        if (inside_app == MODE_AVOID_OBSTACLE){
            //获取避障输出IO
            int value = 0;
            DcSpecialControl(handle, "GetObstacleIO", (void*)&value);
            //获取避障输出结构体，参考LxAvoidanceOutput
            void* app_ptr = nullptr;
            DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &app_ptr);

            //可以自定义发布内容
            std_msgs::msg::String str;
            str.data = std::to_string(value);
            pub_app_info->publish(str);
        }
        else if (inside_app == MODE_PALLET_LOCATE){            
            //获取托盘对接输出结构体，参考LxPalletPose
            void* app_ptr = nullptr;
            DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &app_ptr);

            //可以自定义发布内容
            std_msgs::msg::String str;
            str.data = "";
            pub_app_info->publish(str);
        }

        rclcpp::spin_some(node_);
    }
}

