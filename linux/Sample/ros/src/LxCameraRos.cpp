//
// Created by root on 3/14/23.
//

#include "LxCameraRos.h"
#include "lx_camera_api.h"
#include "lx_camera_application.h"
#include "json.hpp"
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <sensor_msgs/PointCloud2.h>

static DcHandle handle;
static ros::Publisher pub_error;
static ros::Publisher pub_message;

int Check(std::string command, LX_STATE state)
{
    if (LX_SUCCESS != state) {        
        const char* m = DcGetErrorString(state);
        std_msgs::String msg;
        setlocale(LC_ALL, "");
        msg.data = "#command: " + command + " #error code: " + std::to_string(state) + " #report: " + m;
        pub_error.publish(msg);
        ROS_ERROR("%s", msg.data.c_str());
    }
    return state;
}

void LxCamera_Error(const std_msgs::String::ConstPtr& msg)
{
    ROS_ERROR("%s", msg->data.c_str());
}

void LxCamera_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::imshow("RGB", img);
    cv::waitKey(1);
}

void LxCamera_amp(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::imshow("AMP", img);
    cv::waitKey(1);
}

void LxCamera_dep(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::imshow("DEPTH", img);
    cv::waitKey(1);
}

//相机交互，通过json字符串，包含键值"execute"，"command"，"value"，"result"
void LxCamera::LxCamera_Command(const std_msgs::String::ConstPtr& msg)
{
    ROS_WARN("%s", msg->data.c_str());
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
        std_msgs::String str;
        str.data = message.dump();
        pub_message.publish(str);
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
        std_msgs::String str;
        str.data = message.dump();
        pub_message.publish(str);
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
			
			std_msgs::String str;
			str.data = message.dump();
			pub_message.publish(str);
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("%s", e.what());
			return;
		}
    }
}

LxCamera::LxCamera()
{
    ros::NodeHandle nh("~");
    ROS_INFO("Api version: %s", DcGetApiVersion());

    //log
    std::string log_path;
    nh.param<std::string>("log_path", log_path, "./");
    Check("SET_LOG", DcSetInfoOutput(1, true, log_path.c_str()));
    ROS_INFO("Log file path: %s", log_path.c_str());

    nh.param<int>("is_xyz", is_xyz, 1);
    nh.param<int>("is_depth", is_depth, 0);
    nh.param<int>("is_amp", is_amp, 0);
    nh.param<int>("is_rgb", is_rgb, 0);
    nh.param<int>("application", inside_app, 0);
	
	ROS_INFO("publish xyz: %d", is_xyz);
	ROS_INFO("publish depth: %d", is_depth);
	ROS_INFO("publish amp: %d", is_amp);
	ROS_INFO("publish rgb: %d", is_rgb);
	ROS_INFO("application mode: %d", inside_app);
	
    //advertise
    image_transport::ImageTransport it(nh);
	if(is_rgb){
		pub_rgb = it.advertise("LxCamera_Rgb", 1);
		pub_rgb_info = nh.advertise<sensor_msgs::CameraInfo>("LxCamera_RgbInfo", 1);
	}
	if(is_amp)
		pub_amp = it.advertise("LxCamera_Amp", 1);
	if(is_depth){
		pub_depth = it.advertise("LxCamera_Depth", 1);
		pub_tof_info = nh.advertise<sensor_msgs::CameraInfo>("LxCamera_TofInfo", 1);
	}
	if(is_xyz){
		pub_xyz = it.advertise("LxCamera_xyz", 1);
		pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("LxCamera_Cloud", 1);
	}
	if(inside_app)
		pub_app_info = nh.advertise<std_msgs::String>("LxCamera_App", 1);
    pub_message = nh.advertise<std_msgs::String>("LxCamera_Message", 1);
    pub_error = nh.advertise<std_msgs::String>("LxCamera_Error", 1);

    //subscribe
    ros::Subscriber err = nh.subscribe("LxCamera_Error", 10, LxCamera_Error);
    ros::Subscriber com = nh.subscribe("LxCamera_Command", 10, LxCamera_Command);

	int is_show;
    nh.param<int>("is_show", is_show, 0);
	ROS_INFO("is_show: %d", is_show);
    if (is_show) {
		if(is_rgb)
			static image_transport::Subscriber rgb = it.subscribe("LxCamera_Rgb", 10, LxCamera_rgb);
		if(is_amp)
			static image_transport::Subscriber amp = it.subscribe("LxCamera_Amp", 10, LxCamera_amp);
		if(is_depth)
			static image_transport::Subscriber dep = it.subscribe("LxCamera_Depth", 10, LxCamera_dep);
    }

    //find device
    LxDeviceInfo* devlist = nullptr;
    int devnum = 0;
    while (true)
    {
        Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
        if (devnum)
            break;
        ROS_ERROR("Not Found Device. Retry...");
        sleep(1);
    }

    //open device
    std::string ip;
    nh.param<std::string>("ip", ip, "");
    LX_OPEN_MODE open_mode = LX_OPEN_MODE::OPEN_BY_IP;
    if (ip.empty())
    {
        open_mode = LX_OPEN_MODE::OPEN_BY_INDEX;
        ip = "0";
    }

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

    //enable obstacle algrithm
    //开启内置算法（避障和托盘对接）会自动关闭强度流和2D数据流，可以再次手动开启
    Check("LX_INT_ALGORITHM_MODE", DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, inside_app));
    //enable all stream
    Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, is_xyz || is_depth));
    Check("LX_BOOL_ENABLE_3D_AMP_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, is_amp));
    Check("LX_BOOL_ENABLE_2D_STREAM", DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, is_rgb));
    //check stream status
	bool is_enable_depth = false, is_enable_amp = false, is_enable_rgb = false;
    Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM", DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &is_enable_depth));
	is_depth = is_depth && is_enable_depth;
	is_xyz = is_xyz && is_enable_depth;
    Check("LX_BOOL_ENABLE_3D_AMP_STREAM", DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &is_enable_amp));
	is_amp = is_enable_amp;
    Check("LX_BOOL_ENABLE_2D_STREAM", DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &is_enable_rgb));
	is_rgb = is_enable_rgb;
    LxIntValueInfo int_value;
	Check("LX_INT_ALGORITHM_MODE", DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &int_value));

	DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value);
	tof_camera_info.width = int_value.cur_value;
	DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value);
	tof_camera_info.height = int_value.cur_value;

	inside_app = int_value.cur_value;

    //get image params
    //binning，roi，对齐等操作会导致尺寸和内参改变，需要更新
    if (is_depth) {
		tof_camera_info.header.frame_id = "intrinsic_depth";

        float* intr = nullptr;
        DcGetPtrValue(handle, LX_PTR_3D_INTRIC_PARAM, (void**)&intr);
        tof_camera_info.D = std::vector<double>{ intr[4], intr[5], intr[6], intr[7], intr[8] };
        tof_camera_info.K = boost::array<double, 9>{intr[0], 0, intr[2],
            0, intr[1], intr[3],
            0, 0, 1};
        pub_tof_info.publish(tof_camera_info);
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
        rgb_camera_info.D = std::vector<double>{ intr[4], intr[5], intr[6], intr[7], intr[8] };
        rgb_camera_info.K = boost::array<double, 9>{intr[0], 0, intr[2],
            0, intr[1], intr[3],
            0, 0, 1};

        pub_rgb_info.publish(rgb_camera_info);
    }

    if (LX_SUCCESS != Check("START_STREAM", DcStartStream(handle)))
        abort();

    run();
}

LxCamera::~LxCamera()
{
    DcStopStream(handle);
    DcCloseDevice(handle);
}

void LxCamera::run()
{
    LxIntValueInfo int_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
    int rgb_type = int_value.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value);
    int rgb_channel = int_value.cur_value;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (LX_SUCCESS != Check("LX_CMD_GET_NEW_FRAME", DcSetCmd(handle, LX_CMD_GET_NEW_FRAME)))
        {
            ROS_WARN("%s", std::string("get new frame failed").c_str());
            continue;
        }

		if(is_xyz){
            float* xyz_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_XYZ_DATA, (void**)&xyz_data) == LX_SUCCESS)
            {
				{
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl_cloud->points.resize(tof_camera_info.width * tof_camera_info.height);
                for (long i = 0; i < tof_camera_info.width * tof_camera_info.height; i++)
                {
                    long index = 3 * i;						
                    pcl_cloud->points[i].x = xyz_data[index];
                    pcl_cloud->points[i].y = xyz_data[index + 1];
                    pcl_cloud->points[i].z = xyz_data[index + 2];
                }
				pcl_cloud->width = pcl_cloud->size();
				pcl_cloud->height = 1;
				
                sensor_msgs::PointCloud2 pcl2;
                pcl::toROSMsg(*pcl_cloud, pcl2);
                pcl2.header.stamp = ros::Time::now();
                pcl2.header.frame_id = "LxCamera_xyz";
                pub_cloud.publish(pcl2);
				}
				
				{
                cv::Mat xyz_img(tof_camera_info.height, tof_camera_info.width, CV_32FC3, xyz_data);
                sensor_msgs::ImagePtr msg_xyz = cv_bridge::CvImage(std_msgs::Header(), "32FC3", xyz_img).toImageMsg();
                pub_xyz.publish(msg_xyz);
				}				
            }
            else ROS_WARN("%s", std::string("Cloud point data is empty!").c_str());			
		}
		
        if (is_depth)
        {
            void* dep_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &dep_data) == LX_SUCCESS)
            {
                cv::Mat dep_img(tof_camera_info.height, tof_camera_info.width, CV_16UC1, dep_data);
                sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), "16UC1", dep_img).toImageMsg();
                pub_depth.publish(msg_depth);
            }
            else ROS_WARN("%s", std::string("Depth image is empty!").c_str());
        }

        if (is_amp)
        {
            void* amp_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_3D_AMP_DATA, &amp_data) == LX_SUCCESS)
            {
                cv::Mat amp_img(tof_camera_info.height, tof_camera_info.width, CV_16UC1, amp_data);
                sensor_msgs::ImagePtr msg_amp = cv_bridge::CvImage(std_msgs::Header(), "16UC1", amp_img).toImageMsg();
                pub_amp.publish(msg_amp);
            }
            else ROS_WARN("%s", std::string("Amplitude image is empty!").c_str());
        }

        if (is_rgb)
        {
            void* rgb_data = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data) == LX_SUCCESS)
            {
                cv::Mat rgb_img(rgb_camera_info.height, rgb_camera_info.width, CV_MAKETYPE(rgb_type, rgb_channel), rgb_data);
                cv::Mat rgb_pub;
                rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type == CV_16U ? 0.25 : 1);

                std::string type = rgb_channel == 3 ? "8UC3" : "8UC1";
                sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), type, rgb_pub).toImageMsg();
                pub_rgb.publish(msg_rgb);
            }
            else ROS_WARN("%s", std::string("RGB image is empty!").c_str());
        }

        if (inside_app == MODE_AVOID_OBSTACLE)
        {
            //获取避障输出IO
            int value = 0;
            DcSpecialControl(handle, "GetObstacleIO", (void*)&value);
            //获取避障输出结构体，参考LxAvoidanceOutput
            void* app_ptr = nullptr;
            DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &app_ptr);

            //可以自定义发布内容
            std_msgs::String str;
            str.data = std::to_string(value);
            pub_app_info.publish(str);
        }
        else if (inside_app == MODE_PALLET_LOCATE)
        {            
            //获取托盘对接输出结构体，参考LxPalletPose
            void* app_ptr = nullptr;
            DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &app_ptr);

            //可以自定义发布内容
            std_msgs::String str;
            str.data = "";
            pub_app_info.publish(str);
        }

        ros::spinOnce();
		loop_rate.sleep();
    }
}
