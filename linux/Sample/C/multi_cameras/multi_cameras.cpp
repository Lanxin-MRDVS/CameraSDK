// multi_camera.cpp : This file contains the main function. Program execution begins and ends here.
//
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <string.h>
#include <vector>
#include "lx_camera_api.h"
#ifdef HAS_OPENCV
#include "opencv2/opencv.hpp"
#define ENABLE_VISION
using namespace cv;
#endif
using namespace std;

static char wait_key = '0';
#define checkTC(state, handle) {LX_STATE val=state;                     \
    if(val != LX_SUCCESS){                                              \
        if(val == LX_E_RECONNECTING){                                   \
            std::cout << " device reconnecting" << std::endl;}          \
        else if(val == LX_E_NOT_SUPPORT){                               \
            std::cout << "device not support" << std::endl;}            \
        else{                                                           \
            std::cout << DcGetErrorString(val)<<std::endl;              \
            std::cout << " press any key to exit!" << std::endl;        \
            DcCloseDevice(handle);                                      \
            wait_key = getchar();                                       \
            return -1;                                                  \
        }                                                               \
    }                                                                   \
}

int thread_work(std::string ip);

int main(int argc, char** argv)
{
    // Log
    checkTC(DcSetInfoOutput(1, false, "./"), 0);
    std::cout << " call api version:" << DcGetApiVersion() << std::endl;
    // Find devices
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num), 0);
    if (device_num <= 0)
    {
        std::cout << "DcGetDeviceList success, but found no any device, press any key to exit" << std::endl;
        getchar();
        return -1;
    }
    std::cout << "DcGetDeviceList success list: " << device_num << std::endl << std::endl;

    // Start multiple threads
    std::vector<std::thread> threads_vec;
    for (auto device_index = 0; device_index < device_num; device_index++)
    {
        LxDeviceInfo device_info = p_device_list[device_index];
        std::cout << "open device:" << device_info.id << " " << device_info.ip << std::endl;

        threads_vec.push_back(std::thread(thread_work, device_info.ip));
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    printf("**********press 'q' to exit********\n\n");
    while (wait_key != 'q')
    {
        wait_key = getchar();
    }

    for (auto& t : threads_vec)
    {
        t.join();
    }
    return 0;
}

int thread_work(std::string ip)
{
    // Open camera
    DcHandle handle = 0;
    LxDeviceInfo device_info;
    checkTC(DcOpenDevice(OPEN_BY_IP, ip.c_str(), &handle, &device_info), handle);

    std::cout << "dev cameraid:" << device_info.id << ", uniqueid:" << handle
        << ", cameraip:" << device_info.ip << ", firmware_ver:" << device_info.firmware_ver << ", sn:" << device_info.sn
        << ", name:" << device_info.name << " ,img_algor_ver:" << device_info.algor_ver << std::endl;
 
    // Stream settings; enabling too many streams may cause bandwidth issues
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, true), handle);
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false), handle);
    bool rgb_enable = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &rgb_enable), handle);
    // Start stream
    checkTC(DcStartStream(handle), handle);
    // TOF parameters
    LxIntValueInfo int_value;
    int tof_width = 0, tof_height = 0, tof_data_type = LX_DATA_UNSIGNED_SHORT;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value), handle);
    tof_width = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value), handle);
    tof_height = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_DEPTH_DATA_TYPE, &int_value), handle);
    tof_data_type = int_value.cur_value;
    // RGB parameters
    int rgb_width = 0, rgb_height = 0, rgb_channles = 0, rgb_data_type = LX_DATA_UNSIGNED_CHAR;
    if (rgb_enable) {
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value), handle);
        rgb_width = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value), handle);
        rgb_height = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value), handle);
        rgb_channles = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value), handle);
        rgb_data_type = int_value.cur_value;
    }

    while (true)
    {
        if (wait_key == 'q')
            break;
        // Refresh data
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if ((LX_SUCCESS != ret)
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            && (LX_E_FRAME_MULTI_MACHINE != ret))
            continue;

        // Get depth data
        void* depth_data_ptr = nullptr;
        checkTC(DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &depth_data_ptr), handle);
#ifdef HAS_OPENCV
        cv::Mat depth_image = cv::Mat(tof_height, tof_width, CV_MAKETYPE(tof_data_type, 1), depth_data_ptr);
        cv::Mat depth_show;
#ifdef ENABLE_VISION
        depth_image.convertTo(depth_show, CV_8U, 1.0 / 16);
        cv::namedWindow("depth" + ip, 0);
        cv::resizeWindow("depth" + ip, 640, 480);
        cv::imshow("depth" + ip, depth_show);
        cv::waitKey(1);
#endif
#endif
        // Get RGB data
        if (rgb_enable) {
            void* rgb_data_ptr = nullptr;
            checkTC(DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data_ptr), handle);
#ifdef HAS_OPENCV
            cv::Mat rgb_show = cv::Mat(rgb_height, rgb_width, CV_MAKETYPE(rgb_data_type, rgb_channles), rgb_data_ptr);
#ifdef ENABLE_VISION
            cv::namedWindow("rgb" + ip, 0);
            cv::resizeWindow("rgb" + ip, 640, 480);
            cv::imshow("rgb" + ip, rgb_show);
            cv::waitKey(1);
#endif
#endif
        }
        // Frame rate
        static auto _time = std::chrono::system_clock::now();
        auto _time2 = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(_time2 - _time).count() > 1)
        {
            _time = std::chrono::system_clock::now();
            LxFloatValueInfo float_info = { 0 };
            DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &float_info);
            std::cout << "Depth FPS: " << float_info.cur_value;
            DcGetFloatValue(handle, LX_FLOAT_2D_IMAGE_FPS, &float_info);
            std::cout << " Rgb FPS: " << float_info.cur_value << std::endl;
        }
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    return 0;
}
