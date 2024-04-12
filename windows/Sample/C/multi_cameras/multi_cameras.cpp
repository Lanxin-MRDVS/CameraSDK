﻿// multi_camera.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
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
using namespace cv;
#endif
using namespace std;

static char wait_key = '0';
#define checkTC(state, handle) {LX_STATE val=state;                     \
    if(val != LX_SUCCESS){                                              \
        if(val == LX_E_RECONNECTING){                                   \
            std::cout << " device reconnecting" << std::endl;}          \
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
    //日志
    checkTC(DcSetInfoOutput(1, true, "./"), 0);
    std::cout << " call api version:" << DcGetApiVersion() << std::endl;
    //查找设备
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

    //开启多个线程
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
    //开启相机
    DcHandle handle = 0;
    LxDeviceInfo device_info;
    checkTC(DcOpenDevice(OPEN_BY_IP, ip.c_str(), &handle, &device_info), handle);

    std::cout << "dev cameraid:" << device_info.id << ", uniqueid:" << handle
        << ", cameraip:" << device_info.ip << ", firmware_ver:" << device_info.firmware_ver << ", sn:" << device_info.sn
        << ", name:" << device_info.name << " ,img_algor_ver:" << device_info.algor_ver << std::endl;
 
    //数据流设置，开启太多可能会存在带宽不够的问题
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, true), handle);
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false), handle);
    bool rgb_enable = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &rgb_enable), handle);
    //开启数据流
    checkTC(DcStartStream(handle), handle);
    //tof参数
    LxIntValueInfo int_value;
    int tof_width = 0, tof_height = 0, tof_data_type = LX_DATA_UNSIGNED_SHORT;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value), handle);
    tof_width = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value), handle);
    tof_height = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_DEPTH_DATA_TYPE, &int_value), handle);
    tof_data_type = int_value.cur_value;
    //rgb参数
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
        //刷新数据
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if ((LX_SUCCESS != ret)
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            && (LX_E_FRAME_MULTI_MACHINE != ret))
            continue;

        //获取深度数据
        void* depth_data_ptr = nullptr;
        checkTC(DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &depth_data_ptr), handle);
#ifdef HAS_OPENCV
        cv::Mat depth_image = cv::Mat(tof_height, tof_width, CV_MAKETYPE(tof_data_type, 1), depth_data_ptr);
        cv::Mat depth_show;
        depth_image.convertTo(depth_show, CV_8U, 1.0 / 16);
        cv::namedWindow("depth" + ip, 0);
        cv::resizeWindow("depth" + ip, 640, 480);
        cv::imshow("depth" + ip, depth_show);
        wait_key = cv::waitKey(1);
#endif
        //获取rgb数据
        if (rgb_enable) {
            void* rgb_data_ptr = nullptr;
            checkTC(DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data_ptr), handle);
#ifdef HAS_OPENCV
            cv::namedWindow("rgb" + ip, 0);
            cv::resizeWindow("rgb" + ip, 640, 480);
            cv::Mat rgb_show = cv::Mat(rgb_height, rgb_width, CV_MAKETYPE(rgb_data_type, rgb_channles), rgb_data_ptr);
            cv::imshow("rgb" + ip, rgb_show);
            wait_key = cv::waitKey(1);
#endif
        }
        //帧率
        static auto _time = std::chrono::system_clock::now();
        auto _time2 = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(_time2 - _time).count() > 1)
        {
            _time = std::chrono::system_clock::now();
            LxFloatValueInfo float_info = { 0 };
            DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &float_info);
            std::cout << "Depth FPS: " << float_info.cur_value << std::endl;
        }
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    return 0;
}
