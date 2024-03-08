// single_camera.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <thread>
#include <sstream>
#include "lx_camera_api.h"

#ifdef HAS_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
#endif
using namespace std;

static char wait_key = '0';
static DcHandle handle = 0;

static int tof_width = 0;
static int tof_height = 0;
static int tof_depth_type = LX_DATA_TYPE::LX_DATA_UNSIGNED_SHORT;
static int tof_amp_type = LX_DATA_TYPE::LX_DATA_UNSIGNED_SHORT;
static int rgb_width = 0;
static int rgb_height = 0;
static int rgb_channles = 3;
static int rgb_data_type = LX_DATA_TYPE::LX_DATA_UNSIGNED_CHAR;

#define checkTC(state) {LX_STATE val=state;                            \
    if(val != LX_SUCCESS){                                             \
        if(val == LX_E_RECONNECTING){                                  \
            std::cout << " device reconnecting" << std::endl;}         \
        else if(val == LX_E_NOT_SUPPORT){                              \
            std::cout << " not support" << std::endl;}                 \
        else{                                                          \
            std::cout << DcGetErrorString(val)<<std::endl;             \
            std::cout << " press any key to exit!" << std::endl;       \
            DcCloseDevice(handle);                                     \
            wait_key = getchar();                                      \
            return -1;                                                 \
        }                                                              \
    }                                                                  \
}



int TestDepth(bool is_enable, DcHandle handle);
int TestAmp(bool is_enable, DcHandle handle);
int TestRgb(bool is_enable, DcHandle handle);
void WaitKey()
{
    printf("**********press 'q' to exit********\n\n");
    wait_key = getchar();
}

int main(int argc, char** argv)
{
    checkTC(DcSetInfoOutput(1, true, "./"));
    std::cout << " call api version:" << DcGetApiVersion() << std::endl;

    //查找相机
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num));
    if (device_num <= 0)
    {
        std::cout << "not found any device, press any key to exit" << std::endl;
        wait_key = getchar();
        return -1;
    }
    std::cout << "DcGetDeviceList success list: " << device_num << std::endl;

    //打开相机
    std::string open_param;
    int open_mode = OPEN_BY_INDEX;
    switch (open_mode)
    {
        //根据ip打开设备
    case OPEN_BY_IP:
        open_param = "192.168.100.82";
        break;
        //根据id打开设备
    case OPEN_BY_ID:
        open_param = "F13301122647";
        break;
        //根据sn打开设备
    case OPEN_BY_SN:
        open_param = "ccf8981cc50b66b6";
        break;
        //根据搜索设备列表索引打开设备
    case OPEN_BY_INDEX:
    default:
        open_param = "0";
        break;
    }

    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice((LX_OPEN_MODE)open_mode, open_param.c_str(), &handle, &device_info);
    if (LX_SUCCESS != lx_state) {
        std::cout << "open device failed, open_mode:" << open_mode << " open_param:" << open_param << " press any key to exit" << std::endl;
        wait_key = getchar();
        return -1;
    }

    std::cout << "device_info\n cameraid:" << device_info.id << "\n uniqueid:" << handle
        << "\n cameraip:" << device_info.ip << "\n firmware_ver:" << device_info.firmware_ver << "\n sn:" << device_info.sn
        << "\n name:" << device_info.name << "\n img_algor_ver:" << device_info.algor_ver << std::endl;

    //设置数据流
    bool test_depth = false, test_amp = false, test_rgb = false;
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, test_depth));
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, test_amp));
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, test_rgb));

    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &test_depth));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &test_amp));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &test_rgb));
    std::cout << "test_depth:" << test_depth << " test_amp:" << test_amp << " test_rgb:" << test_rgb << std::endl;

    //RGBD对齐，TOF的图像尺寸和像素会扩展到与RGB一致，开启后建议关闭强度流
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_TO_DEPTH, true));

    //获取图像参数，设置ROI BINNING RGBD对齐之后需要重新获取图像尺寸
    LxIntValueInfo int_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value));
    tof_width = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value));
    tof_height = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_DEPTH_DATA_TYPE, &int_value));
    tof_depth_type = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_AMPLITUDE_DATA_TYPE, &int_value));
    tof_amp_type = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value));
    rgb_width = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value));
    rgb_height = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value));
    rgb_channles = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value));
    rgb_data_type = int_value.cur_value;

    //可以根据需要,是否开启帧同步模式, 开启该模式, 内部会对每一帧做同步处理后返回
    //默认若不需要tof与rgb数据同步, 则不需要开启此功能, 内部会优先保证数据实时性
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_SYNC_FRAME, true));

    //开启数据流
    checkTC(DcStartStream(handle));

    std::thread pthread = std::thread(WaitKey);
    pthread.detach();

    auto _time = std::chrono::system_clock::now();
    while (true)
    {
        //更新数据
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if ((LX_SUCCESS != ret)
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            && (LX_E_FRAME_MULTI_MACHINE != ret))
        {
            if (LX_E_RECONNECTING == ret) {
                std::cout << "device reconnecting" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        //帧率
        auto _time2 = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(_time2 - _time).count() > 1)
        {
            _time = std::chrono::system_clock::now();

            LxFloatValueInfo dep_float_info = { 0 };
            LxFloatValueInfo rgb_float_info = { 0 };
            std::stringstream ss;
            if (test_depth)
            {
                checkTC(DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &dep_float_info));
                ss << "depth fps:" << dep_float_info.cur_value;
            }

            if (test_rgb)
            {
                checkTC(DcGetFloatValue(handle, LX_FLOAT_2D_IMAGE_FPS, &rgb_float_info));
                ss << " rgb fps:" << rgb_float_info.cur_value;
            }

            std::cout << ss.str() << std::endl;
        }

        TestDepth(test_depth, handle);
        TestAmp(test_amp, handle);
        TestRgb(test_rgb, handle);

        if (wait_key == 'q' || wait_key == 'Q')
            break;
    }

    DcStopStream(handle);
    DcCloseDevice(handle);

    return 0;
}


int TestDepth(bool is_enable, DcHandle handle)
{
    if (!is_enable)
        return 0;

    void* data_ptr = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &data_ptr));

    //第yRows行xCol列深度数据
    //TOF相机深度数据为unsigned short类型，结构光相机（LX_DEVICE_H3)为float类型
    //点云为float类型，xyz依次存储
    //LX_DATE_TYPE的定义与opencv CV_8U CV_16U CV_32F一致
    int yRow = 100, xCol = 100;
    int pose = yRow * tof_width + xCol;
    if (LX_DATA_UNSIGNED_SHORT == tof_depth_type)
    {
        unsigned short* data = (unsigned short*)data_ptr;
        unsigned short value = data[pose];
    }
    else if (LX_DATA_FLOAT == tof_depth_type)
    {
        float* data = (float*)data_ptr;
        float value = data[pose];
    }

    //第yRows行xCol列点云数据
    float* xyz_data = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_XYZ_DATA, (void**)&xyz_data));
    float x = xyz_data[pose * 3];
    float y = xyz_data[pose * 3 + 1];
    float z = xyz_data[pose * 3 + 2];

#ifdef HAS_OPENCV
    cv::Mat depth_image = cv::Mat(tof_height, tof_width, CV_MAKETYPE(tof_depth_type, 1), data_ptr);
    cv::Mat depth_show;
    depth_image.convertTo(depth_show, CV_8U, 1.0 / 16);
    cv::namedWindow("depth", 0);
    cv::resizeWindow("depth", 640, 480);
    cv::imshow("depth", depth_show);
    cv::waitKey(1);
#endif
    return 0;
}

int TestAmp(bool is_enable, DcHandle handle)
{
    if (!is_enable)
        return 0;

    void* data_ptr = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_3D_AMP_DATA, &data_ptr));

    //第yRows行xCol列数据
    //TOF相机强度数据为unsigned short类型，结构光相机（LX_DEVICE_WK)为unsigned char类型
    //LX_DATE_TYPE的定义与opencv CV_8U CV_16U CV_32F一致
    int yRow = 100, xCol = 100;
    int pose = yRow * tof_width + xCol;
    if (LX_DATA_UNSIGNED_SHORT == tof_amp_type)
    {
        unsigned short value = ((unsigned short*)data_ptr)[pose];
    }
    else if (LX_DATA_UNSIGNED_CHAR == tof_amp_type)
    {
        unsigned char value = ((unsigned char*)data_ptr)[pose];
    }

#ifdef HAS_OPENCV
    cv::Mat amp_image = cv::Mat(tof_height, tof_width, CV_MAKETYPE(tof_amp_type, 1), data_ptr);
    cv::Mat amp_show;
    amp_image.convertTo(amp_show, CV_8U, 0.25);
    cv::imshow("amptitude", amp_show);
    cv::waitKey(1);
#endif
    return 0;
}

int TestRgb(bool is_enable, DcHandle handle)
{
    if (!is_enable)
        return 0;

    unsigned char* data_ptr = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, (void**)&data_ptr));

    //第yRows行xCol列数据
    //2D图像目前只有unsigned char格式
    //LX_DATE_TYPE的定义与opencv CV_8U CV_16U CV_32F一致
    int yRow = 100, xCol = 100;
    int pose = yRow * rgb_width + xCol;
    if (1 == rgb_channles)
    {
        unsigned char value = data_ptr[pose];
    }
    else
    {
        unsigned char r = data_ptr[pose * 3];
        unsigned char g = data_ptr[pose * 3 + 1];
        unsigned char b = data_ptr[pose * 3 + 2];
    }

#ifdef HAS_OPENCV
    cv::Mat rgb_show = cv::Mat(rgb_height, rgb_width, CV_MAKETYPE(rgb_data_type, rgb_channles), data_ptr);
    cv::namedWindow("rgb", 0);
    cv::resizeWindow("rgb", 640, 480);
    cv::imshow("rgb", rgb_show);
    cv::waitKey(1);
#endif
    return 0;
}
