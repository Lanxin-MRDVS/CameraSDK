﻿// single_camera.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <string>
#include <thread>
#include "lx_camera_api.h"

//#undef HAS_OPENCV
#ifdef HAS_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
#endif
using namespace std;

static char wait_key = '0';
#define checkTC(state) {LX_STATE val=state;                            \
    if(val != LX_SUCCESS){                                             \
        if(val == LX_E_RECONNECTING){                                  \
            printf("device reconnecting\n"); }                         \
        else{                                                          \
            printf("%s\n", DcGetErrorString(val));                             \
            printf("press any key to exit!\n");                        \
            wait_key = getchar();                                      \
            DcCloseDevice(handle);                                     \
            return -1;                                                 \
        }                                                              \
    }                                                                  \
}

int TestFrame(DcHandle handle);
void WaitKey()
{
    printf("**********press 'q' to exit********\n\n");
    wait_key = getchar();
}

int main(int argc, char** argv)
{
    DcHandle handle = 0;
    printf("call api version: %s\n", DcGetApiVersion());

    //查找相机
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num));    
    if (device_num <= 0) 
    {
        printf("not found any device, press any key to exit\n");
        wait_key = getchar();
        return -1;
    }
    printf("DcGetDeviceList success list: %d\n", device_num);

    //打开相机。SDK基于GIGE协议，会搜索到所有支持GIGE协议的相机，用索引方式打开相机时需要注意
    //设备打开后会独占权限，其他进程无法再打开相机。如果程序强制结束没有调用DcCloseDevice，需要等待几秒等心跳超时释放权限
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
    if (LX_SUCCESS != lx_state){
        printf("open device failed, open_mode: %d open_param: %s\n.press any key to exit\n", open_mode, open_param.c_str());
        wait_key = getchar();
        return -1;
    }

    printf("device_info\nid: %s\nip: %s\nsn: %s\nfirmware_ver:%s\n",
        device_info.id, device_info.ip, device_info.sn, device_info.firmware_ver);

    //获取数据流开启状态，如需调整可以调用DcSetBoolValue。
    bool test_depth = false, test_amp = false, test_rgb = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &test_depth));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &test_amp));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &test_rgb));

    //RGBD对齐，rgb和depth的图像尺寸和坐标会保持一致。
    //提供两中对齐方式，以depth为基准和以rgb为基准。以rgb为基准时深度和强度图像坐标不一致。
    //checkTC(DcSetIntValue(handle, LX_INT_RGBD_ALIGN_MODE, LX_RGBD_ALIGN_MODE::DEPTH_TO_RGB));

    //可以根据需要,是否开启帧同步模式, 开启该模式, 内部会对每一帧做同步处理后返回
    //若不需要不同数据流之间同步, 则不需要开启此功能, 内部会优先保证数据实时性，也可以通过帧ID和时间戳判断是否同步
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_SYNC_FRAME, true));

    printf("test_depth: %d test_amp: %d test_rgb: %d\n", test_depth, test_amp, test_rgb);

    /*其他操作。相机的设置、参数的获取建议在数据流开启之前操作，尤其涉及到图像尺寸变化的内容*/

    //开启数据流
    checkTC(DcStartStream(handle));

    std::thread pthread = std::thread(WaitKey);
    pthread.detach();

    while (true)
    {
        //更新数据
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if ((LX_SUCCESS != ret)
            //开启帧同步时，在超时范围内没有找到匹配的帧（丢包），会返回LX_E_FRAME_ID_NOT_MATCH
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            //多机模式开启时，检测到多机干扰会返回LX_E_FRAME_MULTI_MACHINE
            && (LX_E_FRAME_MULTI_MACHINE != ret))
        {
            if (LX_E_RECONNECTING == ret) {
                printf("device reconnecting\n");
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        TestFrame(handle);

        if (wait_key == 'q' || wait_key == 'Q')
            break;
    }

    DcStopStream(handle);
    DcCloseDevice(handle);

    return 0;
}


int TestFrame(DcHandle handle)
{
    FrameInfo* frame_ptr = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_FRAME_DATA, (void**)&frame_ptr));
    if (!frame_ptr) {
        printf("new frame not correct, frame ptr is null\n");
        return 1;
    }

    if (frame_ptr->frame_state != LX_SUCCESS) {
        if (frame_ptr->frame_state != LX_E_FRAME_ID_NOT_MATCH)
            printf("frame id not match\n");
        else if(frame_ptr->frame_state != LX_E_FRAME_MULTI_MACHINE)
            printf("found multi machine signal\n");
        else {
            printf("new frame not correct\n");
            return 1;
        }
    }

    //帧ID信息，可以用来识别每个数据流的同步，否则可以忽略
    FrameExtendInfo* pextendframe = nullptr;
    if (frame_ptr->reserve_data != nullptr)
        pextendframe = (FrameExtendInfo*)frame_ptr->reserve_data;

    //深度数据
    if (frame_ptr->depth_data.frame_data != nullptr)
    {
        auto depth_data = frame_ptr->depth_data;
        printf("depth_sensor_time:%lld\n", depth_data.sensor_timestamp);
        if (pextendframe != nullptr)
            printf("depth_frame_id:%d\n", pextendframe->depth_frame_id);

        //将depth数据转换为xyz点云数据
        void* xyz = NULL;
        DcGetPtrValue(handle, LX_PTR_XYZ_DATA, &xyz);

#ifdef HAS_OPENCV
        cv::Mat depth_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_MAKETYPE(depth_data.frame_data_type, depth_data.frame_channel), depth_data.frame_data);
        cv::Mat xyz_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_32FC3, xyz);

        cv::Mat show;
        depth_image.convertTo(show, CV_8U, 1.0 / 16);
        applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("depth", 0);
        cv::resizeWindow("depth", 640, 480);
        cv::imshow("depth", show);
        cv::waitKey(1);
#endif
    }

    //强度数据
    if (frame_ptr->amp_data.frame_data != nullptr)
    {
        auto amp_data = frame_ptr->amp_data;
        printf("amp_sensor_time:%lld\n", amp_data.sensor_timestamp);
        if (pextendframe != nullptr)
            printf("amp_frame_id:%d\n", pextendframe->amp_frame_id);

#ifdef HAS_OPENCV
        cv::Mat amp_image = cv::Mat(amp_data.frame_height, amp_data.frame_width,
            CV_MAKETYPE(amp_data.frame_data_type, amp_data.frame_channel), amp_data.frame_data);

        cv::Mat show;
        if (amp_image.type() == CV_16U)
            amp_image.convertTo(show, CV_8U, 1.0 / 8);
        else
            show = amp_image;
        applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("amp", 0);
        cv::resizeWindow("amp", 640, 480);
        cv::imshow("amp", show);
        cv::waitKey(1);
#endif
    }

    //rgb数据
    if (frame_ptr->rgb_data.frame_data != nullptr)
    {
        auto rgb_data = frame_ptr->rgb_data;
        printf("rgb_sensor_time:%lld\n", rgb_data.sensor_timestamp);
        if (pextendframe != nullptr)
            printf("rgb_frame_id:%d\n", pextendframe->rgb_frame_id);

#ifdef HAS_OPENCV
        cv::Mat rgb_image = cv::Mat(rgb_data.frame_height, rgb_data.frame_width,
            CV_MAKETYPE(rgb_data.frame_data_type, rgb_data.frame_channel), rgb_data.frame_data);

        cv::namedWindow("rgb", 0);
        cv::resizeWindow("rgb", 640, 480);
        cv::imshow("rgb", rgb_image);
        cv::waitKey(1);
#endif
    }

    return 0;
}
