//application_obstacle_v2.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <sstream>
#include <thread>
#include <string.h>
#include <atomic>
#include "lx_camera_api.h"
#include "lx_camera_application.h"
using namespace std;

#define checkTC(state) {LX_STATE val=state;                            \
    if(val != LX_SUCCESS){                                             \
        if(val == LX_E_RECONNECTING){                                  \
            std::cout << " device reconnecting" << std::endl;}         \
        else if(val == LX_E_NOT_SUPPORT){                                  \
            std::cout << " device not support" << std::endl;}         \
        else{                                                          \
            std::cout << DcGetErrorString(val)<<std::endl;             \
            std::cout << " press any key to exit!" << std::endl;       \
            DcCloseDevice(handle);                                     \
            getchar();                                                 \
            return -1;                                                 \
        }                                                              \
    }                                                                  \
}

int PrintData(void* data_ptr, int obstacle_mode);

int main(int argc, char** argv)
{
    DcHandle handle = 0;
    checkTC(DcSetInfoOutput(1, true, "./"));
    std::cout << " call api version:" << DcGetApiVersion() << std::endl;

    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num));
    if (device_num <= 0)
    {
        std::cout << "not found any device, press any key to exit" << std::endl;
        getchar();
        return -1;
    }
    std::cout << "DcGetDeviceList success list: " << device_num << std::endl;

    std::string open_param;
    int open_mode = OPEN_BY_INDEX;
    switch (open_mode)
    {
        //根据ip打开设备
    case OPEN_BY_IP:
        open_param = "192.168.100.82";
        break;
        //根据sn打开设备
    case OPEN_BY_ID:
        open_param = "F13301122647";
        break;
        //根据搜索设备列表索引打开设备
    case OPEN_BY_INDEX:
    default:
        open_param = "0";
        break;
    }

    LxDeviceInfo device_info;
    checkTC(DcOpenDevice((LX_OPEN_MODE)open_mode, open_param.c_str(), &handle, &device_info));

    std::cout << "device_info\n cameraid:" << device_info.id << "\n uniqueid:" << handle
        << "\n cameraip:" << device_info.ip << "\n firmware_ver:" << device_info.firmware_ver << "\n sn:" << device_info.sn
        << "\n name:" << device_info.name << "\n img_algor_ver:" << device_info.algor_ver << std::endl;

    //应用算法
    LxIntValueInfo algor_info = { 0 };
    checkTC(DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &algor_info));
    std::cout << " current LX_INT_ALGORITHM_MODE:" << algor_info.cur_value << std::endl;

    int algor_mode = MODE_AVOID_OBSTACLE2;
    checkTC(DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, algor_mode));

    //算法版本号
    char* algor_ver = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_VERSION, &algor_ver));
    if (algor_ver != nullptr) std::cout << " current algor version:" << algor_ver << std::endl;

    //算法参数，与当前设置的算法有关
    char* cur_algor_json = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &cur_algor_json));
    if (cur_algor_json != nullptr) std::cout << " current algor json param:" << cur_algor_json << std::endl;

    //关闭3D深度图、强度图和2D图像流，以节省带宽。如果需要可以开启
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, false));

    checkTC(DcStartStream(handle));

    int pobstacle_index = 0;
    std::atomic<bool> stop_flag(false);
    std::thread timer_thread([&]() {
        while (!stop_flag) {
            if (pobstacle_index < 4)
                pobstacle_index++;
            else
            {
                pobstacle_index = 0;
            }
            //修改避障检测区域索引参数
            LX_STATE state = DcSpecialControl(handle, "SetObstacleMode", &pobstacle_index);
            if (state != LX_SUCCESS)
                std::cout << DcGetErrorString(state) << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });

    while (true)
    {
        //如果需要获取图像数据或者避障详细信息数据，就得通过LX_CMD_GET_NEW_FRAME刷新
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if ((LX_SUCCESS != ret)
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            && (LX_E_FRAME_MULTI_MACHINE != ret))
        {
            std::cout << " DcSetCmd LX_CMD_GET_NEW_FRAME failed" << std::endl;
            if (LX_E_RECONNECTING == ret) {
                std::cout << " device is reconnecting" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            continue;
        }

        //获取避障IO输出结果
        int obstacle_io_result;
        checkTC(DcSpecialControl(handle, "GetObstacleIO", (void*)&obstacle_io_result));
        std::cout << " obstacle io result:" << obstacle_io_result << std::endl;

        //获取避障详细信息数据
        void* algordata = nullptr;
        checkTC(DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &algordata));
        PrintData(algordata, algor_mode);
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    getchar();
    return 0;
}

int PrintData(void* data_ptr, int obstacle_mode)
{
    if (data_ptr == nullptr)
        return 0;

    std::stringstream algor_ss;
    if (obstacle_mode == MODE_AVOID_OBSTACLE2) {
        LxAvoidanceOutputN* obstacle = (LxAvoidanceOutputN*)data_ptr;
        if (obstacle->state != LxAvSuccess) {
            std::cout << " state is not success" << std::endl;
            return 0;
        }

        algor_ss << " groundPlane.a:" << obstacle->groundPlane.a << " number_3d:" << obstacle->number_3d << " number_box:" << obstacle->number_box << std::endl;

        if (obstacle->cloud_output != nullptr)
        {
            algor_ss << " cloud data "; //print 前10个
            auto cloud_num = obstacle->number_3d < 10 ? obstacle->number_3d : 10;
            for (int i = 0; i < (int)cloud_num; i++)
            {
                algor_ss << "index:" << i << " x:" << obstacle->cloud_output[i].x << " y:" << obstacle->cloud_output[i].y << " z:" << obstacle->cloud_output[i].z;
                algor_ss << " r:" << (int)obstacle->cloud_output[i].r << " g:" << (int)obstacle->cloud_output[i].g << " b:" << (int)obstacle->cloud_output[i].b;
            }
        }

        if (obstacle->obstacleBoxs != nullptr)
        {
            algor_ss << " obstacle data ";
            auto box_num = obstacle->number_box < 10 ? obstacle->number_box : 10;
            for (int i = 0; i < (int)box_num; i++)
            {
                algor_ss << "index:" << i << " width:" << obstacle->obstacleBoxs[i].width << " height:" << obstacle->obstacleBoxs[i].height << " depth:" << obstacle->obstacleBoxs[i].depth;
                algor_ss << " center:" << obstacle->obstacleBoxs[i].center[0] << " " << obstacle->obstacleBoxs[i].center[1] << " " << obstacle->obstacleBoxs[i].center[2];
                algor_ss << " type_idx:" << obstacle->obstacleBoxs[i].type_idx << " prev_id:" << obstacle->obstacleBoxs[i].prev_id;
                algor_ss << " box_2d_x_min:" << obstacle->obstacleBoxs[i].box_2d_x_min << " box_2d_y_min:" << obstacle->obstacleBoxs[i].box_2d_y_min;
                algor_ss << " box_2d_x_max:" << obstacle->obstacleBoxs[i].box_2d_x_max << " box_2d_y_max:" << obstacle->obstacleBoxs[i].box_2d_y_max;
                algor_ss << " type_name_len:" << obstacle->obstacleBoxs[i].type_name_len << " type_name:" << obstacle->obstacleBoxs[i].type_name;
            }
        }
    }

    std::cout << algor_ss.str() << std::endl;
    return 0;
}
