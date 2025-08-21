//application_obstacle_v1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
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

    int algor_mode = MODE_AVOID_OBSTACLE;
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

    //仅切换避障检测区域索引参数可以通过以下接口实现。S2,S10等使用避障V1方式的需要上位机配置好每套避障索引参数后才能切换，否则无效
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
			//注意：SetObstacleIndex是快速切换避障区域索引功能，SetObstacleMode是旧的全量配置切换速度较慢,所以这里建议使用SetObstacleIndex
            //MODE_AVOID_OBSTACLE避障可以支持SetObstacleIndex和SetObstacleMode,MODE_AVOID_OBSTACLE2只能使用SetObstacleMode
            LX_STATE state = DcSpecialControl(handle, "SetObstacleIndex", &pobstacle_index);

            int pobstacle_index_now = 0;
            state = DcSpecialControl(handle, "GetObstacleIndex", &pobstacle_index_now);

            if (state != LX_SUCCESS)
                std::cout << DcGetErrorString(state) << std::endl;

            if (pobstacle_index != pobstacle_index_now) {
                DcLog("pobstacle_index != pobstacle_index_now");
            }

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
     });

    while (true)
    {
        //获取避障IO输出结果
        int obstacle_io_result;
        LX_STATE state = DcSpecialControl(handle, "GetObstacleIO", (void*)&obstacle_io_result);
        if (state != LX_SUCCESS) {
            std::cout << DcGetErrorString(state) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            //todo：异常处理
        }
        else {
            std::cout << " obstacle io result:" << obstacle_io_result << std::endl;
        }
        //休眠，防止访问频繁访问设备
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    getchar();
    return 0;
}
