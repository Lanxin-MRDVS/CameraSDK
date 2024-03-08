//application_obstacle.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <sstream>
#include <thread>
#include <string.h>
#include "lx_camera_api.h"
#include "lx_camera_application.h"
using namespace std;

#define checkTC(state) {LX_STATE val=state;                            \
    if(val != LX_SUCCESS){                                             \
        if(val == LX_E_RECONNECTING){                                  \
            std::cout << " device reconnecting" << std::endl;}         \
        else{                                                          \
            std::cout << DcGetErrorString(val)<<std::endl;             \
            std::cout << " press any key to exit!" << std::endl;       \
            DcCloseDevice(handle);                                     \
            getchar();                                                 \
            return -1;                                                 \
        }                                                              \
    }                                                                  \
}

int PrintData(void* data_ptr);

int main(int argc, char** argv)
{
    DcHandle handle = 0;
    checkTC(DcSetInfoOutput(1, false, "./"));
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
        open_param = "192.168.100.12";
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

    checkTC(DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, MODE_AVOID_OBSTACLE));

    //算法版本号
    char* algor_ver = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_VERSION, &algor_ver));
    if (algor_ver != nullptr) std::cout << " current algor version:" << algor_ver << std::endl;

    //算法参数，与当前设置的算法有关
    char* cur_algor_json = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &cur_algor_json));
    if (cur_algor_json != nullptr) std::cout << " current algor json param:" << cur_algor_json << std::endl;

    //根据需要，修改算法参数，为json格式，可通过json库也可以通过string直接修改，也可以通过文件中转
    //以下为该算法json范例，其中ZoneIndex为当前生效的避障模式索引，对应Zones中多个模式，Zones中的单个模式中区分危险区域和预警区域坐标。
    std::string obstacle_json_param = "{ \
        \"R\": [0.00000002842, 0.00000119, 1.00000119, -1.00000011920, 0.0000119209, 0.0000011920931, -0.00002384, -1.00000011920, 0.0000119209],\
        \"T\": [400, 0, 120],\
        \"ZoneIndex\": 0,\
        \"Zones\": {\
            \"0\": {\
                \"DangerZone\": [400, 700, -500, 500, -50, 500],\
                \"WarningZone\": [700, 1300, -500, 500, -50, 500]\
            },\
            \"1\": {\
                \"DangerZone\": [400, 700, -400, 400, -50, 500],\
                \"WarningZone\": [700, 1300, -400, 400, -50, 500]\
            }\
        },\
        \"confidences\": [0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999, 0.29999999999999999],\
        \"debug\": false,\
        \"distacne\": 0,\
        \"ranging\": false,\
        \"semantics\": true,\
        \"setDetectGroundPlane\": [20, 500, 40, 0, 150],\
        \"setDownsampling\": [30, 30, 30],\
        \"setRadiusFilter\": [-1, 1],\
        \"setRange\": [0, 4000, -500, 500, -50, 500]\
        }";
    //修改避障参数，建议通过上位机LxCameraViewer修改。S2仅支持上位机修改！！！
    //checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, obstacle_json_param.c_str()));

    //仅切换避障参数可以通过以下接口实现。S2需要上位机配置好每套参数后才能切换，否则无效
    int pobstacle_mode = 1;//使用第二套参数
    checkTC(DcSpecialControl(handle, "SetObstacleMode", (void*)&pobstacle_mode));

    checkTC(DcStartStream(handle));

    //正常状态下，网络断开或者SDK关闭之后，相机会切换为待机状态。
    //如果算法结果不通过SDK输出（IO 串口 UDP协议等方式），需要设置为常开模式，相机和内置算法会始终保持工作
    checkTC(DcSetIntValue(handle, LX_INT_WORK_MODE, 1));

    while (true)
    {
        //刷新数据
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

        if (device_info.dev_type == LX_DEVICE_M4
            || device_info.dev_type == LX_DEVICE_M4Pro)
        {
            //获取数据
            void* algordata = nullptr;
            checkTC(DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &algordata));
            PrintData(algordata);
        }
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    getchar();
    return 0;
}

int PrintData(void* data_ptr)
{
    if (data_ptr == nullptr)
        return 0;

    std::stringstream algor_ss;
    LxAvoidanceOutput* obstacle = (LxAvoidanceOutput*)data_ptr;
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
        }
    }
    
    std::cout << algor_ss.str() << std::endl;
    return 0;
}
