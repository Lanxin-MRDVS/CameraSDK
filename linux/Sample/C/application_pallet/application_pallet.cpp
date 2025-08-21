//application_pallet.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <sstream>
#include <thread>
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
    
    //开启托盘对接算法。建议关闭其他数据流，避免LX_CMD_GET_NEW_FRAME时算法数据未更新
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, false));
    checkTC(DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, MODE_PALLET_LOCATE));

    LxIntValueInfo algor_info = { 0 };
    checkTC(DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &algor_info));
    std::cout << " current LX_INT_ALGORITHM_MODE:" << algor_info.cur_value << std::endl;

    //算法版本号
    char* algor_ver = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_VERSION, &algor_ver));
    if (algor_ver != nullptr) std::cout << " current algor version:" << algor_ver << std::endl;

    //托盘算法参数可以不设置。如果需要可通过如下函数设置，参数为json格式
    //std::string str_pallet_json = "{}";
    //checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, str_pallet_json.c_str()));

    //算法参数，与当前设置的算法有关
    char* cur_algor_json = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &cur_algor_json));
    if (cur_algor_json != nullptr) std::cout << " current algor json param:" << cur_algor_json << std::endl;

    //正常状态下，网络断开或者SDK关闭之后，相机会切换为待机状态。
    //如果算法结果不通过SDK输出，需要设置为常开模式，相机和内置算法会始终保持工作。此时启停流无效，不允许需要停流才能进行的操作
    //checkTC(DcSetIntValue(handle, LX_INT_WORK_MODE, 1));

    //开启数据流
    checkTC(DcStartStream(handle));
    while (true)
    {
        //刷新数据
        //如果开启了多个数据流，只要有一个更新，此函数会正常返回。
        //可以设置LX_BOOL_ENABLE_SYNC_FRAME为true保持所有数据同步。或者通过帧ID和时间戳判断需要的数据是否更新
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if ((LX_SUCCESS != ret)
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            && (LX_E_FRAME_MULTI_MACHINE != ret))
        {            
            std::cout << " DcSetCmd LX_CMD_GET_NEW_FRAME failed" << std::endl;

            if (LX_E_RECONNECTING == ret) {
                std::cout << "device is reconnecting" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            continue;
        }

        //获取数据
        void* algordata = nullptr;
        checkTC(DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &algordata));
        PrintData(algordata);
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    return 0;
}

int PrintData(void* data_ptr)
{
    if (data_ptr == nullptr)
        return 0;

    LxPalletPose* palletdata = (LxPalletPose*)data_ptr;
    std::cout << "palletdata ret:" << palletdata->return_val << " x:" << palletdata->x << " y:" << palletdata->y << " yaw:" << palletdata->yaw << std::endl;
    return 0;
}
