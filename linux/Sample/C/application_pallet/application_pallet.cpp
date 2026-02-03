//application_pallet.cpp : This file contains the main function. Program execution begins and ends here.
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
        // Open device by IP
    case OPEN_BY_IP:
        open_param = "192.168.100.82";
        break;
        // Open device by SN
    case OPEN_BY_ID:
        open_param = "F13301122647";
        break;
        // Open device by index in the discovered device list
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
    
    // Enable pallet docking algorithm. Disable other streams to avoid stale algorithm data on LX_CMD_GET_NEW_FRAME.
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, false));
    checkTC(DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, MODE_PALLET_LOCATE));

    LxIntValueInfo algor_info = { 0 };
    checkTC(DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &algor_info));
    std::cout << " current LX_INT_ALGORITHM_MODE:" << algor_info.cur_value << std::endl;

    // Algorithm version
    char* algor_ver = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_VERSION, &algor_ver));
    if (algor_ver != nullptr) std::cout << " current algor version:" << algor_ver << std::endl;

    // Pallet algorithm parameters are optional. If needed, set via the function below (JSON).
    //std::string str_pallet_json = "{}";
    //checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, str_pallet_json.c_str()));

    // Algorithm parameters (depend on current algorithm)
    char* cur_algor_json = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &cur_algor_json));
    if (cur_algor_json != nullptr) std::cout << " current algor json param:" << cur_algor_json << std::endl;

    // Normally, after network disconnect or SDK close, the camera enters standby.
    // If algorithm output is not read via SDK, set always-on mode to keep the camera and algorithms running.
    // In always-on mode, start/stop stream is ignored and operations requiring stop are not allowed.
    //checkTC(DcSetIntValue(handle, LX_INT_WORK_MODE, 1));

    // Start stream
    checkTC(DcStartStream(handle));
    while (true)
    {
        // Refresh data
        // If multiple streams are enabled, this call returns when any stream updates.
        // Set LX_BOOL_ENABLE_SYNC_FRAME to true to keep all streams in sync, or check frame ID/timestamp.
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

        // Get data
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
