//application_obstacle_v1.cpp : This file contains the main function. Program execution begins and ends here.
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

    // Apply algorithm
    LxIntValueInfo algor_info = { 0 };
    checkTC(DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &algor_info));
    std::cout << " current LX_INT_ALGORITHM_MODE:" << algor_info.cur_value << std::endl;

    int algor_mode = MODE_AVOID_OBSTACLE;
    checkTC(DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, algor_mode));

    // Algorithm version
    char* algor_ver = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_VERSION, &algor_ver));
    if (algor_ver != nullptr) std::cout << " current algor version:" << algor_ver << std::endl;

    // Algorithm parameters (depend on current algorithm)
    char* cur_algor_json = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &cur_algor_json));
    if (cur_algor_json != nullptr) std::cout << " current algor json param:" << cur_algor_json << std::endl;

	// Disable 3D depth, intensity, and 2D image streams to save bandwidth (enable if needed)
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, false));

    checkTC(DcStartStream(handle));

    // Switch only the obstacle detection region index using the interface below.
    // For S2/S10 using obstacle V1, the host must preconfigure each index set before switching, otherwise it is invalid.
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
            // Update obstacle detection region index
			// Note: SetObstacleIndex is fast index switching; SetObstacleMode is legacy full config and slower.
            // MODE_AVOID_OBSTACLE supports SetObstacleIndex and SetObstacleMode; MODE_AVOID_OBSTACLE2 only supports SetObstacleMode.
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
        // Get obstacle IO output result
        int obstacle_io_result;
        LX_STATE state = DcSpecialControl(handle, "GetObstacleIO", (void*)&obstacle_io_result);
        if (state != LX_SUCCESS) {
            std::cout << DcGetErrorString(state) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // TODO: error handling
        }
        else {
            std::cout << " obstacle io result:" << obstacle_io_result << std::endl;
        }
        // Sleep to avoid frequent device access
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    getchar();
    return 0;
}
