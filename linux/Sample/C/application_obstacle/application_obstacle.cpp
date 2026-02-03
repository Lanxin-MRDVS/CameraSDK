//application_obstacle.cpp : This file contains the main function. Program execution begins and ends here.
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
        // Open device by IP
    case OPEN_BY_IP:
        open_param = "192.168.100.12";
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

    int algor_mode = 0;

    // Legacy version; support depends on device firmware
    //algor_mode = MODE_AVOID_OBSTACLE;
    
    // V2 version
    algor_mode = MODE_AVOID_OBSTACLE2;
    checkTC(DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, algor_mode));

    // Algorithm version
    char* algor_ver = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_VERSION, &algor_ver));
    if (algor_ver != nullptr) std::cout << " current algor version:" << algor_ver << std::endl;

    // Algorithm parameters (depend on current algorithm)
    char* cur_algor_json = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &cur_algor_json));
    if (cur_algor_json != nullptr) std::cout << " current algor json param:" << cur_algor_json << std::endl;

    // Modify algorithm parameters as needed (JSON). You can edit with a JSON library, a raw string, or via a file.
    // Example JSON below: ZoneIndex selects the active obstacle mode; Zones contains multiple modes with Danger/Warning zones.
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
    // Modify obstacle parameters; recommended via the host tool LxCameraViewer. S2 only supports host-side changes.
    //checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, obstacle_json_param.c_str()));

    // Switch obstacle parameter set using the interface below. S2 requires all sets to be preconfigured on the host.
    int pobstacle_mode = 1; // Use the second parameter set
    checkTC(DcSpecialControl(handle, "SetObstacleMode", (void*)&pobstacle_mode));

    checkTC(DcStartStream(handle));

    // Normally, after network disconnect or SDK close, the camera enters standby.
    // If algorithm output is not read via SDK (IO/serial/UDP, etc.), set always-on mode to keep the camera and algorithms running.
    checkTC(DcSetIntValue(handle, LX_INT_WORK_MODE, 1));

    while (true)
    {
        // Refresh data
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

        // Get obstacle IO output result
        int obstacle_io_result;
        checkTC(DcSpecialControl(handle, "GetObstacleIO", (void*)&obstacle_io_result));
        std::cout << " obstacle io result:" << obstacle_io_result << std::endl;

        if (device_info.dev_type == LX_DEVICE_M4
            || device_info.dev_type == LX_DEVICE_M4Pro)
        {
            // Get data
            void* algordata = nullptr;
            checkTC(DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &algordata));
            PrintData(algordata, algor_mode);
        }
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

    if (obstacle_mode == MODE_AVOID_OBSTACLE) {
        LxAvoidanceOutput* obstacle = (LxAvoidanceOutput*)data_ptr;
        if (obstacle->state != LxAvSuccess) {
            std::cout << " state is not success" << std::endl;
            return 0;
        }

        algor_ss << " groundPlane.a:" << obstacle->groundPlane.a << " number_3d:" << obstacle->number_3d << " number_box:" << obstacle->number_box << std::endl;

        if (obstacle->cloud_output != nullptr)
        {
            algor_ss << " cloud data "; // print first 10
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
    } else if(obstacle_mode == MODE_AVOID_OBSTACLE2){
        LxAvoidanceOutputN* obstacle = (LxAvoidanceOutputN*)data_ptr;
        if (obstacle->state != LxAvSuccess) {
            std::cout << " state is not success" << std::endl;
            return 0;
        }

        algor_ss << " groundPlane.a:" << obstacle->groundPlane.a << " number_3d:" << obstacle->number_3d << " number_box:" << obstacle->number_box << std::endl;

        if (obstacle->cloud_output != nullptr)
        {
            algor_ss << " cloud data "; // print first 10
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
