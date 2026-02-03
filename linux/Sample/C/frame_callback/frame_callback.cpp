//frame_callback.cpp : This file contains the main function. Program execution begins and ends here.
//

#include <string>
#include <thread>
#include "lx_camera_api.h"
#include "lx_camera_application.h"

#undef HAS_OPENCV
#ifdef HAS_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
#endif

static char wait_key = '0';
#define checkTC(state) {LX_STATE val=state;                                 \
    if(val!= LX_SUCCESS){                                                   \
        if(val == LX_E_NOT_SUPPORT) {                                       \
            printf("not support this operator\n");                          \
        }else if(val == LX_E_RECONNECTING){                                 \
            printf("device reconnecting\n");                                \
        } else  if(val == LX_ERROR || val == LX_E_INPUT_ILLEGAL){           \
            printf("press any key to exit!\n");                             \
            wait_key = getchar();                                        \
            return -1;                                                      \
         } else{                                                            \
            printf("press any key to continue\n");                          \
            wait_key = getchar();                                        \
         }                                                                  \
     }                                                                      \
}

void CallbackFunc(FrameInfo* frame, void* usr_data);

int main(int argc, char** argv)
{
    // Set log level and path.
    //checkTC(DcSetInfoOutput(0, true, ""));
    printf("call api version: %s\n", DcGetApiVersion());

    // Find device
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num));
    if (device_num <= 0)
    {
        printf("not found any device, press any key to exit");
        wait_key = getchar();
        return -1;
    }
    printf("DcGetDeviceList success list: %d\n", device_num);

    // Open camera. SDK uses GIGE and will find all GIGE cameras; be careful when opening by index.
    // After opening, the camera is exclusive; other processes cannot open it.
    // If the program exits without DcCloseDevice, wait for heartbeat timeout to release.
    std::string open_param;
    int open_mode = OPEN_BY_INDEX; //OPEN_BY_IP;//
    switch (open_mode)
    {
        // Open device by IP
    case OPEN_BY_IP:
        open_param = "192.168.100.82";
        break;
        // Open device by SN
    case OPEN_BY_SN:
        open_param = "ccf8981cc50b66b6";
        break;
        // Open device by index in the discovered device list
    case OPEN_BY_INDEX:
    default:
        open_param = "0";
        break;
    }

    DcHandle handle = 0;
    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice((LX_OPEN_MODE)open_mode, open_param.c_str(), &handle, &device_info);
    if (LX_SUCCESS != lx_state) {
        printf("open device failed, open_mode: %d open_param: %s\n.press any key to exit\n", open_mode, open_param.c_str());
        wait_key = getchar();
        return -1;
    }
    printf("device_info\nid: %s\nip: %s\nsn: %s\nfirmware_ver:%s\n",
        device_info.id, device_info.ip, device_info.sn, device_info.firmware_ver);

    // Get stream status
    bool depth_enable = false, amp_enable = false, rgb_enable = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &depth_enable));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &amp_enable));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &rgb_enable));
    // Get built-in algorithm status
    int app_mode = LX_ALGORITHM_MODE::MODE_ALL_OFF;
    LxIntValueInfo int_value;
    checkTC(DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &int_value));
    app_mode = int_value.cur_value;

    LX_TRIGGER_MODE trigger_mode;
    if (DcGetIntValue(handle, LX_INT_TRIGGER_MODE, &int_value) == LX_E_NOT_SUPPORT)
        trigger_mode = LX_TRIGGER_MODE_OFF;
    else
        trigger_mode = static_cast<LX_TRIGGER_MODE>(int_value.cur_value);

    printf("depth_enable:%d amp_enable:%d rgb_enable:%d app_mode:%d\n", depth_enable, amp_enable, rgb_enable, app_mode);

    // Register callback
    checkTC(DcRegisterFrameCallback(handle, CallbackFunc, nullptr));

    // Start stream
    checkTC(DcStartStream(handle));

    if (trigger_mode == LX_TRIGGER_MODE::LX_TRIGGER_SOFTWARE) {
        std::thread trigger_pthread = std::thread([&]() {
            do {
                // Software trigger mode requires manual triggering; this is just an example
                auto ret = DcSetCmd(handle, LX_CMD_SOFTWARE_TRIGGER);
                if (ret != LX_SUCCESS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    continue;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } while (wait_key != 'q');
         });
        trigger_pthread.detach();
    }

    printf("**********press 'q' to exit********\n\n");
    while (wait_key != 'q')
    {
        wait_key = getchar();
    }

    checkTC(DcUnregisterFrameCallback(handle));
    checkTC(DcStopStream(handle));
    checkTC(DcCloseDevice(handle));
    return 0;
}

void CallbackFunc(FrameInfo* frame_ptr, void* usr_data)
{
    // Avoid heavy work inside this callback to prevent blocking internal callbacks
    // Callback mode does not require LX_CMD_GET_NEW_FRAME
    if (frame_ptr == nullptr)
    {
        return;
    }
    if (frame_ptr->frame_state != LX_SUCCESS)
    {
        if (frame_ptr->frame_state == LX_E_FRAME_ID_NOT_MATCH) {
            printf("is enabled sync frame, but frame_state:%d\n", frame_ptr->frame_state);
            //return;
        }
        else if (frame_ptr->frame_state == LX_E_FRAME_MULTI_MACHINE) {
            printf("recv multi machine singal, frame_state:%d\n", frame_ptr->frame_state);
            //return;
        }
        else {
            printf("frame_state:%d\n", frame_ptr->frame_state);
            return;
        }
    }

    // If frame ID is needed, use this struct. For frame sync, cache frames and match IDs in a queue.
    FrameExtendInfo* p_extend_info = nullptr;
    if (frame_ptr->reserve_data != nullptr)
        p_extend_info = (FrameExtendInfo*)frame_ptr->reserve_data;

    //depth
    if (frame_ptr->depth_data.frame_data != nullptr)
    {
        printf("depth sensor_timestamp: %lld\n", frame_ptr->depth_data.sensor_timestamp);
        if (p_extend_info != nullptr)
            printf("depth frame_id:%d\n", p_extend_info->depth_frame_id);

        // Get point cloud data (optional). It is derived from depth and updated synchronously.
        float* xyz_data = nullptr;
        if (DcGetPtrValue(frame_ptr->handle, LX_PTR_XYZ_DATA, (void**)&xyz_data) != LX_SUCCESS)
            printf("get xyz failed");
        if (xyz_data != nullptr)
        {
            // xyz_data is managed by the API and ordered like the depth image; points are in camera coordinates.
            // Total size is (frame_width * frame_height * sizeof(float) * 3).
            // Example: get the point at row yRow=100, col xCol=100.
            int yRow = 100, xCol = 100;
            int pose = yRow * frame_ptr->depth_data.frame_width + xCol;
            float x = xyz_data[pose * 3];
            float y = xyz_data[pose * 3 + 1];
            float z = xyz_data[pose * 3 + 2];
        }
    }
    //amp
    if (frame_ptr->amp_data.frame_data != nullptr)
    {
        printf("amp sensor_timestamp:%lld\n", frame_ptr->amp_data.sensor_timestamp);
        if (p_extend_info != nullptr)
            printf("amp frame_id:%d\n", p_extend_info->amp_frame_id);
    }
    //rgb
    if (frame_ptr->rgb_data.frame_data != nullptr)
    {
        printf("rgb sensor_timestamp:%lld\n", frame_ptr->rgb_data.sensor_timestamp);
        if (p_extend_info != nullptr)
            printf("rgb frame_id:%d\n", p_extend_info->rgb_frame_id);
    }

#ifdef HAS_OPENCV
    //show
    if (frame_ptr->depth_data.frame_data != nullptr) {
        FrameDataInfo depth_data = frame_ptr->depth_data;
        cv::Mat depth_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_MAKETYPE(depth_data.frame_data_type, depth_data.frame_channel), depth_data.frame_data);

        cv::Mat show;
        depth_image.convertTo(show, CV_8U, 1.0 / 16);
        applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("depth", 0);
        //cv::resizeWindow("depth", 640, 480);
        cv::imshow("depth", show);
    }

    if (frame_ptr->amp_data.frame_data != nullptr) {
        FrameDataInfo amp_data = frame_ptr->amp_data;
        cv::Mat amp_image = cv::Mat(amp_data.frame_height, amp_data.frame_width,
            CV_MAKETYPE(amp_data.frame_data_type, amp_data.frame_channel), amp_data.frame_data);

        cv::Mat show;
        if (amp_image.type() == CV_16U)
            amp_image.convertTo(show, CV_8U, 1.0 / 8);
        else
            show = amp_image;

        cv::namedWindow("amp", 0);
        //cv::resizeWindow("amp", 640, 480);
        cv::imshow("amp", show);
    }

    if (frame_ptr->rgb_data.frame_data != nullptr) {
        FrameDataInfo rgb_data = frame_ptr->rgb_data;
        cv::Mat rgb_image = cv::Mat(rgb_data.frame_height, rgb_data.frame_width,
            CV_MAKETYPE(rgb_data.frame_data_type, rgb_data.frame_channel), rgb_data.frame_data);

        cv::namedWindow("rgb", 0);
        //cv::resizeWindow("rgb", 640, 480);
        cv::imshow("rgb", rgb_image);
    }

    cv::waitKey(1);
#endif
    //process application
    if (frame_ptr->app_data.frame_data != nullptr)
    {
        if (frame_ptr->app_data.frame_data_type == LX_DATA_PALLET)
        {
            LxPalletPose* palletdata = (LxPalletPose*)frame_ptr->app_data.frame_data;
            printf("palletdata ret:%d x:%f, y:%f yaw:%f\n",
                palletdata->return_val, palletdata->x, palletdata->y, palletdata->yaw);
        }
        else if (frame_ptr->app_data.frame_data_type == LX_DATA_OBSTACLE2)
        {
            LxAvoidanceOutputN* obstacle = (LxAvoidanceOutputN*)frame_ptr->app_data.frame_data;
            if (obstacle->state != LxAvSuccess) {
                printf("state is not success\n");
                return;
            }

            if (obstacle->obstacleBoxs != nullptr)
            {
                printf(" obstacle data ");
                int box_num = obstacle->number_box < 10 ? obstacle->number_box : 10;
                for (int i = 0; i < box_num; i++)
                {
                    printf("index:%d width:%f height:%f depth:%f center:%f %f %f",
                        i, obstacle->obstacleBoxs[i].width, obstacle->obstacleBoxs[i].height, obstacle->obstacleBoxs[i].depth,
                        obstacle->obstacleBoxs[i].center[0], obstacle->obstacleBoxs[i].center[1], obstacle->obstacleBoxs[i].center[2]);
                }
            }
        }
    }
    return;
}
