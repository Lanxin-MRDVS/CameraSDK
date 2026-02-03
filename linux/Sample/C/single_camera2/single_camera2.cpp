// single_camera.cpp : This file contains the main function. Program execution begins and ends here.
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
        else if(val == LX_E_NOT_SUPPORT){                              \
            printf("device not support\n");                          \
        }                                                               \
        else{                                                          \
            printf("%s\n", DcGetErrorString(val));                     \
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

    // Find camera
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

    // Open camera. SDK uses GIGE and finds all GIGE cameras; be careful when opening by index.
    // After opening, the camera is exclusive; other processes cannot open it.
    // If the program exits without DcCloseDevice, wait for heartbeat timeout to release.
    std::string open_param;
    int open_mode = OPEN_BY_INDEX;
    switch (open_mode)
    {
        // Open device by IP
    case OPEN_BY_IP:
        open_param = "192.168.100.82";
        break;
        // Open device by ID
    case OPEN_BY_ID:
        open_param = "F13301122647";
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

    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice((LX_OPEN_MODE)open_mode, open_param.c_str(), &handle, &device_info);
    if (LX_SUCCESS != lx_state){
        printf("open device failed, open_mode: %d open_param: %s\n.press any key to exit\n", open_mode, open_param.c_str());
        wait_key = getchar();
        return -1;
    }

    printf("device_info\nid: %s\nip: %s\nsn: %s\nfirmware_ver:%s\n",
        device_info.id, device_info.ip, device_info.sn, device_info.firmware_ver);

    // Get stream enable status; adjust via DcSetBoolValue if needed.
    bool test_depth = false, test_amp = false, test_rgb = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &test_depth));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &test_amp));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &test_rgb));

    // RGBD alignment keeps RGB and depth sizes/coordinates consistent.
    // Two modes: align to depth or to RGB. Aligning to RGB makes depth/amp coordinates inconsistent.
    //checkTC(DcSetIntValue(handle, LX_INT_RGBD_ALIGN_MODE, LX_RGBD_ALIGN_MODE::DEPTH_TO_RGB));

    // Enable frame sync if needed; when enabled, frames are synchronized internally before returning.
    // If not needed, keep it off for better real-time behavior; you can also use frame ID/timestamp.
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_SYNC_FRAME, true));

    printf("test_depth: %d test_amp: %d test_rgb: %d\n", test_depth, test_amp, test_rgb);

    /* Other operations: set parameters before starting stream, especially those affecting image size. */

    // Trigger mode
    LxIntValueInfo info;
    LX_TRIGGER_MODE trigger_mode = LX_TRIGGER_MODE_OFF;  
    //DcSetIntValue(handle, LX_INT_TRIGGER_MODE, trigger_mode);
    if(DcGetIntValue(handle, LX_INT_TRIGGER_MODE, &info) == LX_SUCCESS)
        trigger_mode = static_cast<LX_TRIGGER_MODE>(info.cur_value);

    // Start stream
    checkTC(DcStartStream(handle));

    std::thread pthread = std::thread(WaitKey);
    pthread.detach();

    std::this_thread::sleep_for(std::chrono::seconds(1));
    while (true)
    {
        // Update data
        if (wait_key == 'q' || wait_key == 'Q')
            break;

        LX_STATE ret;
        if (trigger_mode == LX_TRIGGER_MODE::LX_TRIGGER_SOFTWARE)
        {
            //// Software trigger mode requires manual triggering; this is just an example
            ret = DcSetCmd(handle, LX_CMD_SOFTWARE_TRIGGER);
            if (ret != LX_SUCCESS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                continue;
            }
        }

        ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);

        if ((LX_SUCCESS != ret)
            // With frame sync enabled, timeout without a matched frame (packet loss) returns LX_E_FRAME_ID_NOT_MATCH
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            // In multi-camera mode, interference returns LX_E_FRAME_MULTI_MACHINE
            && (LX_E_FRAME_MULTI_MACHINE != ret))
        {
            if (LX_E_RECONNECTING == ret) {
                printf("device reconnecting\n");
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        TestFrame(handle);
    }

    DcStopStream(handle);
    DcCloseDevice(handle);

    return 0;
}


int TestFrame(DcHandle handle)
{
    FrameInfo* frame_ptr = nullptr;
    if (LX_SUCCESS != DcGetPtrValue(handle, LX_PTR_FRAME_DATA, (void**)&frame_ptr)) {
		printf("DcGetPtrValue failed\n");
        return 1;
    }
    if (!frame_ptr) {
        printf("new frame not correct, frame ptr is null\n");
        return 1;
    }

    if (frame_ptr->frame_state != LX_SUCCESS) {
        if (frame_ptr->frame_state == LX_E_FRAME_ID_NOT_MATCH)
            printf("frame id not match\n");
        else if(frame_ptr->frame_state == LX_E_FRAME_MULTI_MACHINE)
            printf("found multi machine signal\n");
        else {
            printf("new frame not correct\n");
            return 1;
        }
    }

    // Frame ID info can be used to synchronize streams; otherwise can be ignored
    FrameExtendInfo* pextendframe = nullptr;
    if (frame_ptr->reserve_data != nullptr)
        pextendframe = (FrameExtendInfo*)frame_ptr->reserve_data;

    // Depth data
    if (frame_ptr->depth_data.frame_data != nullptr)
    {
        auto depth_data = frame_ptr->depth_data;
        printf("depth_sensor_time:%lld\n", depth_data.sensor_timestamp);
        if (pextendframe != nullptr)
            printf("depth_frame_id:%d\n", pextendframe->depth_frame_id);

        // Convert depth data to XYZ point cloud
        void* xyz = NULL;
        DcGetPtrValue(handle, LX_PTR_XYZ_DATA, &xyz);

#ifdef HAS_OPENCV
        cv::Mat depth_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_MAKETYPE(depth_data.frame_data_type, depth_data.frame_channel), depth_data.frame_data).clone();
        cv::Mat xyz_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_32FC3, xyz).clone();

        cv::Mat show;
        depth_image.convertTo(show, CV_8U, 1.0 / 16);
        cv::applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("depth", 0);
        cv::resizeWindow("depth", 640, 480);
        cv::imshow("depth", show);
        cv::waitKey(1);
#endif
    }

    // Intensity data
    if (frame_ptr->amp_data.frame_data != nullptr)
    {
        auto amp_data = frame_ptr->amp_data;
        printf("amp_sensor_time:%lld\n", amp_data.sensor_timestamp);
        if (pextendframe != nullptr)
            printf("amp_frame_id:%d\n", pextendframe->amp_frame_id);

#ifdef HAS_OPENCV
        cv::Mat amp_image = cv::Mat(amp_data.frame_height, amp_data.frame_width,
            CV_MAKETYPE(amp_data.frame_data_type, amp_data.frame_channel), amp_data.frame_data).clone();

        cv::Mat show;
        if (amp_image.type() == CV_16U)
            amp_image.convertTo(show, CV_8U, 1.0 / 8);
        else
            show = amp_image;
        cv::applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("amp", 0);
        cv::resizeWindow("amp", 640, 480);
        cv::imshow("amp", show);
        cv::waitKey(1);
#endif
    }

    // RGB data
    if (frame_ptr->rgb_data.frame_data != nullptr)
    {
        auto rgb_data = frame_ptr->rgb_data;
        printf("rgb_sensor_time:%lld\n", rgb_data.sensor_timestamp);
        if (pextendframe != nullptr)
            printf("rgb_frame_id:%d\n", pextendframe->rgb_frame_id);

#ifdef HAS_OPENCV
        cv::Mat rgb_image = cv::Mat(rgb_data.frame_height, rgb_data.frame_width,
            CV_MAKETYPE(rgb_data.frame_data_type, rgb_data.frame_channel), rgb_data.frame_data).clone();

        cv::namedWindow("rgb", 0);
        cv::resizeWindow("rgb", 640, 480);
        cv::imshow("rgb", rgb_image);
        cv::waitKey(1);
#endif
    }

    return 0;
}
