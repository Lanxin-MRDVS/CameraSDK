// single_camera.cpp : This file contains the main function. Program execution begins and ends here.
//

#include <iostream>
#include <thread>
#include <sstream>
#include "lx_camera_api.h"

using namespace std;

DcHandle handle = 0;
#define checkTC(state) {LX_STATE val=state;                                  \
    if(val!= LX_SUCCESS){                                                    \
        if(val == LX_E_NOT_SUPPORT) {                                        \
            std::cout << " not support this operator" << std::endl;          \
        }else if(val == LX_E_RECONNECTING){                                  \
            std::cout << " device reconnecting" << std::endl;                \
        } else  if(val == LX_ERROR || val == LX_E_INPUT_ILLEGAL){            \
            std::cout << " press any key to exit!" << std::endl;             \
            DcCloseDevice(handle);                       \
            getchar();                                         \
            return -1;                                                       \
         } else{                                                             \
            std::cout << " press any key to continue" << std::endl;          \
            getchar();                                         \
         }                                                                   \
     }                                                                       \
}

int TestDepth(bool is_enable, DcHandle handle);
int TestAmp(bool is_enable, DcHandle handle);
int TestRgb(bool is_enable, DcHandle handle);

int main(int argc, char** argv)
{
    checkTC(DcSetInfoOutput(1, true, "./"));
    std::cout << " call api version:" << DcGetApiVersion() << std::endl;
    
    std::string open_param = "127.0.0.1";
    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice(OPEN_BY_IP, open_param.c_str(), &handle, &device_info);
    if (LX_SUCCESS != lx_state){
        std::cout << "open device failed, open_param:" << open_param << " press any key to exit" << std::endl;
        getchar();
        return -1;
    }

    std::cout << "device_info\n cameraid:" << device_info.id << "\n uniqueid:" << handle 
        << "\n cameraip:" << device_info.ip << "\n firmware_ver:" << device_info.firmware_ver << "\n sn:" << device_info.sn 
        << "\n name:" << device_info.name <<"\n img_algor_ver:" << device_info.algor_ver << std::endl;

    bool test_depth = true, test_amp = false, test_rgb = true; 
    std::cout << "test_depth:" << test_depth << " test_amp:" << test_amp << " test_rgb:" << test_rgb << std::endl;
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, test_depth));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, test_amp));
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, test_rgb));

    checkTC(DcStartStream(handle));
    auto _time = std::chrono::system_clock::now();
    while (true)
    {
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if (LX_SUCCESS != ret)
        {
            if (LX_E_RECONNECTING == ret) {
                std::cout << "device reconnecting" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        auto _time2 = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(_time2 - _time).count() > 1)
        {
            _time = std::chrono::system_clock::now();

            LxFloatValueInfo dep_float_info = { 0 };
            LxFloatValueInfo amp_float_info = { 0 };
            LxFloatValueInfo rgb_float_info = { 0 };
            std::stringstream ss;
            if (test_depth) 
            {
                checkTC(DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &dep_float_info));
                ss << " depth fps:" << dep_float_info.cur_value;
            }
            
            if (test_amp)
            {
                checkTC(DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &amp_float_info));
                ss << " amp fps:" << amp_float_info.cur_value;
            }

            if (test_rgb)
            {
                checkTC(DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &rgb_float_info));
                ss << " rgb fps:" << rgb_float_info.cur_value;
            }

            std::cout << ss.str() << std::endl;
        }

        TestDepth(test_depth, handle);
        TestAmp(test_amp, handle);
        TestRgb(test_rgb, handle);
    }

    DcStopStream(handle);
    DcCloseDevice(handle);

    return 0;
}


int TestDepth(bool is_enable, DcHandle handle)
{
    if (!is_enable)
        return 0;

    // In theory you don't need to query width/height/data type every time.
    // Re-query only if ROI/binning/depth_calib changes the resolution; done here for safety.
    int width = 0, height = 0;
    LxIntValueInfo int_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value));
    width = int_value.cur_value;

    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value));
    height = int_value.cur_value;

    void* data_ptr = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &data_ptr));

    checkTC(DcGetIntValue(handle, LX_INT_3D_DEPTH_DATA_TYPE, &int_value));
    int data_type = int_value.cur_value;
    // Depth data at row yRow, column xCol
    // TOF camera depth is unsigned short; structured-light camera (LX_DEVICE_WK) is float
    // Point cloud uses float; xyz stored sequentially
    // LX_DATE_TYPE matches OpenCV CV_8U / CV_16U / CV_32F
    int yRow = 100, xCol = 100;
    int pose = yRow * width + xCol;
    if (LX_DATA_UNSIGNED_SHORT == data_type)
    {
        unsigned short* data = (unsigned short*)data_ptr;
        unsigned short value = data[pose];
    }
    else if (LX_DATA_FLOAT == data_type)
    {
        float* data = (float*)data_ptr;
        float value = data[pose];
    }

    // Point cloud data at row yRow, column xCol
    float* xyz_data = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_XYZ_DATA, (void**)&xyz_data));
    float x = xyz_data[pose * 3];
    float y = xyz_data[pose * 3 + 1];
    float z = xyz_data[pose * 3 + 2];
    return 0;
}

int TestAmp(bool is_enable, DcHandle handle)
{
    if (!is_enable)
        return 0;

    int width = 0, height = 0;
    LxIntValueInfo int_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value));
    width = int_value.cur_value;

    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value));
    height = int_value.cur_value;


    // Intensity data at row yRow, column xCol
    // TOF camera intensity is unsigned short; structured-light camera (LX_DEVICE_WK) is unsigned char
    // LX_DATE_TYPE matches OpenCV CV_8U / CV_16U / CV_32F
    int yRow = 100, xCol = 100;
    int pose = yRow * width + xCol;
    checkTC(DcGetIntValue(handle, LX_INT_3D_AMPLITUDE_DATA_TYPE, &int_value));
    int data_type = int_value.cur_value;
    if (LX_DATA_UNSIGNED_SHORT == data_type)
    {
        unsigned short* data_ptr = nullptr;
        checkTC(DcGetPtrValue(handle, LX_PTR_3D_AMP_DATA, (void**)&data_ptr));
        unsigned short value = data_ptr[pose];
    }
    else if (LX_DATA_UNSIGNED_CHAR == data_type)
    {
        unsigned char* data_ptr = nullptr;
        checkTC(DcGetPtrValue(handle, LX_PTR_3D_AMP_DATA, (void**)&data_ptr));
        unsigned char value = data_ptr[pose];
    }

    return 0;
}

int TestRgb(bool is_enable, DcHandle handle)
{
    if (!is_enable)
        return 0;

    int width = 0, height = 0, channles = 0, rgb_data_type = 0;

    LxIntValueInfo int_value;
    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value));
    width = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value));
    height = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value));
    channles = int_value.cur_value;

    checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value));
    rgb_data_type = int_value.cur_value;

    unsigned char* rgb_data = nullptr;
    checkTC(DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, (void**)&rgb_data));

    // Data at row yRow, column xCol
    // 2D image currently only uses unsigned char format
    // LX_DATE_TYPE matches OpenCV CV_8U / CV_16U / CV_32F
    int yRow = 100, xCol = 100;
    int pose = yRow * width + xCol;
    if (1 == channles)
    {
        unsigned char value = rgb_data[pose];
    }
    else
    {
        unsigned char r = rgb_data[pose * 3];
        unsigned char g = rgb_data[pose * 3 + 1];
        unsigned char b = rgb_data[pose * 3 + 2];
    }
    return 0;
}
