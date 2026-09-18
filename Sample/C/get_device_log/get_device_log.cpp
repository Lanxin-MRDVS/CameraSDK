#include <string>
#include <iostream>
#include "lx_camera_api.h"
using namespace std;

#define checkTC(state) {LX_STATE val=state;                             \
if(LX_SUCCESS!=val){                                                    \
    if(val > 0){                                                        \
        printf("WARNING %s\n", DcGetErrorString(val));}                 \
    else if(val == LX_E_RECONNECTING){                                  \
        printf("device reconnecting\n"); }                              \
    else{                                                               \
        printf("%s. press any key to exit!\n", DcGetErrorString(val));  \
        DcCloseDevice(handle);                                          \
        return -1;                                                      \
    }                                                                   \
}}


int main(int argc, char** argv)
{
    DcSetInfoOutput(1, true, "");

    DcHandle handle = 0;
    printf("call api version: %s\n", DcGetApiVersion());

    //查找相机
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num));    
    if (device_num <= 0) 
    {
        printf("not found any device, press any key to exit\n");
        return -1;
    }
    printf("DcGetDeviceList success list: %d\n", device_num);

    std::string open_param;
    if (argc > 1) open_param = argv[1];

    bool bfind = false;
    for (int i = 0; i < device_num; i++) 
    {
        if (open_param.empty()) {
            open_param = p_device_list[0].ip;
            handle = p_device_list[0].handle;
            bfind = true;
            break;
        }
        
        std::string str_ip = p_device_list[i].ip;
        if (str_ip.find(open_param) != std::string::npos){
            handle = p_device_list[i].handle;
            bfind = true;
            break;
        }
    }
    
    if (!bfind) 
    {
        std::cout << "not found matched device, open device ip:" << open_param << ", exit..." << std::endl;
        return -1;
    } 

    int authority = 1;
    DcSpecialControl(handle, "LxSetPrivilege", &authority);
    
    int open_mode = OPEN_BY_IP;

    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice((LX_OPEN_MODE)open_mode, open_param.c_str(), &handle, &device_info);
    if (LX_SUCCESS != lx_state)
    {
        printf("open device failed, open_mode: %d open_param: %s\n.press any key to exit\n", open_mode, open_param.c_str());
        return -1;
    }

    std::string log_path = device_info.id;
    log_path+="_camera_log.zip";
    auto sret = DcSetStringValue(handle, 4002, log_path.c_str());
    if (sret != LX_SUCCESS)
        std::cout << " get device log file error, sret:" << sret << std::endl;
    else
        std::cout << " success get device log..." << std::endl;

    DcCloseDevice(handle);

    return 0;
}

