//application_location.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <sstream>
#include <thread>
#include <fstream>
#include <map>
#include "lx_camera_application.h"
#include "lx_camera_internal.h" 
#include "json.hpp"

#ifdef HAS_OPENCV
#include <opencv2/opencv.hpp>
#endif

using namespace std;

static char wait_key = '0';

#ifndef _WIN32
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}
#else
#include <conio.h>
#endif


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

int PrintData(void* data_ptr, std::fstream& location_file)
{
    if (data_ptr == nullptr)
        return 0;

    static uint64_t cnt = 0;
    LxLocation* location = (LxLocation*)data_ptr;
    //std::cout << "locationdata ret:" << location->status << " x:" << location->x << " y:" << location->y << " theta:" << location->theta << " timestamp:" << location->timestamp << std::endl;
    if(location_file.is_open())
        location_file << "locationdata ret:" << location->status << " x:" << location->x << " y:" << location->y << " theta:" << location->theta << " timestamp:" << location->timestamp << std::endl;

    if ((cnt++ % 10) == 0)  
        location_file.flush();

    return 0;
}

int TestRgb(bool is_enable, DcHandle handle, bool& is_first)
{
    if (!is_enable)
        return 0;

    static int width = 0, height = 0, channles = 0, rgb_data_type = 0;
    if (is_first) 
    {
        LxIntValueInfo int_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value));
        width = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value));
        height = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value));
        channles = int_value.cur_value;

        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value));
        rgb_data_type = int_value.cur_value;
        
        is_first = false;
    }

    LxFloatValueInfo rgb_fps_info = { 0 };
    checkTC(DcGetFloatValue(handle, LX_FLOAT_2D_IMAGE_FPS, &rgb_fps_info));
    std::cout << " rgb fps:" << rgb_fps_info.cur_value << std::endl;

    unsigned char* data_ptr = nullptr;
    if(DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, (void**)&data_ptr) == LX_SUCCESS && data_ptr != nullptr)
    {
        //第yRows行xCol列数据
        //2D图像目前只有unsigned char格式
        //LX_DATE_TYPE的定义与opencv CV_8U CV_16U CV_32F一致
        int yRow = 100, xCol = 100;
        int pose = yRow * width + xCol;
        if (1 == channles)
        {
            unsigned char value = data_ptr[pose];
        }
        else
        {
            unsigned char r = data_ptr[pose * 3];
            unsigned char g = data_ptr[pose * 3 + 1];
            unsigned char b = data_ptr[pose * 3 + 2];
        }

#ifdef HAS_OPENCV
        cv::Mat rgb_show = cv::Mat(height, width, CV_MAKETYPE(rgb_data_type, channles), data_ptr);
        //cv::namedWindow("rgb", 0);
        //cv::resizeWindow("rgb", 640, 480);
        cv::imshow("rgb", rgb_show);
        wait_key = cv::waitKey(1);
#endif
    }
    
    return 0;
}

int ReadAndSendOdom(DcHandle handle)
{
    const char* odom_file_path = "F:/test_V1Pro/visual_localization_sdk_dataset/pose.log";
    std::ifstream infile(odom_file_path);
    if (!infile.is_open())
    {
        std::cout << " open odom file failed" << std::endl;
        return -1;
    }

#if 0
    auto parse_data = [](std::string data_str, int64_t& timestamp, double& x, double& y, double& yaw) -> bool
    {
        //data example: {"time":1687228878872,"x":-4.639,"y":0.214,"yaw":-3.099};
        const char* time_mark = "\"time\":";
        const char* x_mark = ",\"x\":";
        const char* y_mark = ",\"y\":";
        const char* yaw_mark = ",\"yaw\":";
        auto time_index = data_str.find(time_mark);
        auto x_index = data_str.find(x_mark);
        auto y_index = data_str.find(y_mark);
        auto yaw_index = data_str.find(yaw_mark);
        if ((time_index != std::string::npos)
            && (x_index != std::string::npos)
            && (y_index != std::string::npos)
            && (yaw_index != std::string::npos)) 
        {
            timestamp = std::stoll(data_str.substr(time_index + strlen(time_mark), x_index - time_index - strlen(time_mark)));
            x = std::stod(data_str.substr(x_index + strlen(x_mark), y_index - x_index - strlen(x_mark)));
            y = std::stod(data_str.substr(y_index + strlen(y_mark), yaw_index - y_index - strlen(y_mark)));
            yaw = std::stod(data_str.substr(yaw_index + strlen(yaw_mark), data_str.length() - yaw_index - strlen(yaw_mark) - 1));

            return true;
        }
        
        return false;
    };

#endif

    std::map<int64_t, LxOdomData> odom_map;
    std::map<int64_t, LxRelocPose> pose_map;
    const char* odom_mark = ", \"odom\": ";
    const char* pose_mark = ", \"pose\": ";
    unsigned int odom_mark_len = strlen(odom_mark);
    unsigned int line_index = 0;
    LxRelocPose init_pose = {0};
    try {
        std::string strline;
        while (getline(infile, strline)) 
        {
            auto odom_index = strline.find(odom_mark);
            auto pose_index = strline.find(pose_mark);
            if (odom_index != std::string::npos && pose_index != std::string::npos)
            {
#if 0
                std::string str_odom_json = strline.substr(odom_index + odom_mark_len, pose_index - odom_index - odom_mark_len);
#else
                std::string str_odom_json = strline.substr(pose_index + strlen(pose_mark), strline.length() - pose_index - strlen(pose_mark) - 1);

#endif
                nlohmann::json js_odom;
                std::stringstream odom_json(str_odom_json);
                odom_json >> js_odom;
                LxOdomData tmp_odom_data = { 0 };
                if (js_odom.contains("time") && js_odom.contains("x") && js_odom.contains("y") && js_odom.contains("yaw"))
                {
                    tmp_odom_data.timestamp = js_odom["time"];
                    tmp_odom_data.x = js_odom["x"];
                    tmp_odom_data.y = js_odom["y"];
                    tmp_odom_data.theta = js_odom["yaw"];
                }
                else continue;


                //if (line_index < 10) 
                //{
                    std::string str_pose_json = strline.substr(pose_index + strlen(pose_mark), strline.length() - pose_index - strlen(pose_mark) - 1);
                    nlohmann::json js_pose;
                    std::stringstream pose_json(str_pose_json);
                    pose_json >> js_pose;
                    LxRelocPose tmp_pose_data = { 0 };
                    if (js_pose.contains("time") && js_pose.contains("x") && js_pose.contains("y") && js_pose.contains("yaw"))
                    {
                        init_pose.x = js_pose["x"];
                        init_pose.y = js_pose["y"];
                        init_pose.theta = js_pose["yaw"];

                        tmp_pose_data.x = js_pose["x"];
                        tmp_pose_data.y = js_pose["y"];
                        tmp_pose_data.theta = js_pose["yaw"];
                    }
                    else continue;
                //}

                line_index++;
                odom_map[tmp_odom_data.timestamp] = tmp_odom_data;
                pose_map[tmp_odom_data.timestamp] = tmp_pose_data;
            }
        }
    }
    catch (std::exception e)
    {
        std::cout << "ocured exception ," << e.what() << " : when read odom data" << std::endl;
    }

    infile.close();

    if (line_index > 0) 
    {
        auto has_send_reloc = false;
        uint64_t ict = 0;
        while (wait_key != 'q') 
        {
            for (auto it = odom_map.begin(); it != odom_map.end(); it++) 
            {
                if (wait_key == 'q') return 0;

                if(DcSpecialControl(handle, "SetOdomData", &it->second) != LX_SUCCESS) //发送odom数据
                    continue;

                if (!has_send_reloc && ict++ == 9)
                {
                    if(DcSpecialControl(handle, "SetRelocPose", &init_pose) == LX_SUCCESS) //发送reloc数据
                        has_send_reloc = true;
                }

                //if (DcSpecialControl(handle, "SetRelocPose", &pose_map[it->first]) != LX_SUCCESS)
                //    continue;

                std::this_thread::sleep_for(std::chrono::milliseconds(50)); //间隔50ms发送一次
            }

            //反向循环时不需要重定位
            auto iter2 = odom_map.rbegin();
            for (;iter2 != odom_map.rend(); iter2++) 
            {
                if (wait_key == 'q') return 0;

                if (DcSpecialControl(handle, "SetOdomData", &iter2->second) != LX_SUCCESS) //发送odom数据
                    continue;

                //if (DcSpecialControl(handle, "SetRelocPose", &pose_map[iter2->first]) != LX_SUCCESS)
                //    continue;

                std::this_thread::sleep_for(std::chrono::milliseconds(50)); //间隔50ms发送一次
            }
        }
    }
    
    return 0;
}

int main(int argc, char** argv)
{
    DcHandle handle = 0;
    checkTC(DcSetInfoOutput(0, false, "./"));
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
    int open_mode = OPEN_BY_IP;//OPEN_BY_INDEX;
    switch (open_mode)
    {
        //根据ip打开设备
    case OPEN_BY_IP:
        open_param = "192.168.100.120";
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
    
    LxIntValueInfo  algor_mode = { 0 };
    checkTC(DcGetIntValue(handle, LX_INT_ALGORITHM_MODE, &algor_mode));
    std::cout << " current mode :" << algor_mode.cur_value << std::endl;

    //开启定位算法
    checkTC(DcSetIntValue(handle, LX_INT_ALGORITHM_MODE, MODE_VISION_LOCATION));

    //算法版本号
    char* algor_ver = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_VERSION, &algor_ver));
    if (algor_ver != nullptr) std::cout << " current algor version:" << algor_ver << std::endl;

    //算法参数，与当前设置的算法有关
    char* cur_algor_json = nullptr;
    checkTC(DcGetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, &cur_algor_json));
    if (cur_algor_json != nullptr) std::cout << " current algor json param:" << cur_algor_json << std::endl;

    auto sret = LX_ERROR;

    //导入地图
    std::vector<std::string> in_map_list;
    //in_map_list.push_back("F:/work/aaa_test_file.pgm");
    in_map_list.push_back("./22.pcapng");
    //in_map_list.push_back("zip_utils_src");
    checkTC(DCAPI::DcUpLoadFiles(handle, in_map_list, 11));

    //建图使能
    checkTC(DcSetBoolValue(handle, DCAPI::LX_BOOL_ENABLE_BUILD_MAP, true)); 

    //定位使能
    checkTC(DcSetBoolValue(handle, DCAPI::LX_BOOL_ENABLE_LOCATION, true)); 

     //是否设置定位调试模式
    checkTC(DcSetIntValue(handle, DCAPI::LX_INI_RELOC_DEBUG_MODE, 0));

    //获取所有地图名
    char* out_map_name = nullptr;
    checkTC(DcGetStringValue(handle, DCAPI::LX_STRING_LOCATION_MAP_NAME, &out_map_name));
    if (out_map_name != nullptr) std::cout << " GetAllMapName:" << out_map_name << std::endl;

    //导出地图文件
    const char* out_map = "./outmap.zip";
    checkTC(DcSetStringValue(handle, DCAPI::LX_STRING_LOCATION_MAP_FILE, out_map));


    //定位算法参数可通过如下函数设置，参数为json格式
    char sz_algor_json[512] = "{\"map_name\":\"XS1019\", \"external_param\":[-0.14, -0.172, 1.2, 179.799, -0.512, 0.116]}";
    checkTC(DcSetStringValue(handle, LX_STRING_ALGORITHM_PARAMS, sz_algor_json));

    //切换地图
    //char newmap[] = "0509";
    //sret = DcSpecialControl(handle, "SetMapName", newmap);
    //std::cout << " DcSpecialControl  SetMapName:" << newmap << " sret:" << sret << std::endl;

    //正常状态下，网络断开或者SDK关闭之后，相机会切换为待机状态。
    //如果算法结果不通过SDK输出，需要设置为常开模式，相机和内置算法会始终保持工作
    checkTC(DcSetIntValue(handle, LX_INT_WORK_MODE, 1));

    //开启RGB
    bool enable_rgb = true;
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, enable_rgb));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &enable_rgb));

    //开启数据流
    checkTC(DcStartStream(handle));

    //开启线程, 读取odom数据
    std::thread threads_odom(ReadAndSendOdom, handle);
    threads_odom.detach();

    std::fstream loca_file("./location_file.log", ios::out | ios::in | ios::trunc);

    static bool is_first = true;
    while (true)
    {
        //刷新数据
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if (LX_SUCCESS != ret)
        {            
            std::cout << " DcSetCmd LX_CMD_GET_NEW_FRAME failed" << std::endl;

            if (LX_E_RECONNECTING == ret) {
                std::cout << "device is reconnecting" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            continue;
        }

        //show rgb img
        TestRgb(enable_rgb, handle, is_first);

#ifndef HAS_OPENCV
     if (kbhit()) wait_key = getchar();
#endif

     if (wait_key == 'q')  break;

        //获取数据
        void* algordata = nullptr;
        if(DcGetPtrValue(handle, LX_PTR_ALGORITHM_OUTPUT, &algordata) == LX_SUCCESS) PrintData(algordata, loca_file);
    }

    DcStopStream(handle);
    DcCloseDevice(handle);
    if (loca_file.is_open())
    {
        loca_file.flush();
        loca_file.close();
    }
    getchar();
    return 0;
}
