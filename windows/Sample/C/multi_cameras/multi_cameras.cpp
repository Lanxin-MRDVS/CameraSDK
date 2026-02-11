// multi_camera.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <string.h>
#include <vector>
#include "lx_camera_api.h"
#ifdef HAS_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
#include <mutex>
#include <queue>
#include <condition_variable>
#include <unordered_set>
#include <atomic>

// GUI 线程控制与交换队列（仅在 HAS_OPENCV 时启用）
static std::atomic<bool> g_guiExit{ false };

struct GuiItem {
	std::string name;
	cv::Mat img;
	int width;
	int height;
};

static std::mutex g_guiMutex;
static std::condition_variable g_guiCond;
static std::queue<GuiItem> g_guiQueue;

static int mapLXToCvDepth(int lxType)
{
	switch (lxType) {
	case LX_DATA_UNSIGNED_CHAR:  return CV_8U;
	case LX_DATA_UNSIGNED_SHORT: return CV_16U;
	case LX_DATA_FLOAT:          return CV_32F;
	default:                     return -1;
	}
}

static void guiThreadProc()
{
	std::unordered_set<std::string> created;
	while (!g_guiExit.load()) {
		GuiItem item;
		{
			std::unique_lock<std::mutex> lk(g_guiMutex);
			g_guiCond.wait_for(lk, std::chrono::milliseconds(10), [] {
				return !g_guiQueue.empty() || g_guiExit.load();
			});
			if (g_guiExit.load()) break;
			if (g_guiQueue.empty()) { cv::waitKey(1); continue; }
			item = std::move(g_guiQueue.front());
			g_guiQueue.pop();
		}
		if (created.find(item.name) == created.end()) {
			cv::namedWindow(item.name, 0);
			cv::resizeWindow(item.name, item.width, item.height);
			created.insert(item.name);
		}
		cv::imshow(item.name, item.img);
		cv::waitKey(1);
	}
}
#endif
using namespace std;

//相机唯一标识信息，传给回调函数以区分设备
typedef struct CameraCallbackData {
    DcHandle handle;
    std::string ip;
    std::string deviceId;
    std::string sn;
} CameraCallbackData;


void CameraStatusCallback(CameraStatus* statusInfo, void* usr_data);

static char wait_key = '0';
#define checkTC(state,handle) {LX_STATE val=state;                             \
if(LX_SUCCESS!=val){                                                    \
    if(val > 0){                                                        \
        printf("WARNING %s\n", DcGetErrorString(val));}                 \
    else if(val == LX_E_RECONNECTING){                                  \
        printf("device reconnecting\n"); }                              \
    else{                                                               \
        printf("%s. press any key to exit!\n", DcGetErrorString(val));  \
        wait_key = getchar();                                           \
        DcCloseDevice(handle);                                          \
        return -1;                                                      \
    }                                                                   \
}}

int thread_work(std::string ip);

int main(int argc, char** argv)
{
    //日志
    checkTC(DcSetInfoOutput(1, false, "./"), 0);
    std::cout << " call api version:" << DcGetApiVersion() << std::endl;

    //查找设备
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    checkTC(DcGetDeviceList(&p_device_list, &device_num), 0);
    if (device_num <= 0)
    {
        std::cout << "DcGetDeviceList success, but found no any device, press any key to exit" << std::endl;
        getchar();
        return -1;
    }
    std::cout << "DcGetDeviceList success list: " << device_num << std::endl << std::endl;

    //开启多个线程
    std::vector<std::thread> threads_vec;
#ifdef HAS_OPENCV
    // 启动 GUI 渲染线程
    std::thread guiThread(guiThreadProc);
#endif
    for (auto device_index = 0; device_index < device_num; device_index++)
    {
        LxDeviceInfo device_info = p_device_list[device_index];
        std::cout << "open device:" << device_info.id << " " << device_info.ip << std::endl;

        threads_vec.push_back(std::thread(thread_work, device_info.ip));
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    printf("**********press 'q' to exit********\n\n");
    while (wait_key != 'q')
    {
        wait_key = getchar();
    }

    for (auto& t : threads_vec)
    {
        t.join();
    }
#ifdef HAS_OPENCV
    g_guiExit.store(true);
    g_guiCond.notify_all();
    // 等待 GUI 线程退出
    if (guiThread.joinable()) guiThread.join();
#endif
    return 0;
}

int thread_work(std::string ip)
{
    //开启相机
    DcHandle handle = 0;
    LxDeviceInfo device_info;
    checkTC(DcOpenDevice(OPEN_BY_IP, ip.c_str(), &handle, &device_info), handle);

    std::cout << "dev cameraid:" << device_info.id << ", uniqueid:" << handle
        << ", cameraip:" << device_info.ip << ", firmware_ver:" << device_info.firmware_ver << ", sn:" << device_info.sn
        << ", name:" << device_info.name << " ,img_algor_ver:" << device_info.algor_ver << std::endl;

    //初始化回调用户数据和注册状态回调
    CameraCallbackData* camCallbackData = new CameraCallbackData();
    camCallbackData->handle = handle;
    camCallbackData->ip = device_info.ip;
    camCallbackData->deviceId = device_info.id;
    camCallbackData->sn = device_info.sn;

    //注册相机状态信息回调
    LX_STATE regStatus = DcRegisterCameraStatusCallback(handle, CameraStatusCallback, camCallbackData);
    if (regStatus != LX_SUCCESS) {
        std::cerr << "  Camera  " << ip << "  callback registration failed:  " << DcGetErrorString(regStatus) << std::endl;
        delete camCallbackData;
        checkTC(regStatus, handle);
    }
    std::cout << "  Camera  " << ip << "  callback registration success!" << std::endl;


    //数据流设置，开启太多可能会存在带宽不够的问题
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, true), handle);
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false), handle);
    bool rgb_enable = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &rgb_enable), handle);
    //开启数据流
    checkTC(DcStartStream(handle), handle);
    //tof参数
    LxIntValueInfo int_value;
    int tof_width = 0, tof_height = 0, tof_data_type = LX_DATA_UNSIGNED_SHORT;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &int_value), handle);
    tof_width = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &int_value), handle);
    tof_height = int_value.cur_value;
    checkTC(DcGetIntValue(handle, LX_INT_3D_DEPTH_DATA_TYPE, &int_value), handle);
    tof_data_type = int_value.cur_value;
    //rgb参数
    int rgb_width = 0, rgb_height = 0, rgb_channles = 0, rgb_data_type = LX_DATA_UNSIGNED_CHAR;
    if (rgb_enable) {
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &int_value), handle);
        rgb_width = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &int_value), handle);
        rgb_height = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_CHANNEL, &int_value), handle);
        rgb_channles = int_value.cur_value;
        checkTC(DcGetIntValue(handle, LX_INT_2D_IMAGE_DATA_TYPE, &int_value), handle);
        rgb_data_type = int_value.cur_value;
    }

    while (true)
    {
        if (wait_key == 'q')
            break;
        //刷新数据
        auto ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);
        if ((LX_SUCCESS != ret)
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            && (LX_E_FRAME_MULTI_MACHINE != ret))
            continue;

        //获取深度数据
        void* depth_data_ptr = nullptr;
        if (DcGetPtrValue(handle, LX_PTR_3D_DEPTH_DATA, &depth_data_ptr) == LX_SUCCESS && depth_data_ptr != nullptr) {
#ifdef HAS_OPENCV
            int cvDepth = mapLXToCvDepth(tof_data_type);
            if (cvDepth >= 0 && tof_width > 0 && tof_height > 0) {
                cv::Mat depth_image(tof_height, tof_width, CV_MAKETYPE(cvDepth, 1), depth_data_ptr);
                cv::Mat depth_show;
                if (cvDepth == CV_16U) {
                    depth_image.convertTo(depth_show, CV_8U, 1.0 / 16.0);
                } else if (cvDepth == CV_32F) {
                    depth_image.convertTo(depth_show, CV_8U, 255.0);
                } else {
                    depth_show = depth_image.clone();
                }

                GuiItem item;
                item.name = "depth" + ip;
                item.img = std::move(depth_show);
                item.width = 640; item.height = 480;
                {
                    std::lock_guard<std::mutex> lk(g_guiMutex);
                    g_guiQueue.push(std::move(item));
                }
                g_guiCond.notify_one();
            }
#endif
        }
        //获取rgb数据
        if (rgb_enable) {
            void* rgb_data_ptr = nullptr;
            if (DcGetPtrValue(handle, LX_PTR_2D_IMAGE_DATA, &rgb_data_ptr) == LX_SUCCESS && rgb_data_ptr != nullptr) {
#ifdef HAS_OPENCV
                int cvDepthRgb = mapLXToCvDepth(rgb_data_type);
                int ch = (rgb_channles > 0 && rgb_channles <= 4) ? rgb_channles : 3;
                if (cvDepthRgb >= 0 && rgb_width > 0 && rgb_height > 0) {
                    cv::Mat rgb_mat(rgb_height, rgb_width, CV_MAKETYPE(cvDepthRgb, ch), rgb_data_ptr);
                    GuiItem item;
                    item.name = "rgb" + ip;
                    item.img = rgb_mat.clone();
                    item.width = 640; item.height = 480;
                    {
                        std::lock_guard<std::mutex> lk(g_guiMutex);
                        g_guiQueue.push(std::move(item));
                    }
                    g_guiCond.notify_one();
                }
#endif
            }
        }
        //统计帧率
        static thread_local auto _time = std::chrono::system_clock::now();
        auto _time2 = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(_time2 - _time).count() > 1)
        {
            _time = std::chrono::system_clock::now();
            LxFloatValueInfo float_info = { 0 };
            DcGetFloatValue(handle, LX_FLOAT_3D_DEPTH_FPS, &float_info);
            std::cout << "  Camera  " << ip << "  Depth FPS:   " << float_info.cur_value << std::endl;

            if (rgb_enable) {
                DcGetFloatValue(handle, LX_FLOAT_2D_IMAGE_FPS, &float_info);
                std::cout << "  Camera  " << ip << "  RGB FPS:   " << float_info.cur_value << std::endl;
            }
        }
    }

    //取消相机状态信息回调注册，并释放内存
    checkTC(DcUnregisterCameraStatusCallback(handle), handle);
    delete camCallbackData;
    camCallbackData = nullptr;

    DcStopStream(handle);
    DcCloseDevice(handle);
    return 0;
}

void CameraStatusCallback(CameraStatus* statusInfo, void* usr_data) {
    if (statusInfo == nullptr || usr_data == nullptr) {
        std::cerr << "Callback error: null pointer parameter!" << std::endl;
        return;
    }

    //获取相机标识
    CameraCallbackData* camData = static_cast<CameraCallbackData*>(usr_data);

    const char* statusDesc = nullptr;
    switch (statusInfo->camera_state) {
    case STATUS_CLOSED:
        statusDesc = "Not on/closed";
        break;
    case STATUS_OPENED_UNSTARTED:
        statusDesc = "Opened but not streamed";
        break;
    case STATUS_STARTED:
        statusDesc = "Turned on and started stream";
        break;
    case STATUS_CONNECTING:
        statusDesc = "The broken line is being reconnected";
        break;
    case STATUS_CONNECT_SUCCESS:
        statusDesc = "The reconnection was successful";
        break;
    default:
        statusDesc = "Unknown";
        break;
    }

    std::cout << "=====================================" << std::endl;
    std::cout << "【Camera status update】" << std::endl;
    std::cout << "  Device IP   : " << camData->ip << std::endl;
    std::cout << "  Device ID    : " << camData->deviceId << std::endl;
    std::cout << "  Device Handle  : " << camData->handle << std::endl;
    std::cout << "  Device SN    : " << camData->sn << std::endl;
    std::cout << "  Current status  : " << statusDesc << "（value：" << statusInfo->camera_state << "）" << std::endl;
    std::cout << "  Update time  : " << statusInfo->status_time << "（timestamp）" << std::endl;
    std::cout << "=====================================" << std::endl << std::endl;

}