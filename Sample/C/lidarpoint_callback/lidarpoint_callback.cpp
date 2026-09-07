//frame_callback.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <string>
#include <thread>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "lx_camera_api.h"

#ifdef HAS_OPENCV
#define ENABLE_VISION
#include "opencv2/opencv.hpp"
using namespace cv;
#endif

static char wait_key = '0';
static bool save_pcd_requested = false;
static LxPointCloudData* latest_pointcloud_data = nullptr;
#define checkTC(state) {LX_STATE val=state;                             \
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

void CallbackFunc(FrameInfo* frame, void* usr_data);
bool SavePointCloudToPCD(const LxPointCloudData* pointcloud_data, const std::string& filename);
int main(int argc, char** argv)
{
    DcHandle handle = 0;

    //设置日志等级和路径。
    checkTC(DcSetInfoOutput(1, true, ""));
    printf("call api version: %s\n", DcGetApiVersion());

    //查找设备
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

    //打开相机。SDK基于GIGE协议，会搜索到所有支持GIGE协议的相机，用索引方式打开相机时需要注意
    //设备打开后会独占权限，其他进程无法再打开相机。如果程序强制结束没有调用DcCloseDevice，需要等待几秒等心跳超时释放权限
    std::string open_param;
    int open_mode = OPEN_BY_INDEX; //OPEN_BY_IP;//
    switch (open_mode)
    {
        //根据ip打开设备
    case OPEN_BY_IP:
        open_param = "192.168.100.82";
        break;
        //根据sn打开设备
    case OPEN_BY_SN:
        open_param = "ccf8981cc50b66b6";
        break;
        //根据搜索设备列表索引打开设备
    case OPEN_BY_INDEX:
    default:
        open_param = "0";
        break;
    }

    LxDeviceInfo device_info;
    LX_STATE lx_state = DcOpenDevice((LX_OPEN_MODE)open_mode, open_param.c_str(), &handle, &device_info);

    if (LX_SUCCESS != lx_state) {
        printf("open device failed, open_mode: %d open_param: %s\n.press any key to exit\n", open_mode, open_param.c_str());
        wait_key = getchar();
        return -1;
    }
    printf("device_info\nid: %s\nip: %s\nsn: %s\nfirmware_ver:%s\n",
        device_info.id, device_info.ip, device_info.sn, device_info.firmware_ver);

    //获取数据流状态
    bool depth_enable = false, amp_enable = false, rgb_enable = false;
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &depth_enable));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &amp_enable));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &rgb_enable));

    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_SYNC_FRAME, true));

    //关闭RGB图像处理，节省解码和其它图像处理占用的资源
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_RGB_PROC, false));

    //获取内置应用算法状态
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

    //设置回调
    checkTC(DcRegisterFrameCallback(handle, CallbackFunc, nullptr));

    //启流
    checkTC(DcStartStream(handle));

    if (trigger_mode == LX_TRIGGER_MODE::LX_TRIGGER_SOFTWARE) {
        std::thread trigger_pthread = std::thread([&]() {
            do {
                //软触发模式需要自行控制触发，这里只是示例
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

    printf("**********press 'q' to exit, 's' to save point cloud********\n\n");
    while (wait_key != 'q')
    {
        wait_key = getchar();
        if (wait_key == 's' || wait_key == 'S') {
            printf("请求保存数据...\n");
            save_pcd_requested = true;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    checkTC(DcUnregisterFrameCallback(handle));
    checkTC(DcStopStream(handle));
    checkTC(DcCloseDevice(handle));
    return 0;
}

bool SavePointCloudToPCD(const LxPointCloudData* pointcloud_data, const std::string& filename)
{
    if (pointcloud_data == nullptr) {
        printf("点云数据为空，无法保存\n");
        return false;
    }

    uint32_t total_points = pointcloud_data->point_num;
    if (total_points == 0) {
        printf("点云数据中没有有效点，无法保存\n");
        return false;
    }

    std::ofstream file(filename, std::ios::out);
    if (!file.is_open()) {
        printf("无法创建文件: %s\n", filename.c_str());
        return false;
    }

    // 写入PCD文件头
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z intensity time\n";
    file << "SIZE 4 4 4 4 2\n";
    file << "TYPE F F F U I\n";
    file << "COUNT 1 1 1 1 1\n";
    file << "WIDTH " << total_points << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << total_points << "\n";
    file << "DATA ascii\n";

    // 写入点云数据
    for (uint32_t i = 0; i < total_points; ++i) {
        const LxPointXYZIRT& point = pointcloud_data->points[i];
        file << std::fixed << std::setprecision(6)
             << point.x << " " << point.y << " " << point.z
            << " " << point.intensity << " " << point.offset_time << "\n";
    }

    file.close();
    printf("点云数据已保存到: %s (总点数: %u)\n", filename.c_str(), total_points);
    return true;
}

int PrintPcd(void* data_ptr)
{
    if (data_ptr == nullptr)
        return 0;

    LxPointCloudData* roidata = (LxPointCloudData*)data_ptr;

    // 保存最新的点云数据供后续保存使用
    latest_pointcloud_data = roidata;

    // 如果用户请求保存点云
    if (save_pcd_requested) {
        //生成带时间戳的文件
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        std::stringstream ss;
        ss << "SavePointCloudToPCD_" << std::to_string(timestamp) << ".pcd";

        if (SavePointCloudToPCD(roidata, ss.str())) {
            printf("SavePointCloudToPCD点云保存成功！\n");
        } else {
            printf("SavePointCloudToPCD点云保存失败！\n");
        }

        save_pcd_requested = false;
    }

    return 0;
}

void CallbackFunc(FrameInfo* frame_ptr, void* usr_data)
{
    //此函数内部,尽量不要做复杂操作,否则可能阻塞内部数据回调
    //回调方式不需要再调用LX_CMD_GET_NEW_FRAME
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

    //若需要帧id信息,可选通过此结构体获取, 若需要帧同步, 则可通过将此处接收到的数据缓存到队列中,后续在队列中匹配帧id方式实现
    FrameExtendInfo* p_extend_info = nullptr;
    if (frame_ptr->reserve_data != nullptr)
        p_extend_info = (FrameExtendInfo*)frame_ptr->reserve_data;

    //depth
    if (frame_ptr->depth_data.frame_data != nullptr)
    {
        printf("depth sensor_timestamp: %lld\n", frame_ptr->depth_data.sensor_timestamp);
        if (p_extend_info != nullptr)
            printf("depth frame_id:%d\n", p_extend_info->depth_frame_id);

        //获取点雷达点云数据。点云数据由深度和强度数据转换，与深度数据同步更新
        void* xyzirt_data = nullptr;
        if (DcGetPtrValue(frame_ptr->handle, LX_PTR_XYZIRT_DATA, (void**)&xyzirt_data) != LX_SUCCESS)
            printf("get LX_PTR_XYZ_DATA failed");

        PrintPcd(xyzirt_data);
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
    if (frame_ptr->depth_data.frame_data != nullptr) {
        FrameDataInfo depth_data = frame_ptr->depth_data;
        cv::Mat depth_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_MAKETYPE(depth_data.frame_data_type, depth_data.frame_channel), depth_data.frame_data);

#ifdef ENABLE_VISION
        cv::Mat show;
        depth_image.convertTo(show, CV_8U, 1.0 / 16);
        applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("depth", 0);
        cv::resizeWindow("depth", 640, 480);
        cv::imshow("depth", show);
#endif //ENABLE_VISION
    }

    if (frame_ptr->amp_data.frame_data != nullptr) {
        FrameDataInfo amp_data = frame_ptr->amp_data;
        cv::Mat amp_image = cv::Mat(amp_data.frame_height, amp_data.frame_width,
            CV_MAKETYPE(amp_data.frame_data_type, amp_data.frame_channel), amp_data.frame_data);
#ifdef ENABLE_VISION
        cv::Mat show;
        if (amp_image.type() == CV_16U)
            amp_image.convertTo(show, CV_8U, 1.0 / 8);
        else
            show = amp_image;

        cv::namedWindow("amp", 0);
        cv::resizeWindow("amp", 640, 480);
        cv::imshow("amp", show);
#endif //ENABLE_VISION
    }

    if (frame_ptr->rgb_data.frame_data != nullptr) {
        FrameDataInfo rgb_data = frame_ptr->rgb_data;
        cv::Mat rgb_image = cv::Mat(rgb_data.frame_height, rgb_data.frame_width,
            CV_MAKETYPE(rgb_data.frame_data_type, rgb_data.frame_channel), rgb_data.frame_data);
#ifdef ENABLE_VISION
        cv::namedWindow("rgb", 0);
        cv::resizeWindow("rgb", 640, 480);
        cv::imshow("rgb", rgb_image);
#endif //ENABLE_VISION
    }

#ifdef ENABLE_VISION
    cv::waitKey(1);
#endif //ENABLE_VISION

#endif //HAS_OPENCV
    
    return;
}
