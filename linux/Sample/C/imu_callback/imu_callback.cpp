#include <string>
#include <thread>
#include <vector>
#include "lx_camera_api.h"
using namespace std;

// 全局变量：用于统计IMU频率
static int64_t last_imu_timestamp = 0;
static int imu_count = 0;
static double imu_freq = 0.0;

// 滑动窗口参数：使用最近20个间隔计算平均频率
#define IMU_FREQ_WINDOW_SIZE 20
static vector<int64_t> interval_buffer;  // 存储时间间隔的缓冲区

static char wait_key = '0';
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

void ImuDataCallback(LxImuData* imu_data, void* user_data);

void WaitKey()
{
    printf("**********press 'q' to exit********\n\n");
    wait_key = getchar();
}

int main(int argc, char** argv)
{
    DcHandle handle = 0;
    checkTC(DcSetInfoOutput(1, true, ""));
    printf("call api version: %s\n", DcGetApiVersion());

    //查找相机
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

    //打开相机
    std::string open_param;
    int open_mode = OPEN_BY_INDEX;
    switch (open_mode)
    {
        //根据ip打开设备
    case OPEN_BY_IP:
        open_param = "192.168.100.82";
        break;
        //根据id打开设备
    case OPEN_BY_ID:
        open_param = "F13301122647";
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

    //注册IMU数据回调函数
    checkTC(DcRegisterImuDataCallback(handle, ImuDataCallback, nullptr));

    DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, false);
    DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, false);
    DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, false);
    //使能IMU
    DcSetBoolValue(handle, LX_BOOL_ENABLE_IMU, true);
    //设置IMU量程
    DcSetIntValue(handle, LX_INT_IMU_ACCELERATION_LEVEL, 0);
    DcSetIntValue(handle, LX_INT_IMU_ANGULAR_RANGE_LEVEL, 0);

    DcStartStream(handle);

    std::thread pthread = std::thread(WaitKey);
    pthread.detach();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    wait_key = getchar();

    DcUnregisterImuDataCallback(handle);
    DcStopStream(handle);
    DcCloseDevice(handle);

    return 0;
}


void ImuDataCallback(LxImuData* imu_data, void* usr_data) {
    if (!imu_data) return;

    // 计算当前与上一帧的时间间隔
    if (last_imu_timestamp != 0 && imu_data->imu_data.sensor_timestamp > last_imu_timestamp) {
        int64_t interval_us = imu_data->imu_data.sensor_timestamp - last_imu_timestamp;
        
        interval_buffer.push_back(interval_us);
        
        // 保持缓冲区大小不超过窗口大小
        if (interval_buffer.size() > IMU_FREQ_WINDOW_SIZE) {
            interval_buffer.erase(interval_buffer.begin());
        }
        
        // 计算平均频率
        if (!interval_buffer.empty()) {
            int64_t total_interval = 0;
            for (int64_t interval : interval_buffer) {
                total_interval += interval;
            }
            // 计算平均间隔（微秒）并转换为频率（Hz）
            double avg_interval_us = static_cast<double>(total_interval) / interval_buffer.size();
            imu_freq = 1000000.0 / avg_interval_us;  
        }
    }
    
    last_imu_timestamp = imu_data->imu_data.sensor_timestamp;
    imu_count++;
  
    printf("\n");
    printf("Timestamp: %lld (us)\n", imu_data->imu_data.sensor_timestamp);
    printf("Frequency: %.1f Hz (window size: %d)\n", imu_freq, IMU_FREQ_WINDOW_SIZE);
    printf("Acceleration (m/s^2): X=%.4f, Y=%.4f, Z=%.4f\n",
        imu_data->imu_data.acc_x, imu_data->imu_data.acc_y, imu_data->imu_data.acc_z);
    printf("Angular Velocity (rad/s): X=%.4f, Y=%.4f, Z=%.4f\n",
        imu_data->imu_data.gry_x, imu_data->imu_data.gry_y, imu_data->imu_data.gry_z);
}
