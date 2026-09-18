// single_camera.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <string>
#include <thread>
#include "lx_camera_api.h"

#ifdef HAS_OPENCV
#define ENABLE_VISION
#include "opencv2/opencv.hpp"
using namespace cv;
#endif
using namespace std;

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

int TestFrame(DcHandle handle);

bool is_save_depth = false;
bool is_save_rgb = false;
int cols = 0;
void WaitKey()
{
    printf("\n**********press 's' to save********\n");
    printf("\n**********press 'q' to exit********\n");
    while (1) {
        wait_key = getchar();
        if (wait_key == 'q' || wait_key == 'Q')
            break;
        if (wait_key == 's' || wait_key == 'S') {
            is_save_depth = true;
            is_save_rgb = true;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char** argv)
{
    //1、设置全局参数，可略过
    //日志
    DcSetInfoOutput(1, true, "");
    DcSetMaxLogFile(10);
    DcSetMaxLogFileSize(50 * 1024 * 1024);
    //GPU，默认关闭，已适配大多数平台，opencl实现
    DcSetGpuEnable(true);
    //rgb jpeg解码，默认JPEG_DECODE_TURBOJPEG
    DcSetJpegDecodeMethod(JPEG_DECODE_DP);
    //ptp时间同步，默认开启
    DcSetPtpEnable(true);
    //部分平台多线程会有效率问题，默认-1
    DcSetParallelThread(1);
    //版本，也可查看lx_camera_api_version.h，或者strings命令
    printf("call api version: %s\n", DcGetApiVersion());

    //2、查找并打开相机，也可以不查找直接指定IP打开相机
    int device_num = 0;
    LxDeviceInfo* p_device_list = NULL;
    DcGetDeviceList(&p_device_list, &device_num);
    if (device_num <= 0) 
    {
        printf("not found any device, press any key to exit\n");
        wait_key = getchar();
        return -1;
    }
    printf("DcGetDeviceList success list: %d\n", device_num);

    //打开相机。SDK基于GIGE协议，会搜索到所有支持GIGE协议的相机，用索引方式打开相机时需要注意
    //设备打开后会独占权限，其他进程无法再打开相机。如果程序强制结束没有调用DcCloseDevice，需要等待几秒等心跳超时释放权限
    DcHandle handle = 0;
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
    if (LX_SUCCESS != lx_state){
        printf("open device failed, open_mode: %d open_param: %s\n.press any key to exit\n", open_mode, open_param.c_str());
        wait_key = getchar();
        return -1;
    }

    printf("device_info\nid: %s\nip: %s\nsn: %s\nfirmware_ver:%s\n",
        device_info.id, device_info.ip, device_info.sn, device_info.firmware_ver);

    //3、读写相机参数，影响图像尺寸的部分参数必须在StartStream之前设置，否则会报逻辑错误LX_E_FUNCTION_CALL_LOGIC_ERROR

    //获取数据流开启状态，如需调整可以调用DcSetBoolValue。
    bool test_depth = true, test_amp = false, test_rgb = true;
    //开启深度
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, test_depth));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &test_depth));
    //建议关闭强度，强度分辨率会随时跟深度保持一致，rgbd对齐后一般用不上强度
    //checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, test_amp));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_3D_AMP_STREAM, &test_amp));
    //开启rgb图像
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, test_rgb));
    checkTC(DcGetBoolValue(handle, LX_BOOL_ENABLE_2D_STREAM, &test_rgb));

    //可以根据需要,是否开启帧同步模式, 开启该模式, 内部会对每一帧做同步处理后返回
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_SYNC_FRAME, true));

    printf("test_depth: %d test_amp: %d test_rgb: %d\n", test_depth, test_amp, test_rgb);

    //开启对齐，分辨率会保持一致，深度、rgb、点云可以按照同样的像素取值。有多种对齐方式
    checkTC(DcSetIntValue(handle, LX_INT_RGBD_ALIGN_MODE, LX_RGBD_ALIGN_MODE::DEPTH_TO_RGB));
    //设置可能失败，以获取的状态为准
    LxIntValueInfo ivalue;
    checkTC(DcGetIntValue(handle, LX_INT_RGBD_ALIGN_MODE, &ivalue));
    int rgbd_align_mode = ivalue.cur_value;

    //开启深度图像去畸变。低分辨率的深度图像往高分辨率rgb图像对齐，会有很多插值操作。开启深度图像去畸变可以确保对齐的深度图像没有空洞，但占用资源较高。
    checkTC(DcSetBoolValue(handle, LX_BOOL_ENABLE_3D_UNDISTORT, true));
    //设置去畸变系数，尽量保留所有有效数据
    checkTC(DcSetIntValue(handle, LX_INT_3D_UNDISTORT_SCALE, 100));

    //DEPTH_TO_RGB模式下，分辨率以rgb图像为准，可减小分辨率节省算力
    checkTC(DcSetIntValue(handle, LX_INT_2D_BINNING_MODE, LX_BINNING_2X2));

    //获取图像尺寸，因为开启了对齐，二者尺寸相同
    DcGetIntValue(handle, LX_INT_3D_IMAGE_WIDTH, &ivalue);
    int depth_width = ivalue.cur_value;
    DcGetIntValue(handle, LX_INT_3D_IMAGE_HEIGHT, &ivalue);
    int depth_height = ivalue.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_WIDTH, &ivalue);
    int rgb_width = ivalue.cur_value;
    DcGetIntValue(handle, LX_INT_2D_IMAGE_HEIGHT, &ivalue);
    int rgb_height = ivalue.cur_value;
    cols = rgb_width;

    //4、开启数据流，数据流传输的情况下不允许修改图像尺寸，binning，rgbd align，roi等
    checkTC(DcStartStream(handle));

    std::thread pthread = std::thread(WaitKey);
    pthread.detach();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    //5、获取数据
    while (true)
    {
        if (wait_key == 'q' || wait_key == 'Q')
            break;

        //更新数据，此示例为主动更新，也可通过回调函数被动更新
        LX_STATE ret = DcSetCmd(handle, LX_CMD_GET_NEW_FRAME);

        if ((LX_SUCCESS != ret)
            //开启帧同步时，在超时范围内没有找到匹配的帧（丢包），会返回LX_E_FRAME_ID_NOT_MATCH
            && (LX_E_FRAME_ID_NOT_MATCH != ret)
            //多机模式开启时，检测到多机干扰会返回LX_E_FRAME_MULTI_MACHINE
            && (LX_E_FRAME_MULTI_MACHINE != ret))
        {
            //设备断线会自动重连，此时返回LX_E_RECONNECTING
            if (LX_E_RECONNECTING == ret) {
                printf("device reconnecting\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        TestFrame(handle);
    }

    //6、停止数据流，相机进入待机状态（WORK_FOREVER会持续工作，不受此接口影响）
    DcStopStream(handle);
    //7、关闭相机，释放权限和资源
    DcCloseDevice(handle);

    return 0;
}


int TestFrame(DcHandle handle)
{
    //读取yRow行xCol列的数据
    int yRow = 10, xCol = 20;
    //对齐模式下，分辨率相同，可以用同样的像素索引
    int index = yRow * cols + xCol;

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
        printf("new frame not correct. error code %d\n", frame_ptr->frame_state);
        return 1;
    }

    //帧ID信息，可以用来识别每个数据流的同步，否则可以忽略
    FrameExtendInfo* pextendframe = nullptr;
    if (frame_ptr->reserve_data != nullptr)
        pextendframe = (FrameExtendInfo*)frame_ptr->reserve_data;

    //深度数据
    if (frame_ptr->depth_data.frame_data != nullptr)
    {
        FrameDataInfo depth_data = frame_ptr->depth_data;
        //printf("depth_sensor_time:%lld\n", depth_data.sensor_timestamp);
        //if (pextendframe != nullptr)
         //   printf("depth_frame_id:%d\n", pextendframe->depth_frame_id);

        //将depth数据转换为xyz点云数据
        float* xyz_ptr = NULL;
        DcGetPtrValue(handle, LX_PTR_XYZ_DATA, (void**)&xyz_ptr);

        //深度数据主要是unsigned short格式，具体要根据frame_ptr->depth_data.frame_data_type判断
        unsigned short* depth_ptr = (unsigned short*)frame_ptr->depth_data.frame_data;
        unsigned short depth_value = depth_ptr[index];
        //点云是三通道float格式
        float x = xyz_ptr[index * 3];
        float y = xyz_ptr[index * 3 + 1];
        float z = xyz_ptr[index * 3 + 2];

#ifdef HAS_OPENCV
        cv::Mat depth_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_MAKETYPE(depth_data.frame_data_type, depth_data.frame_channel), depth_data.frame_data).clone();
        cv::Mat xyz_image = cv::Mat(depth_data.frame_height, depth_data.frame_width,
            CV_32FC3, xyz_ptr).clone();
        if (is_save_depth) {
            is_save_depth = true;
            cv::imwrite("depth.pgm", depth_image);
            DcSaveXYZ(handle, "xyz.pcd");
        }
#ifdef ENABLE_VISION
        cv::Mat show;
        depth_image.convertTo(show, CV_8U, 1.0 / 16);
        cv::applyColorMap(show, show, COLORMAP_JET);
        cv::namedWindow("depth", 0);
        cv::resizeWindow("depth", 640, 480);
        cv::imshow("depth", show);
        cv::waitKey(1);
#endif
#endif
    }

    //rgb数据
    if (frame_ptr->rgb_data.frame_data != nullptr)
    {
        auto rgb_data = frame_ptr->rgb_data;
        //printf("rgb_sensor_time:%lld\n", rgb_data.sensor_timestamp);
        //if (pextendframe != nullptr)
        //    printf("rgb_frame_id:%d\n", pextendframe->rgb_frame_id);

        //rgb一般是三通道unsigned char格式，具体可通过枚举或者frame_ptr->rgb_data获取
        unsigned char* rgb_ptr = (unsigned char* )frame_ptr->rgb_data.frame_data;
        unsigned char r = rgb_ptr[index * 3];
        unsigned char g = rgb_ptr[index * 3 + 1];
        unsigned char b = rgb_ptr[index * 3 + 2];

#ifdef HAS_OPENCV
        cv::Mat rgb_image = cv::Mat(rgb_data.frame_height, rgb_data.frame_width,
            CV_MAKETYPE(rgb_data.frame_data_type, rgb_data.frame_channel), rgb_data.frame_data).clone();
        if (is_save_rgb) {
            is_save_rgb = false;
            cv::imwrite("rgb.png", rgb_image);
        }

#ifdef ENABLE_VISION
        cv::namedWindow("rgb", 0);
        cv::resizeWindow("rgb", 640, 480);
        cv::imshow("rgb", rgb_image);
        cv::waitKey(1);
#endif
#endif
    }

    return 0;
}
