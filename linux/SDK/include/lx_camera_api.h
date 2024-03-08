
#ifndef _LX_CAMERA_API_H_
#define _LX_CAMERA_API_H_

#include "lx_camera_define.h"

//功能: 获取API版本号
//参数：[out]version  API版本号，无需外部分配内存
LX_API_STR DcGetApiVersion();

//功能: 设置打印信息等级
//参数：[in]print_level 0：info     所有调试信息
//                      1：warn     重要及警告类调试信息
//                      2：error    仅输出错误信息
//      [in]enable_screen_print 是否在窗口打印
//      [in]log_path            log文件保存路径
#ifdef __cplusplus
LX_API DcSetInfoOutput(int print_level = 1, bool enable_screen_print = false, const char* log_path = "", int language = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcSetInfoOutput(int print_level, bool enable_screen_print, const char* log_path);
#endif

//功能: 允许用户输出调试信息到log文件
//参数：[in]str     要输出的字符串，'\0'结尾
LX_API DcLog(const char* str);


//功能：查找支持的相机
//参数  [out]devlist            查找到的相机列表
//参数  [out]devnum             查找到的相机数量
#ifdef __cplusplus
LX_EXPORT LX_STATE LX_STDC DcGetDeviceList(LxDeviceInfo** devlist, int* devnum, LX_DEVICE_SERIALS lx_serials = LX_SERIAL_ALL);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcGetDeviceList(LxDeviceInfo** devlist, int* devnum);
#endif


//功能：连接设备
//参数：[in]open_mode           打开方式, 具体说明见LX_OPEN_MODE
//      [in]param               不同的打开方式，填写不同的参数(当sdk与相机端程序运行在同一主机情况下，此处填写"127.0.0.1")
//      [out]handle             连接成功后返回的设备句柄,后续所有接口访问都依赖该handle字段
//      [out]info               连接成功后返回的相机详细信息
LX_API DcOpenDevice(LX_OPEN_MODE open_mode, const char* param, DcHandle* handle, LxDeviceInfo* info);


//功能: 关闭设备
//参数：[in]handle  设备句柄
LX_API DcCloseDevice(DcHandle handle);

//功能: 打开数据流
//参数：[in]handle  设备句柄
LX_API DcStartStream(DcHandle handle);

//功能: 关闭数据流
//参数：[in]handle  设备句柄
LX_API DcStopStream(DcHandle handle);

//功能: 保存点云，可直接调用
//参数：[in]handle                设备句柄
//      [in]filename              文件名，支持txt,ply和pcd格式。txt格式按图像顺序保存所有数据，ply和pcd仅保存非零数据
LX_API DcSaveXYZ(DcHandle handle, const char* filename);

//功能: 设置相机IP相关参数
//      修改完之后设备列表会变化，需重新调用DcGetDeviceList接口重新获取新的设备列表
//参数：[in]handle                 设备句柄, (当未连接上设备情况下，需先通过搜索获取)
//      [in]ip                     设备IP
//      [in]netmask                子网掩码(若传空则内部默认"255.255.0.0")
//      [in]gateway                网关ip(若传空则内部默认将ip最后网段置为"1"后作为网关)
#ifdef __cplusplus
LX_API DcSetCameraIp(DcHandle handle, const char* ip, const char* netmask = 0, const char* gateway = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcSetCameraIp(DcHandle handle, const char* ip, const char* netmask, const char* gateway);
#endif

//功能：设置int类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
LX_API DcSetIntValue(DcHandle handle, int cmd, int value);

//功能：获取int类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数结构体
LX_API DcGetIntValue(DcHandle handle, int cmd, LxIntValueInfo* value);

//功能：设置float类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
LX_API DcSetFloatValue(DcHandle handle, int cmd, float value);

//功能：获取float类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数结构体
LX_API DcGetFloatValue(DcHandle handle, int cmd, LxFloatValueInfo* value);

//功能：设置bool类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
LX_API DcSetBoolValue(DcHandle handle, int cmd, bool value);

//功能：获取bool类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数
LX_API DcGetBoolValue(DcHandle handle, int cmd, bool* value);

//功能：设置string类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
LX_API DcSetStringValue(DcHandle handle, int cmd, const char* value);

//功能：获取string类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数，无需外部分配内存
LX_API DcGetStringValue(DcHandle handle, int cmd, char** value);

//功能：获取指针类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数，无需外部分配内存
LX_API DcGetPtrValue(DcHandle handle, int cmd, void** value);

//功能：执行对应CMD类型指令操作
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
LX_API DcSetCmd(DcHandle handle, int cmd);


//功能：LX_CAMERA_FEATURE定义之外的特殊操作，具体功能和参数由字符串command确定
//参数：[in]handle                 设备句柄
//      [in]command                指令
//      [in][out]value             设置时为对应入参，获取时为对应出参，无需外部分配内存
LX_API DcSpecialControl(DcHandle handle, const char* command, void* value);


//功能：设置ROI区域, 输入数值若不是8的整数倍,内部会自动处理为目标值最近的8的整倍数
//参数：[in]handle                  设备句柄
//      [in]offsetx                 起始点水平偏移像素
//      [in]offsety                 起始点垂直偏移参数
//      [in]width                   roi目标区域的宽
//      [in]height                  roi目标区域的高
//      [in]type                    0-3D图像 1-2D图像
LX_API DcSetROI(DcHandle handle, int offsetx, int offsety, int width, int height, int img_type);


//功能: 注册数据帧回调函数,收到新的数据时自动调用
//参数：[in]handle      设备句柄
//      [in]func        回调函数指针
//      [in]usr_data    用户自定义参数
#ifdef __cplusplus
LX_API DcRegisterFrameCallback(DcHandle handle, LX_FRAME_CALLBACK func, void* usr_data = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcRegisterFrameCallback(DcHandle handle, LX_FRAME_CALLBACK func, void* usr_data);
#endif

//功能: 取消数据帧注册回调函数
//参数：[in]handle      设备句柄
LX_API DcUnregisterFrameCallback(DcHandle handle);


//功能: 注册相机状态回调函数,相机状态变化时自动调用
//参数：[in]handle      设备句柄
//      [in]func        回调函数指针
//      [in]usr_data    用户自定义参数
#ifdef __cplusplus
LX_API DcRegisterCameraStatusCallback(DcHandle handle, LX_CAMERA_STATUS_CALLBACK func, void* usr_data = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcRegisterCameraStatusCallback(DcHandle handle, LX_CAMERA_STATUS_CALLBACK func, void* usr_data);
#endif

//功能: 取消相机状态注册回调函数
//参数：[in]handle      设备句柄
LX_API DcUnregisterCameraStatusCallback(DcHandle handle);

//功能：设置参数路径
//参数：[in]handle                  设备句柄
//      [in]filepath                参数文件路径，不包含文件名(大部分情况不需要调用，仅部分相型号机打开时需要参数，如M3，I2等)
LX_API DcSetParamPath(DcHandle handle, const char* filepath);

//功能: 获取函数返回状态对应的说明
//参数：[in]state  函数接口返回的状态
LX_API_STR DcGetErrorString(LX_STATE state);

#endif
