
#ifndef _LX_CAMERA_API_H_
#define _LX_CAMERA_API_H_

#include "lx_camera_define.h"

//~chinese:
//功能: 获取API版本号
//参数：[out]version  API版本号，无需外部分配内存
//~english:
/**
 * @brief Get api version
 * 
 * @return LX_API_STR 
 */
LX_API_STR DcGetApiVersion();

//~chinese:
//功能: 设置打印信息等级
//参数：[in]print_level 0：info     所有调试信息
//                      1：warn     重要及警告类调试信息
//                      2：error    仅输出错误信息
//      [in]enable_screen_print 是否在窗口打印
//      [in]log_path            log文件保存路径，可以为空，则保存在当前执行路径
//~english:
/**
 * @brief Set the level of log
 * 
 * @param print_level [in]print_level 0: info All debug messages 1: warn Important and warning debug messages 2: error Only error messages are output.
 * @param enable_screen_print [in]Whether to print in the window
 * @param log_path [in]Log file save path
 * @return LX_API 
 */
#ifdef __cplusplus
LX_API DcSetInfoOutput(int print_level = 1, bool enable_screen_print = false, const char* log_path = "", int language = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcSetInfoOutput(int print_level, bool enable_screen_print, const char* log_path);
#endif

//~chinese:
//功能: 允许用户输出调试信息到log文件
//参数：[in]str     要输出的字符串，'\0'结尾
//~english:
/**
 * @brief Allows users to output debugging information to log files.
 * 
 * @param str [in]String to be output, ending with '\0'.
 * @return LX_API 
 */
LX_API DcLog(const char* str);

//~chinese:
//功能：查找支持的相机
//参数  [out]devlist            查找到的相机列表
//参数  [out]devnum             查找到的相机数量
//~english:
/**
 * @brief Search for camera list
 * 
 * @param devlist [out]List of found cameras
 * @param devnum [out]Number of cameras found
 * @return LX_API 
 */
#ifdef __cplusplus
LX_EXPORT LX_STATE LX_STDC DcGetDeviceList(LxDeviceInfo** devlist, int* devnum, LX_DEVICE_SERIALS lx_serials = LX_SERIAL_ALL);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcGetDeviceList(LxDeviceInfo** devlist, int* devnum);
#endif

//~chinese:
//功能：打开设备。设备打开后会独占权限，其他进程无法再打开相机。如果程序强制结束没有调用DcCloseDevice，需要等待几秒等心跳超时释放权限
//参数：[in]open_mode           打开方式, 具体说明见LX_OPEN_MODE
//      [in]param               不同的打开方式，填写不同的参数(当sdk运行在相机内部时，此处填写"127.0.0.1")
//      [out]handle             连接成功后返回的设备句柄,后续所有接口访问都依赖该handle字段
//      [out]info               连接成功后返回的相机详细信息
//~english:
/**
 * @brief open device  
 * 
 * @param open_mode[in]Open mode, see LX_OPEN_MODE for details. 
 * @param param[in]Fill in different parameters for different opening methods (when the sdk is running inside the camera, fill in "127.0.0.1" here) 
 * @param handle[out]The handle of the device returned after a successful connection, all subsequent interface accesses rely on this handle field. 
 * @param info[out]Camera details returned after successful connection 
 * @return LX_API 
 */
LX_API DcOpenDevice(LX_OPEN_MODE open_mode, const char* param, DcHandle* handle, LxDeviceInfo* info);

//~chinese
//功能: 关闭设备。与DcOpenDevice对应，释放权限和资源。
//参数：[in]handle  设备句柄
//~english
/**
 * @brief Close device
 * 
 * @param handle[in]Device handle 
 * @return LX_API 
 */
LX_API DcCloseDevice(DcHandle handle);

//~chinese
//功能: 打开数据流
//参数：[in]handle  设备句柄
//~english
/**
 * @brief Start stream data
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcStartStream(DcHandle handle);

//~chinese
//功能: 关闭数据流
//参数：[in]handle  设备句柄
//~english
/**
 * @brief stop stream data
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcStopStream(DcHandle handle);

//~chinese
//功能: 设置相机IP相关参数
//      修改完之后设备列表会变化，需重新调用DcGetDeviceList接口重新获取新的设备列表
//参数：[in]handle                 设备句柄, (当未连接上设备情况下，需先通过搜索获取)
//      [in]ip                     设备IP
//      [in]netmask                子网掩码(若传空则内部默认"255.255.0.0")
//      [in]gateway                网关ip(若传空则内部默认将ip最后网段置为"1"后作为网关)
//~english
/**
 * @brief Set camera IP-related parameters
 * 
 * @param handle[in]Device handle  
 * @param ip[in]Device IP
 * @param netmask[in]Subnet Mask (if empty, internal default is "255.255.0.0") 
 * @param gateway[in]gateway ip (if pass empty, internal default will set the last segment of ip to "1" as gateway) 
 * @return LX_API 
 */
#ifdef __cplusplus
LX_API DcSetCameraIp(DcHandle handle, const char* ip, const char* netmask = 0, const char* gateway = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcSetCameraIp(DcHandle handle, const char* ip, const char* netmask, const char* gateway);
#endif

//~chinese
//功能：设置int类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
//~english
/**
 * @brief Set int type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in]Setting parameter values 
 * @return LX_API 
 */
LX_API DcSetIntValue(DcHandle handle, int cmd, int value);

//~chinese
//功能：获取int类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数结构体
//~english
/**
 * @brief Obtain int type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter structure 
 * @return LX_API 
 */
LX_API DcGetIntValue(DcHandle handle, int cmd, LxIntValueInfo* value);

//~chinese
//功能：设置float类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
//~english
/**
 * @brief Set float type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in]Setting parameter values 
 * @return LX_API 
 */
LX_API DcSetFloatValue(DcHandle handle, int cmd, float value);

//~chinese
//功能：获取float类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数结构体
//~english
/**
 * @brief Obtain float type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter structure 
 * @return LX_API 
 */
LX_API DcGetFloatValue(DcHandle handle, int cmd, LxFloatValueInfo* value);

//~chinese
//功能：设置bool类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
//~english
/**
 * @brief Set bool type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in]Setting parameter values 
 * @return LX_API 
 */
LX_API DcSetBoolValue(DcHandle handle, int cmd, bool value);

//~chinese
//功能：获取bool类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数
//~english
/**
 * @brief Obtain bool type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return Parameters 
 * @return LX_API 
 */
LX_API DcGetBoolValue(DcHandle handle, int cmd, bool* value);

//~chinese
//功能：设置string类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
//~english
/**
 * @brief Set string type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in] 
 * @return LX_API 
 */
LX_API DcSetStringValue(DcHandle handle, int cmd, const char* value);

//~chinese
//功能：获取string类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数，无需外部分配内存
//~english
/**
 * @brief Obtain string type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter, no external memory allocation required 
 * @return LX_API 
 */
LX_API DcGetStringValue(DcHandle handle, int cmd, char** value);

//~chinese
//功能：获取指针类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [out]value                  返回参数，无需外部分配内存
//~english
/**
 * @brief Obtain pointer type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter, no external memory allocation required 
 * @return LX_API 
 */
LX_API DcGetPtrValue(DcHandle handle, int cmd, void** value);

//~chinese
//功能：执行对应CMD类型指令操作
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//~english
/**
 * @brief Execute the corresponding CMD type command
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @return LX_API 
 */
LX_API DcSetCmd(DcHandle handle, int cmd);

//~chinese
//功能：LX_CAMERA_FEATURE定义之外的特殊操作，具体功能和参数由字符串command确定
//参数：[in]handle                 设备句柄
//      [in]command                指令
//      [in][out]value             设置时为对应入参，获取时为对应出参，无需外部分配内存
//~english
/**
 * @brief Special operations outside the definition of LX_CAMERA_FEATURE, with specific functions and parameters defined by the string command
 * 
 * @param handle[in]Device handle  
 * @param command[in]Command      
 * @param value[in][out]Input parameter when setting, output parameter when getting, no need to allocate memory externally. 
 * @return LX_API 
 */
LX_API DcSpecialControl(DcHandle handle, const char* command, void* value);


//~chinese
//功能：设置ROI区域, 输入数值若不是8的整数倍,内部会自动处理为目标值最近的8的整倍数
//参数：[in]handle                  设备句柄
//      [in]offsetx                 起始点水平偏移像素
//      [in]offsety                 起始点垂直偏移参数
//      [in]width                   roi目标区域的宽
//      [in]height                  roi目标区域的高
//      [in]img_type                0-3D图像 1-2D图像
//~english
/**
 * @brief Set the ROI area, if the input value is not an integer multiple of 8, it will be automatically processed as the nearest integer multiple of 8 to the target value.
 * 
 * @param handle[in]Device handle  
 * @param offsetx[in]Horizontal offset of the start point in pixels 
 * @param offsety[in]Start Point Vertical Offset Parameter 
 * @param width[in]The width of the roi target area 
 * @param height[in]The height of the roi target area 
 * @param img_type[in] 0-3D image 1-2D image 
 * @return LX_API 
 */
LX_API DcSetROI(DcHandle handle, int offsetx, int offsety, int width, int height, int img_type);


//~chinese
//功能: 注册数据帧回调函数,收到新的数据时自动调用
//参数：[in]handle      设备句柄
//      [in]func        回调函数指针
//      [in]usr_data    用户自定义参数
//~english
/**
 * @brief Register the data frame callback function to be called automatically when new data is received.
 * 
 * @param handle[in]Device handle  
 * @param func[in]Callback function pointer 
 * @param usr_data[in]User-defined parameters 
 * @return LX_API 
 */
#ifdef __cplusplus
LX_API DcRegisterFrameCallback(DcHandle handle, LX_FRAME_CALLBACK func, void* usr_data = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcRegisterFrameCallback(DcHandle handle, LX_FRAME_CALLBACK func, void* usr_data);
#endif

//~chinese
//功能: 取消数据帧注册回调函数
//参数：[in]handle      设备句柄
//~english
/**
 * @brief Cancel data frame registration callback function
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcUnregisterFrameCallback(DcHandle handle);

//~chinese
//功能: 注册相机状态回调函数,相机状态变化时自动调用
//参数：[in]handle      设备句柄
//      [in]func        回调函数指针
//      [in]usr_data    用户自定义参数
//~english
/**
 * @brief Register the camera state callback function and call it automatically when the camera state changes.
 * 
 * @param handle[in]Device handle  
 * @param func[in]Callback function pointer 
 * @param usr_data[in]User-defined parameters 
 * @return LX_API 
 */
#ifdef __cplusplus
LX_API DcRegisterCameraStatusCallback(DcHandle handle, LX_CAMERA_STATUS_CALLBACK func, void* usr_data = 0);
#else
//纯C风格接口，原接口逐步弃用
LX_API DcRegisterCameraStatusCallback(DcHandle handle, LX_CAMERA_STATUS_CALLBACK func, void* usr_data);
#endif

//~chinese
//功能: 取消相机状态注册回调函数
//参数：[in]handle      设备句柄
//~english
/**
 * @brief Cancel camera state registration callback function
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcUnregisterCameraStatusCallback(DcHandle handle);

//~chinese
//功能：往设备写入自定义内容
//参数：[in]handle             设备句柄
//      [in]start_address         起始地址，从0开始
//      [in]data               写入内存的内容，最大支持128kb内容
//      [in]data_size          写入内存的内容长度，最大不超过128kb
//~english
/**
 * @brief Write custom content to device
 * 
 * @param handle[in]Device handle  
 * @param start_address[in]Starting address, from 0 
 * @param data[in]Content written to memory, supports up to 128kb of content 
 * @param data_size[in]Length of content written to memory, max. 128kb 
 * @return LX_API 
 */
LX_API DcWriteUserData(DcHandle handle, int start_address, char* data, int data_size);

//~chinese
//功能：从设备读取自定义内容,无需外部分配内存，但每次调用读取前会重置上次读取时分配的内存
//参数：[in]handle             设备句柄
//      [in]start_address      起始地址，从0开始
//      [out]data              读取内存的内容，最大支持128kb内容
//      [out]data_size         读取的内容长度，最大不超过128kb
//~english
/**
 * @brief Read customized content from the device without external memory allocation, but each call to read will reset the memory allocated during the last read.
 * 
 * @param handle[in]Device handle  
 * @param start_address[in]Starting address, from 0 
 * @param data[out]Read the contents of the memory, maximum support for 128kb content 
 * @param data_size[out]Length of read content, max. 128kb 
 * @return LX_API 
 */
LX_API DcReadUserData(DcHandle handle, int start_address, char** data, int& data_size);

//~chinese
//功能: 保存点云，可直接调用
//参数：[in]handle                设备句柄
//      [in]filename              文件名，支持txt,ply和pcd格式。txt格式按图像顺序保存所有数据，ply和pcd仅保存非零数据
//~english
/**
 * @brief Save the point cloud, which can be called directly
 * 
 * @param handle[in]Device handle  
 * @param filename[in]File name, supports txt, ply and pcd formats. txt format saves all data in image order, ply and pcd only save non-zero data 
 * @return LX_API 
 */
LX_API DcSaveXYZ(DcHandle handle, const char* filename);

//~chinese
//功能：设置参数路径
//参数：[in]handle                  设备句柄
//      [in]filepath                参数文件路径，不包含文件名(大部分情况不需要调用，仅部分相型号机打开时需要参数，如M3，I2等)
//~english
/**
 * @brief Setting the parameter path
 * 
 * @param handle[in]Device handle  
 * @param filepath[in]Parameter file path, without file name (most cases do not need to call, only part of the camera type machine to open the parameters needed, such as M3, I2, etc.) 
 * @return LX_API 
 */
LX_API DcSetParamPath(DcHandle handle, const char* filepath);

//~chinese
//功能: 获取函数返回状态对应的说明
//参数：[in]state  函数接口返回的状态
//~english
/**
 * @brief Get the description of the function's return status
 * 
 * @param state[in]The status returned by the function interface 
 * @return LX_API_STR 
 */
LX_API_STR DcGetErrorString(LX_STATE state);


//~chinese:
//功能: 算法库使用GPU加速，打开相机前必须先调用，否则无效。默认关闭。部分GPU设备可能不支持，如果出现异常，建议关闭。
//参数：[in]is_enable     是否使能
//~english:
/**
 * @brief Use gpu to accelerate, must call first. Recommend to set false if it works abnormally.
 *
 * @param is_enable [bool]
 * @return LX_API
 */
LX_API DcSetGpuEnable(bool is_enable);
LX_API DcGetGpuEnable(bool* is_enable);

#endif
