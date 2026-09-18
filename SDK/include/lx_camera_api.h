
#ifndef _LX_CAMERA_API_H_
#define _LX_CAMERA_API_H_

#include "lx_camera_define.h"

//~chinese:
//功能: 获取API版本号
//参数：[out]version  API版本号，无需外部分配内存
//~english:
/**
 * @brief Get the API version.
 *
 * @return LX_API_STR API
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
 * @brief Set the log output level and output options.
 *
 * @param print_level [in] Log output level:
 *                    0: info  - Output all debug messages.
 *                    1: warn  - Output important and warning messages.
 *                    2: error - Output error messages only.
 *                    3: no output.
 * @param enable_screen_print [in] Whether to print log messages to the console/window.
 * @param log_path [in] Path for saving log files. If empty, logs are saved in the current working directory.
 * @return LX_API
 */
#ifdef __cplusplus
LX_API DcSetInfoOutput(int print_level = 1, bool enable_screen_print = false, const char* log_path = "", int language = 0);
#else
//纯C风格接口，原接口逐步弃用
//C-style interface; the legacy interface is being phased out.
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
 * @brief Search for supported cameras.
 *
 * @param devlist [out] List of detected cameras.
 * @param devnum [out] Number of detected cameras.
 * @return LX_API
 */
#ifdef __cplusplus
LX_API DcGetDeviceList(LxDeviceInfo** devlist, int* devnum, LX_DEVICE_SERIALS lx_serials = LX_SERIAL_ALL);
#else
//纯C风格接口，原接口逐步弃用
//C-style interface; the legacy interface is being phased out.
LX_API DcGetDeviceList(LxDeviceInfo** devlist, int* devnum, LX_DEVICE_SERIALS lx_serials);
#endif

//~chinese:
//功能：打开设备。设备打开后会独占权限，其他进程无法再打开相机。如果程序强制结束没有调用DcCloseDevice，需要等待几秒等心跳超时释放权限
//参数：[in]open_mode           打开方式, 具体说明见LX_OPEN_MODE
//      [in]param               不同的打开方式，填写不同的参数(当sdk运行在相机内部时，此处填写"127.0.0.1")
//      [out]handle             连接成功后返回的设备句柄,后续所有接口访问都依赖该handle字段
//      [out]info               连接成功后返回的相机详细信息
//~english:
/**
 * @brief Open a device.
 *
 * After the device is opened, the current process has exclusive access and other processes cannot open the camera.
 * If the program terminates without calling DcCloseDevice, wait a few seconds for the heartbeat timeout to release the access permission.
 *
 * @param open_mode [in] Open mode. See LX_OPEN_MODE for details.
 * @param param [in] Parameter corresponding to the selected open mode. When the SDK runs on the camera itself, use "127.0.0.1".
 * @param handle [out] Device handle returned after a successful connection. All subsequent API calls for this device use this handle.
 * @param info [out] Detailed camera information returned after a successful connection.
 * @return LX_API
 */
LX_API DcOpenDevice(LX_OPEN_MODE open_mode, const char* param, DcHandle* handle, LxDeviceInfo* info);

//~chinese:
//功能：打开设备。设备打开后会独占权限，其他进程无法再打开相机。如果程序强制结束没有调用DcCloseDevice，需要等待几秒等心跳超时释放权限, 使用指定网卡IP进行绑定
//参数：[in]open_mode           打开方式, 具体说明见LX_OPEN_MODE
//      [in]param               不同的打开方式，填写不同的参数(当sdk运行在相机内部时，此处填写"127.0.0.1")
//      [in]eth_ip              指定网卡IP, 必须指定, 用于gvsp,imu数据指定网卡接收数据
//      [out]handle             连接成功后返回的设备句柄,后续所有接口访问都依赖该handle字段
//      [out]info               连接成功后返回的相机详细信息
//~english:
/**
 * @brief Open a device and bind data reception to a specified Ethernet interface IP address.
 *
 * After the device is opened, the current process has exclusive access and other processes cannot open the camera.
 * If the program terminates without calling DcCloseDevice, wait a few seconds for the heartbeat timeout to release the access permission.
 *
 * @param open_mode [in] Open mode. See LX_OPEN_MODE for details.
 * @param param [in] Parameter corresponding to the selected open mode. When the SDK runs on the camera itself, use "127.0.0.1".
 * @param eth_ip [in] IP address of the specified Ethernet interface. This parameter is required and is used to receive GVSP and IMU data through that interface.
 * @param handle [out] Device handle returned after a successful connection. All subsequent API calls for this device use this handle.
 * @param info [out] Detailed camera information returned after a successful connection.
 * @return LX_API
 */
LX_API DcOpenDeviceFixedEth(LX_OPEN_MODE open_mode, const char* param, const char* eth_ip, DcHandle* handle, LxDeviceInfo* info);

//~chinese
//功能: 关闭设备。与DcOpenDevice对应，释放权限和资源。
//参数：[in]handle  设备句柄
//~english
/**
 * @brief Close a device and release its access permission and resources.
 *
 * This function corresponds to DcOpenDevice.
 *
 * @param handle [in] Device handle.
 * @return LX_API
 */
LX_API DcCloseDevice(DcHandle handle);

//~chinese
//功能: 打开数据流
//参数：[in]handle  设备句柄
//~english
/**
 * @brief Start the data stream.
 *
 * @param handle [in] Device handle.
 * @return LX_API
 */
LX_API DcStartStream(DcHandle handle);

//~chinese
//功能: 关闭数据流
//参数：[in]handle  设备句柄
//~english
/**
 * @brief Stop the data stream.
 *
 * @param handle [in] Device handle.
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
 * @brief Set the camera IP configuration.
 *
 * After the IP configuration is changed, the device list also changes. Call DcGetDeviceList again to obtain the updated device list.
 *
 * @param handle [in] Device handle. If the device is not connected, obtain the handle by searching for devices first.
 * @param ip [in] Device IP address.
 * @param netmask [in] Subnet mask. If null, the default value "255.255.0.0" is used internally.
 * @param gateway [in] Gateway IP address. If null, the last segment of the device IP address is replaced with "1" and used as the gateway.
 * @return LX_API
 */
#ifdef __cplusplus
LX_API DcSetCameraIp(DcHandle handle, const char* ip, const char* netmask = 0, const char* gateway = 0);
#else
//纯C风格接口，原接口逐步弃用
// C-style interface; the legacy interface is being phased out.
LX_API DcSetCameraIp(DcHandle handle, const char* ip, const char* netmask, const char* gateway);
#endif

//~chinese
//功能：设置int类型参数
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//      [in]value                   设置参数值
//~english
/**
 * @brief Set an integer parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [in] Parameter value to set.
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
 * @brief Get an integer parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [out] Structure containing the returned parameter information.
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
 * @brief Set a floating-point parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [in] Parameter value to set.
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
 * @brief Get a floating-point parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [out] Structure containing the returned parameter information.
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
 * @brief Set a Boolean parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [in] Parameter value to set.
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
 * @brief Get a Boolean parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [out] Returned parameter value.
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
 * @brief Set a string parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [in] Parameter value to set.
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
 * @brief Get a string parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [out] Returned parameter value. No external memory allocation is required.
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
 * @brief Get a pointer-type parameter.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @param value [out] Returned parameter value. No external memory allocation is required.
 * @return LX_API
 */
LX_API DcGetPtrValue(DcHandle handle, int cmd, void** value);

//~chinese
//功能：执行对应CMD类型指令操作
//参数：[in]handle                  设备句柄
//      [in]cmd                     参考LX_CAMERA_FEATURE
//~english
/**
 * @brief Execute the command corresponding to the specified CMD-type feature.
 *
 * @param handle [in] Device handle.
 * @param cmd [in] Feature identifier. See LX_CAMERA_FEATURE.
 * @return LX_API
 */
LX_API DcSetCmd(DcHandle handle, int cmd);

//~chinese
//功能：LX_CAMERA_FEATURE定义之外的特殊操作，具体功能和参数由字符串command确定
//参数：[in]handle                 设备句柄
//      [in]command                指令(具体详见demo)
//                                 SetObstacleIndex,GetObstacleIndex,SetObstacleMode,GetObstacleIO,SetOdomData,SetRelocPose,SetLaserData,ImportLocationMapFile,EnableBuildMap,EnableLocation,ExportLocationMapFile
// 
//      [in][out]value             设置时为对应入参，获取时为对应出参，无需外部分配内存
//~english
/**
 * @brief Perform a special operation not defined in LX_CAMERA_FEATURE. The operation and its parameters are determined by the command string.
 *
 * @param handle [in] Device handle.
 * @param command [in] Command string. See the demo for details. Supported commands include:
 *                     SetObstacleIndex, GetObstacleIndex, SetObstacleMode, GetObstacleIO, SetOdomData,
 *                     SetRelocPose, SetLaserData, ImportLocationMapFile, EnableBuildMap, EnableLocation,
 *                     and ExportLocationMapFile.
 * @param value [in,out] Input parameter for a set operation or output parameter for a get operation. No external memory allocation is required.
 * @return LX_API
 */
LX_API DcSpecialControl(DcHandle handle, const char* command, void* value);

//~chinese
//功能：LX_CAMERA_FEATURE定义之外的特殊操作，具体功能和参数由字符串command确定
//参数：[in]handle                 设备句柄
//      [in]command                指令(具体详见demo)
//                                 SetObstacleIndex,GetObstacleIndex,SetObstacleMode,GetObstacleIO,SetOdomData,SetRelocPose,SetLaserData,ImportLocationMapFile,EnableBuildMap,EnableLocation,ExportLocationMapFile
// 
//      [in][out]value             设置时为对应入参，获取时为对应出参，无需外部分配内存
//      [in]timeout_ms             超时时间
//~english
/**
 * @brief Perform a special operation not defined in LX_CAMERA_FEATURE, with an extended timeout. The operation and its parameters are determined by the command string.
 *
 * @param handle [in] Device handle.
 * @param command [in] Command string. See the demo for details. Supported commands include:
 *                     SetObstacleIndex, GetObstacleIndex, SetObstacleMode, GetObstacleIO, SetOdomData,
 *                     SetRelocPose, SetLaserData, ImportLocationMapFile, EnableBuildMap, EnableLocation,
 *                     and ExportLocationMapFile.
 * @param value [in,out] Input parameter for a set operation or output parameter for a get operation. No external memory allocation is required.
 * @param timeout_ms [in] Timeout in milliseconds. If timeout_ms <= 300, the operation is attempted once; otherwise, it is retried up to timeout_ms / 300 times before timing out. If timeout_ms < 1, LX_INT_GVCP_TIME_OUT is used (default: 1800 ms).
 * @return LX_API
 */
LX_API DcSpecialControlExtend(DcHandle handle, const char* command, void* value, int timeout_ms);

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
 * @brief Set the ROI. If an input value is not a multiple of 8, it is automatically adjusted to the nearest multiple of 8.
 *
 * @param handle [in] Device handle.
 * @param offsetx [in] Horizontal offset of the ROI start point, in pixels.
 * @param offsety [in] Vertical offset of the ROI start point, in pixels.
 * @param width [in] Width of the ROI.
 * @param height [in] Height of the ROI.
 * @param img_type [in] Image type: 0 for 3D image; 1 for 2D image.
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
 * @brief Register a data-frame callback that is invoked automatically when new data is received.
 *
 * @param handle [in] Device handle.
 * @param func [in] Callback function pointer.
 * @param usr_data [in] User-defined data passed to the callback.
 * @return LX_API
 */
#ifdef __cplusplus
LX_API DcRegisterFrameCallback(DcHandle handle, LX_FRAME_CALLBACK func, void* usr_data = 0);
#else
//纯C风格接口，原接口逐步弃用
// C-style interface; the legacy interface is being phased out.
LX_API DcRegisterFrameCallback(DcHandle handle, LX_FRAME_CALLBACK func, void* usr_data);
#endif

//~chinese
//功能: 取消数据帧注册回调函数
//参数：[in]handle      设备句柄
//~english
/**
 * @brief Unregister the data-frame callback.
 *
 * @param handle [in] Device handle.
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
 * @brief Register a camera-status callback that is invoked automatically when the camera status changes.
 *
 * @param handle [in] Device handle.
 * @param func [in] Callback function pointer.
 * @param usr_data [in] User-defined data passed to the callback.
 * @return LX_API
 */
#ifdef __cplusplus
LX_API DcRegisterCameraStatusCallback(DcHandle handle, LX_CAMERA_STATUS_CALLBACK func, void* usr_data = 0);
#else
//纯C风格接口，原接口逐步弃用
//C-style interface; the legacy interface is being phased out.
LX_API DcRegisterCameraStatusCallback(DcHandle handle, LX_CAMERA_STATUS_CALLBACK func, void* usr_data);
#endif

//~chinese
//功能: 取消相机状态注册回调函数
//参数：[in]handle      设备句柄
//~english
/**
 * @brief Unregister the camera-status callback.
 *
 * @param handle [in] Device handle.
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
 * @brief Write user-defined data to the device.
 *
 * @param handle [in] Device handle.
 * @param start_address [in] Start address, beginning at 0.
 * @param data [in] Data to write. Up to 128 KB is supported.
 * @param data_size [in] Length of the data to write, in bytes. The maximum is 128 KB.
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
 * @brief Read user-defined data from the device.
 *
 * No external memory allocation is required. Before each read, the memory allocated for the previous read is reset.
 *
 * @param handle [in] Device handle.
 * @param start_address [in] Start address, beginning at 0.
 * @param data [out] Returned data. Up to 128 KB is supported.
 * @param data_size [out] Length of the returned data, in bytes. The maximum is 128 KB.
 * @return LX_API
 */
LX_API DcReadUserData(DcHandle handle, int start_address, char** data, int& data_size);

//~chinese
//功能: 保存点云，可直接调用。需要在采集数据线程中调用否则数据会有异常
//参数：[in]handle                设备句柄
//      [in]filename              文件名，支持txt,ply和pcd格式。txt格式按图像顺序保存所有数据，ply和pcd仅保存非零数据
//~english
/**
 * @brief Save the point cloud.
 *
 * This function can be called directly, but it must be called from the data-acquisition thread; otherwise, the saved data may be invalid.
 *
 * @param handle [in] Device handle.
 * @param filename [in] Output file name. Supported formats are TXT, PLY, and PCD. TXT saves all data in image order; PLY and PCD save only non-zero data.
 * @return LX_API
 */
LX_API DcSaveXYZ(DcHandle handle, const char* filename);

//~chinese
//功能：设置参数路径
//参数：[in]handle                  设备句柄
//      [in]filepath                参数文件路径，不包含文件名(大部分情况不需要调用，仅部分相型号机打开时需要参数，如M3，I2等)
//~english
/**
 * @brief Set the parameter-file directory.
 *
 * @param handle [in] Device handle.
 * @param filepath [in] Directory containing the parameter files, excluding the file name. This function is not required in most cases; it is needed only when opening certain camera models that require external parameters, such as M3 and I2.
 * @return LX_API
 */
LX_API DcSetParamPath(DcHandle handle, const char* filepath);

//~chinese
//功能: 获取函数返回状态对应的说明
//参数：[in]state  函数接口返回的状态
//~english
/**
 * @brief Get the description corresponding to a function return status.
 *
 * @param state [in] Status returned by an API function.
 * @return LX_API_STR Description of the specified status.
 */
LX_API_STR DcGetErrorString(LX_STATE state);


//~chinese:
//功能: 算法库使用GPU加速，打开相机前必须先调用，否则无效。默认关闭。部分GPU设备可能不支持，如果出现异常，建议关闭。
//参数：[in]is_enable     是否使能
//~english:
/**
 * @brief Enable or disable GPU acceleration for the algorithm library.
 *
 * This function must be called before opening the camera; otherwise, the setting does not take effect. GPU acceleration is disabled by default. Some GPU devices may not be supported; disable this option if abnormal behavior occurs.
 *
 * @param is_enable [in] Whether GPU acceleration is enabled.
 * @return LX_API
 */
LX_API DcSetGpuEnable(bool is_enable);
LX_API DcGetGpuEnable(bool* is_enable);

//~chinese:
//功能: 开启ptp时间同步源服务，打开相机前调用，否则无效。默认打开
//参数：[in]is_enable     是否使能
//~english:
/**
 * @brief Enable or disable the PTP time-synchronization source service.
 *
 * This function must be called before opening the camera; otherwise, the setting does not take effect. The service is enabled by default.
 *
 * @param is_enable [in] Whether the PTP time-synchronization source service is enabled.
 * @return LX_API
 */
LX_API DcSetPtpEnable(bool is_enable);
LX_API DcGetPtpEnable(bool* is_enable);


/**
 * 功能：多线程并行计算加速线程数量
 * 参数：{int} thread_num: 设置并行计算的线程数，默认-1自动调整
 * 开启并行计算会增加CPU消耗，但是计算速度有较大提升
 */
//~english:
/**
 * @brief Set the number of threads used for multithreaded parallel computation.
 *
 * Enabling parallel computation increases CPU usage but can significantly improve computation speed.
 *
 * @param thread_num [in] Number of threads used for parallel computation. The default value is -1, which enables automatic adjustment.
 * @return LX_API
 */
LX_API DcSetParallelThread(int thread_num);

/**
 * 功能：2d图像传输采用jpeg压缩，设置jpeg解码方式，
 * 参数：{int} mode: 参考JPEG_DECODE_METHOD
 */
//~english:
/**
 * @brief Set the JPEG decoding method for 2D image transmission, which uses JPEG compression.
 *
 * @param mode [in] JPEG decoding method. See JPEG_DECODE_METHOD.
 * @return LX_API
 */
LX_API DcSetJpegDecodeMethod(int mode);

/**
 * 功能：设置日志文件滚动存储最大数量。在DcSetInfoOutput之前调用
 */
//~english:
/**
 * @brief Set or get the maximum number of log files retained for rolling log storage. Should before calling DcSetInfoOutput.
 *
 * @param count [in,out] Maximum number of log files for DcSetMaxLogFile, or the returned maximum number for DcGetMaxLogFile.
 * @return LX_API
 */
LX_API DcSetMaxLogFile(int count);
LX_API DcGetMaxLogFile(int* count);

/**
 * 功能：设置单个日志文件最大容量，单位byte。在DcSetInfoOutput之前调用
 */
//~english:
/**
 * @brief Set or get the maximum size of a single log file, in bytes. Should before calling DcSetInfoOutput.
 *
 * @param size [in,out] Maximum log-file size for DcSetMaxLogFileSize, or the returned maximum size for DcGetMaxLogFileSize.
 * @return LX_API
 */
LX_API DcSetMaxLogFileSize(int size);
LX_API DcGetMaxLogFileSize(int* size);


//~chinese
//功能: 注册IMU传感器数据回调函数,收到新的数据时自动调用
//参数：[in]handle      设备句柄
//      [in]func        回调函数指针
//      [in]usr_data    用户自定义参数
//~english
/**
 * @brief Register an IMU data callback that is invoked automatically when new IMU data is received.
 *
 * @param handle [in] Device handle.
 * @param func [in] Callback function pointer.
 * @param usr_data [in] User-defined data passed to the callback.
 * @return LX_API
 */
LX_API DcRegisterImuDataCallback(DcHandle handle, LX_IMUDATA_CALLBACK func, void* usr_data);

//~chinese
//功能: 取消IMU传感器数据帧注册回调函数
//参数：[in]handle      设备句柄
//~english
/**
 * @brief Unregister the IMU data callback.
 *
 * @param handle [in] Device handle.
 * @return LX_API
 */
LX_API DcUnregisterImuDataCallback(DcHandle handle);

//~chinese
// 功能: 设置ptp时间来源回调
// 参数 [in]func,回调函数指针, func=NULL时为取消回调
// 仅可以设置一次有效回调, 多次无效,已第一次设置为准, 全局共享;
// 接口线程安全, 但回调函数内逻辑线程安全由设置方保证
//~english
/**
 * @brief Set the PTP time-source callback.
 *
 * Only one valid callback can be set. Subsequent attempts to set another callback have no effect; the first valid callback remains active and is shared globally.
 * The API itself is thread-safe, but the caller is responsible for ensuring thread safety within the callback implementation.
 *
 * @param func [in] Callback function pointer. Set func to NULL to unregister the callback.
 * @return LX_API
 */
LX_API DcSetPtpClockCallback(LX_PTP_CLOCK_CALLBACK func);

#endif
