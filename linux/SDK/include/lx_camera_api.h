
#ifndef _LX_CAMERA_API_H_
#define _LX_CAMERA_API_H_

#include "lx_camera_define.h"

/**
 * @brief Get api version
 * 
 * @return LX_API_STR 
 */
LX_API_STR DcGetApiVersion();

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
LX_API DcSetInfoOutput(int print_level, bool enable_screen_print, const char* log_path);
#endif

/**
 * @brief Allows users to output debugging information to log files.
 * 
 * @param str [in]String to be output, ending with '\0'.
 * @return LX_API 
 */
LX_API DcLog(const char* str);

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
LX_API DcGetDeviceList(LxDeviceInfo** devlist, int* devnum);
#endif

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

/**
 * @brief Close device
 * 
 * @param handle[in]Device handle 
 * @return LX_API 
 */
LX_API DcCloseDevice(DcHandle handle);

/**
 * @brief Start stream data
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcStartStream(DcHandle handle);

/**
 * @brief stop stream data
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcStopStream(DcHandle handle);

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
LX_API DcSetCameraIp(DcHandle handle, const char* ip, const char* netmask, const char* gateway);
#endif

/**
 * @brief Set int type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in]Setting parameter values 
 * @return LX_API 
 */
LX_API DcSetIntValue(DcHandle handle, int cmd, int value);

/**
 * @brief Obtain int type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter structure 
 * @return LX_API 
 */
LX_API DcGetIntValue(DcHandle handle, int cmd, LxIntValueInfo* value);

/**
 * @brief Set float type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in]Setting parameter values 
 * @return LX_API 
 */
LX_API DcSetFloatValue(DcHandle handle, int cmd, float value);

/**
 * @brief Obtain float type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter structure 
 * @return LX_API 
 */
LX_API DcGetFloatValue(DcHandle handle, int cmd, LxFloatValueInfo* value);

/**
 * @brief Set bool type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in]Setting parameter values 
 * @return LX_API 
 */
LX_API DcSetBoolValue(DcHandle handle, int cmd, bool value);

/**
 * @brief Obtain bool type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return Parameters 
 * @return LX_API 
 */
LX_API DcGetBoolValue(DcHandle handle, int cmd, bool* value);

/**
 * @brief Set string type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[in] 
 * @return LX_API 
 */
LX_API DcSetStringValue(DcHandle handle, int cmd, const char* value);

/**
 * @brief Obtain string type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter, no external memory allocation required 
 * @return LX_API 
 */
LX_API DcGetStringValue(DcHandle handle, int cmd, char** value);

/**
 * @brief Obtain pointer type parameters
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @param value[out]Return parameter, no external memory allocation required 
 * @return LX_API 
 */
LX_API DcGetPtrValue(DcHandle handle, int cmd, void** value);

/**
 * @brief Execute the corresponding CMD type command
 * 
 * @param handle[in]Device handle  
 * @param cmd[in]Reference LX_CAMERA_FEATURE 
 * @return LX_API 
 */
LX_API DcSetCmd(DcHandle handle, int cmd);

//                                 SetObstacleIndex,GetObstacleIndex,SetObstacleMode,GetObstacleIO,SetOdomData,SetRelocPose,SetLaserData,ImportLocationMapFile,EnableBuildMap,EnableLocation,ExportLocationMapFile
// 
/**
 * @brief Special operations outside the definition of LX_CAMERA_FEATURE, with specific functions and parameters defined by the string command
 * 
 * @param handle[in]Device handle  
 * @param command[in]Command      
 * @param value[in][out]Input parameter when setting, output parameter when getting, no need to allocate memory externally. 
 * @return LX_API 
 */
LX_API DcSpecialControl(DcHandle handle, const char* command, void* value);


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
LX_API DcRegisterFrameCallback(DcHandle handle, LX_FRAME_CALLBACK func, void* usr_data);
#endif

/**
 * @brief Cancel data frame registration callback function
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcUnregisterFrameCallback(DcHandle handle);

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
LX_API DcRegisterCameraStatusCallback(DcHandle handle, LX_CAMERA_STATUS_CALLBACK func, void* usr_data);
#endif

/**
 * @brief Cancel camera state registration callback function
 * 
 * @param handle[in]Device handle  
 * @return LX_API 
 */
LX_API DcUnregisterCameraStatusCallback(DcHandle handle);

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

/**
 * @brief Save the point cloud, which can be called directly
 * 
 * @param handle[in]Device handle  
 * @param filename[in]File name, supports txt, ply and pcd formats. txt format saves all data in image order, ply and pcd only save non-zero data 
 * @return LX_API 
 */
LX_API DcSaveXYZ(DcHandle handle, const char* filename);

/**
 * @brief Setting the parameter path
 * 
 * @param handle[in]Device handle  
 * @param filepath[in]Parameter file path, without file name (most cases do not need to call, only part of the camera type machine to open the parameters needed, such as M3, I2, etc.) 
 * @return LX_API 
 */
LX_API DcSetParamPath(DcHandle handle, const char* filepath);

/**
 * @brief Get the description of the function's return status
 * 
 * @param state[in]The status returned by the function interface 
 * @return LX_API_STR 
 */
LX_API_STR DcGetErrorString(LX_STATE state);


/**
 * @brief Use gpu to accelerate, must call first. Recommend to set false if it works abnormally.
 *
 * @param is_enable [bool]
 * @return LX_API
 */
LX_API DcSetGpuEnable(bool is_enable);
LX_API DcGetGpuEnable(bool* is_enable);

/**
 * @brief Enable the PTP time synchronization source service and call it before turning on the camera, otherwise it will be invalid. It is turned on by default

 *
 * @param is_enable [bool]
 * @return LX_API
 */
LX_API DcSetPtpEnable(bool is_enable);
LX_API DcGetPtpEnable(bool* is_enable);


/**
 */
/**
 * @brief Multi-threaded parallel computing accelerates the number of threads
 *
 * @param {int} thread_num: Set the number of threads to be computed in parallel, and the default -1 will automatically adjust
 *        Enabling parallel computing will increase CPU consumption, but the computing speed will be greatly improved
 * @return LX_API
 */
LX_API DcSetParallelThread(int thread_num);

/**
 * @brief Enable SDK inner log 

 *
 * @param is_enable [bool]
 * @return LX_API
 */
LX_API DcSetLogEnable(bool is_enable);



/**
 * @brief Register the imu data callback function to be called automatically when new data is received.
 *
 * @param handle[in]Device handle
 * @param func[in]Callback function pointer
 * @param usr_data[in]User-defined parameters
 * @return LX_API
 */
LX_API DcRegisterImuDataCallback(DcHandle handle, LX_IMUDATA_CALLBACK func, void* usr_data);

/**
    * @brief Cancel imu data  registration callback function
    *
    * @param handle[in]Device handle
    * @return LX_API
    */
LX_API DcUnregisterImuDataCallback(DcHandle handle);

/**
    * @brief Start IMU data listening
    *
    * @param handle[in]Device handle
    * @param acc_range[in]acc range 0:±2g，1:±4g，2:±8g，3:±16g
    * @param gry_range[in]gry range 0: 2000*PI/180 rad/s, 1: 1000*PI/180 rad/s, 2: 500*PI/180 rad/s, 3:250*PI/180 rad/s, 4:125*PI/180 rad/s
    * @return LX_API
    */
LX_API DcStartIMU(DcHandle handle, uint16_t acc_range, uint16_t gry_range);

/**
* @brief Start IMU data listening
*
* @param handle[in]Device handle
* @return LX_API
*/
LX_API DcStopImu(DcHandle handle);


#endif
