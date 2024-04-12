#ifndef _LX_CAMERA_DEFINE_H_
#define _LX_CAMERA_DEFINE_H_

#ifdef WIN32
    #ifndef _WIN32
        #define _WIN32
    #endif
#endif

#ifndef LX_EXPORT
    #ifdef _WIN32
        #ifdef LX_DC_EXPORTS
            #define LX_EXPORT __declspec(dllexport)
        #else
            #define LX_EXPORT __declspec(dllimport)
        #endif

        #define LX_STDC __stdcall

    #else
        #define LX_EXPORT

        #if defined(__i386__)
            #define LX_STDC __attribute__((stdcall))
        #else
            #define LX_STDC
        #endif
    #endif
#endif

#ifndef LX_EXTC
    #if defined(__cplusplus)
        #define LX_EXTC extern "C"
    #else
        #define LX_EXTC
    #endif
#endif

//错误类型定义
typedef enum LX_STATE
{
    LX_SUCCESS = 0,                 //成功
    LX_ERROR = -1,                  //未知错误
    LX_E_NOT_SUPPORT = -2,          //功能不支持
    LX_E_NETWORK_ERROR = -3,        //网络通讯错误
    LX_E_INPUT_ILLEGAL = -4,        //入参非法
    LX_E_RECONNECTING = -5,         //设备重连中
    LX_E_DEVICE_ERROR = -6,         //设备故障或设备响应失败
    LX_E_DEVICE_NEED_UPDATE = -7,   //设备版本过低，需升级
    LX_E_API_NEED_UPDATE = -8,      //API版本过低
    LX_E_CTRL_PERMISS_ERROR = -9,   //独占控制权限失败
    LX_E_GET_DEVICEINFO_ERROR = -10,//获取设备信息失败
    LX_E_IMAGE_SIZE_ERROR = -11,     //图像尺寸不匹配，需重新打开
    LX_E_IMAGE_PARTITION_ERROR = -12, //图像解析失败(pixformat不正确)，需尝试重新打开
    LX_E_DEVICE_NOT_CONNECTED = -13, //相机未连接
    LX_E_DEVICE_INIT_FAILED = -14,   //相机初始化失败
    LX_E_DEVICE_NOT_FOUND = -15,     //未找到相机（或未找到匹配的相机）
    LX_E_FILE_INVALID = -16,         //文件错误（文件名或类型或格式不正确或文件打开失败）
    LX_E_CRC_CHECK_FAILED = -17,     //文件crc或md5校验失败
    LX_E_TIME_OUT = -18,             //超时
    LX_E_FRAME_LOSS = -19,           //漏帧
    LX_E_ENABLE_ANYSTREAM_FAILED = -20, //开启任意流失败
    LX_E_NOT_RECEIVE_STREAM = -21,      //未收到流数据
    LX_E_PARSE_STREAM_FAILED = -22,     //启流成功但解析流数据失败
    LX_E_PROCESS_IMAGE_FAILED = -23,    //图像计算处理失败
    LX_E_SETTING_NOT_ALLOWED = -24,     //常开模式下不允许设置
    LX_E_LOAD_DATAPROCESSLIB_ERROR = -25, //加载图像处理算法库错误
    LX_E_FUNCTION_CALL_LOGIC_ERROR = -26, //函数调用逻辑错误
    LX_E_IPAPPDR_UNREACHABLE_ERROR = -27, //IP不可达或网络配置错误
    LX_E_FRAME_ID_NOT_MATCH        = -28, //超时范围内帧不同步错误
    LX_E_FRAME_MULTI_MACHINE       = -29, //帧中检测到多机干扰信号
}LX_STATE;

#define LX_API LX_EXTC LX_EXPORT LX_STATE LX_STDC
#define LX_API_STR LX_EXTC LX_EXPORT const char* LX_STDC

typedef unsigned long long DcHandle;

//设备型号
typedef enum LX_DEVICE_TYPE {
    LX_DEVICE_M2 = 1001,
    LX_DEVICE_M3,
    LX_DEVICE_M4Pro,
    LX_DEVICE_M4_MEGA,
    LX_DEVICE_M4,
    LX_DEVICE_S1 = 2001,
    LX_DEVICE_S2,
    LX_DEVICE_S2MaxV1,
    LX_DEVICE_S2MaxV2,
    LX_DEVICE_I1 = 3001,
    LX_DEVICE_I2,
    LX_DEVICE_T1 = 4001,
    LX_DEVICE_T2,
    LX_DEVICE_H3 = 5001,
    LX_DEVICE_V1Pro = 6001,

    LX_DEVICE_NULL = 0
}LX_DEVICE_TYPE;

//设备大类，暂未使用
typedef enum LX_DEVICE_SERIALS {
    LX_SERIAL_GIGE = 1,
    LX_SERIAL_WK,
    LX_SERIAL_OTHER,
    LX_SERIAL_ALL,
    LX_SERIAL_LOCAL_LOOP, //本地回环(用于区分SDK与相机端程序运行在同一主机上的情况)
}LX_DEVICE_SERIALS;

//设备详细信息
typedef struct LxDeviceInfo
{
    DcHandle handle;               //设备唯一标识
    LX_DEVICE_TYPE dev_type;       //设备类型
    char id[32];                   //设备id
    char ip[32];                   //设备ip:port
    char sn[32];                   //设备序列号
    char mac[32];                  //设备mac地址
    char firmware_ver[32];         //设备软件版本号
    char algor_ver[32];            //设备算法版本号
    char name[32];                 //设备名称，如：camera_M3_192.168.11.13_9803
    char reserve[32];              //预留字段, 子网掩码 //add at 20231120
    char reserve2[32];             //预留字段2, 网关ip  //add at 20231120
    char reserve3[64];             //预留字段3
    char reserve4[128];            //预留字段4
}LxDeviceInfo;

//整形参数结构体
typedef struct LxIntValueInfo
{
    bool set_available;   //当前值是否可设置, true-可设置，false-不可设置
    int cur_value;        //当前值
    int max_value;        //最大值
    int min_value;        //最小值
    int reserve[4];       //预留字段
}LxIntValueInfo;

//浮点参数结构体
typedef struct LxFloatValueInfo
{
    bool set_available;     //当前值是否可设置, true-可设置，false-不可设置
    float cur_value;        //当前值
    float max_value;        //最大值
    float min_value;        //最小值
    float reserve[4];       //预留字段
}LxFloatValueInfo;

//相机打开方式
typedef enum LX_OPEN_MODE
{
    OPEN_BY_INDEX = 0,     //按搜索列表中索引下标方式打开，对应的参数为索引号，当搜索到的设备列表发生变化时，选择打开的设备也会不一样
    OPEN_BY_IP = 1,        //按搜索列表中对应ip方式打开，对应的参数为设备ip或ip:port
    OPEN_BY_SN = 2,        //按搜索列表中对应sn方式打开，对应的参数为设备sn
    OPEN_BY_ID = 3,        //按搜索列表中对应id方式打开，对应的参数为设备id
}LX_OPEN_MODE;

//数据格式。3D强度图和深度图有不同的数据格式，获取数据指针需要使用对应的feature
typedef enum LX_DATA_TYPE
{
    LX_DATA_UNSIGNED_CHAR = 0,
    LX_DATA_UNSIGNED_SHORT = 2,
    LX_DATA_SIGNED_SHORT = 3,
    LX_DATA_FLOAT = 5,
    LX_DATA_OBSTACLE = 16,
    LX_DATA_PALLET = 17,
    LX_DATA_LOCATION =18,
    LX_DATA_OBSTACLE2 = 19,
}LX_DATA_TYPE;

//2D数据流支持不同的压缩格式
typedef enum LX_STREAM_ENCODE_MODE
{
    LX_STREAM_RAW = 0,
    LX_STREAM_H264 = 1,
    LX_STREAM_H265 = 2,
    LX_STREAM_MJPEG = 3,
    LX_STREAM_JPEG = 4,

}LX_STREAM_ENCODE_MODE;

//图像binning模式
typedef enum LX_BINNING_MODE
{
    LX_BINNING_1X1 = 0,
    LX_BINNING_2X2 = 1,
    LX_BINNING_4X4 = 2
}LX_BINNING_MODE;

//结构光相机编码方式
typedef enum LX_STRUCT_LIGHT_CODE_MODE
{
    LX_CODE_NORMAL = 1,//常规
    LX_CODE_STATBLE = 2,//稳定
    LX_CODE_ENHANCE = 3, //高精度加强
}LX_STRUCT_LIGHT_CODE_MODE;

//相机内置算法
typedef enum LX_ALGORITHM_MODE 
{
    MODE_ALL_OFF = 0,                 //关闭内置避障算法
    MODE_AVOID_OBSTACLE = 1,          //内置避障算法
    MODE_PALLET_LOCATE = 2,           //内置托盘对接算法
    MODE_VISION_LOCATION = 3,         //内置视觉定位算法
    MODE_AVOID_OBSTACLE2 = 4,         //内置避障算法V2
}LX_ALGORITHM_MODE;

//相机工作模式
typedef enum LX_CAMERA_WORK_MODE 
{
    KEEP_HEARTBEAT = 0,             //保持心跳，SDK心跳中断后相机自动待机
    WORK_FOREVER = 1,               //始终保持工作，此时部分参数无法设置
}LX_CAMERA_WORK_MODE;

//相机触发模式
typedef enum LX_TRIGGER_MODE 
{
    LX_TRIGGER_MODE_OFF,          //关闭触发模式,流模式,默认
    LX_TRIGGER_SOFTWARE,          //软触发模式
    LX_TRIGGER_HARDWARE,          //硬触发模式
}LX_TRIGGER_MODE;

//IO工作模式
typedef enum LX_IO_WORK_MODE {
    ALGORITHM_IO_MODE = 0,  //IO作为内置应用算法的输入输出
    USER_IO_MODE = 1,       //用户可以自己控制IO状态，参考LX_IO_OUT_USERCTRL_MODE
    TRIGGER_IO_MODE = 2,    //IO作为触发信号输出，仅触发模式下有效
}LX_IO_WORK_MODE;

//自定义IO输出状态
typedef enum LX_IO_OUTPUT_STATE {
    OUT1_0_OUT2_0 = 0,
    OUT1_0_OUT2_1 = 1,
    OUT1_1_OUT2_0 = 2,
    OUT1_1_OUT2_1 = 3,
}LX_IO_OUTPUT_STATE;

//滤波设置模式
typedef enum {
    FILTER_SIMPLE = 1,
    FILTER_NORMAL = 2,
    FILTER_EXPERT = 3,
}LX_FILTER_MODE;


//图像显示相关信息
typedef struct FrameDataInfo
{
    LX_DATA_TYPE frame_data_type;
    int frame_width;
    int frame_height;
    int frame_channel;
    void* frame_data;
    unsigned long long sensor_timestamp;     //sensor出图时间戳
    unsigned long long recv_timestamp;       //接收完帧数据时的时间戳
}FrameDataInfo;


//数据帧具体信息
typedef struct FrameInfo {
    LX_STATE frame_state;
    DcHandle handle;

    FrameDataInfo depth_data;   //深度图 
    FrameDataInfo amp_data;     //强度图
    FrameDataInfo rgb_data;     //rgb图
    FrameDataInfo app_data;     //算法输出结果
    void* reserve_data;         //扩展预留字段
}FrameInfo;

//数据帧扩展信息结构体
typedef struct FrameExtendInfo {
    unsigned int depth_frame_id;
    unsigned int amp_frame_id;
    unsigned int rgb_frame_id;
    unsigned int app_frame_id;
    void* reserve_data1;           //预留字段
    void* reserve_data2;           //预留字段
    void* reserve_data3;           //预留字段
}FrameExtendInfo;

//相机当前状态
typedef enum LX_CAMERA_STATUS
{
    STATUS_CLOSED,              //相机未打开或关闭状态
    STATUS_OPENED_UNSTARTED,    //相机已打开但未启流状态
    STATUS_STARTED,             //相机打开且已启流状态
    STATUS_CONNECTING,          //相机断线正在重连状态
    STATUS_CONNECT_SUCCESS      //相机重连成功
}LX_CAMERA_STATUS;

//相机状态回调信息
typedef struct CameraStatus {
    LX_CAMERA_STATUS camera_state;
    unsigned long long status_time;
    void* reserve_data;           //扩展预留字段
}CameraStatus;

//数据帧回调
typedef void(*LX_FRAME_CALLBACK)(FrameInfo*, void*);

//相机状态回调
typedef void(*LX_CAMERA_STATUS_CALLBACK)(CameraStatus*, void*);

typedef enum LX_CAMERA_FEATURE
{
    /*int feature*/
    LX_INT_FIRST_EXPOSURE = 1001,   //默认高积分曝光值，单位us, 针对多积分情况为第一个积分的曝光时间
    LX_INT_SECOND_EXPOSURE = 1002,  //默认低积分曝光值，单位us, 针对多积分情况为第二个积分的曝光时间
    LX_INT_THIRD_EXPOSURE = 1003,   //针对多积分情况为第三个积分的曝光时间，单位us
    LX_INT_FOURTH_EXPOSURE = 1004,  //针对多积分情况为第四个积分的曝光时间，单位us
    LX_INT_GAIN = 1005,             //增益，与曝光效果等价。会引入噪声，可适当调节增益防止曝光参数过大

    LX_INT_MIN_DEPTH = 1011,        //最小深度值
    LX_INT_MAX_DEPTH = 1012,        //最大深度值
    LX_INT_MIN_AMPLITUDE = 1013,    //有效信号最小强度值
    LX_INT_MAX_AMPLITUDE = 1014,    //有效信号最大强度值
    LX_INT_CODE_MODE = 1016,        //结构光相机编码模式，参考LX_STRUCT_LIGHT_CODE_MODE
    LX_INT_WORK_MODE = 1018,        //工作模式，参考LX_CAMERA_WORK_MODE
    LX_INT_LINK_SPEED = 1019,       //协商的网卡网速 100-百兆，1000-千兆, 只支持获取，不可设置

    //3D图像参数
    LX_INT_3D_IMAGE_WIDTH = 1021,       //3D图像分辨率宽度,(ROI/Binning/2D对齐后会变化,需重新获取)
    LX_INT_3D_IMAGE_HEIGHT = 1022,      //3D图像分辨率高度,(ROI/Binning/2D对齐后会变化,需重新获取)
    LX_INT_3D_IMAGE_OFFSET_X = 1023,    //ROI水平偏移像素，ROI设置中WIDTH HEIGHT OFFSET_X OFFSET_Y一起设置, 设置参数请用DcSetROI
    LX_INT_3D_IMAGE_OFFSET_Y = 1024,    //ROI垂直偏移像素，ROI设置中WIDTH HEIGHT OFFSET_X OFFSET_Y一起设置, 设置参数请用DcSetROI
    LX_INT_3D_BINNING_MODE = 1025,      //3D图像binning，参考LX_BINNING_MODE
    LX_INT_3D_DEPTH_DATA_TYPE = 1026,   //深度图像数据格式，只能获取，对应的值参考LX_DATA_TYPE

    //3D强度图像，尺寸与3D深度图一致
    LX_INT_3D_AMPLITUDE_CHANNEL = 1031,  //3D强度图像通道数，与深度图通道共用,单色为1，彩色为3
    LX_INT_3D_AMPLITUDE_DATA_TYPE = 1035,//强度图像数据格式，只能获取，对应的值参考LX_DATA_TYPE
    LX_INT_3D_AUTO_EXPOSURE_LEVEL = 1036,//3D自动曝光开启时的曝光等级,期间不允许设置曝光值与增益
    LX_INT_3D_AUTO_EXPOSURE_MAX = 1037,  //3D自动曝光上限值
    LX_INT_3D_AUTO_EXPOSURE_MIN = 1038,  //3D自动曝光下限值

    //2D图像
    LX_INT_2D_IMAGE_WIDTH = 1041,       //2D图像分辨率宽度,(ROI或Binning后会变化)
    LX_INT_2D_IMAGE_HEIGHT = 1042,      //2D图像分辨率高度,(ROI或Binning后会变化)
    LX_INT_2D_IMAGE_OFFSET_X = 1043,    //2D图像ROI水平偏移像素，ROI设置中WIDTH HEIGHT OFFSET_X OFFSET_Y一起设置, 设置参数请用DcSetROI
    LX_INT_2D_IMAGE_OFFSET_Y = 1044,    //2D图像ROI垂直偏移像素，ROI设置中WIDTH HEIGHT OFFSET_X OFFSET_Y一起设置, 设置参数请用DcSetROI
    LX_INT_2D_BINNING_MODE = 1045,      //2D图像binning，参考LX_BINNING_MODE
    LX_INT_2D_IMAGE_CHANNEL = 1046,     //2D图像通道数，单色为1，彩色为3
    LX_INT_2D_IMAGE_DATA_TYPE = 1047,   //2D图像数据格式，只能获取,对应的值参考LX_DATA_TYPE

    LX_INT_2D_MANUAL_EXPOSURE = 1051,   //2D手动曝光时的曝光值
    LX_INT_2D_MANUAL_GAIN = 1052,       //2D手动曝光时的增益值
    LX_INT_2D_ENCODE_TYPE = 1053,       //2D图像压缩格式，对应的值参考LX_STREAM_ENCODE_MODE
    LX_INT_2D_AUTO_EXPOSURE_LEVEL = 1054,//2D图像自动曝光时曝光等级[0-100], 0-整体更暗， 100-整体更亮

    LX_INT_TOF_GLOBAL_OFFSET = 1061,    //TOF深度数据偏移
    LX_INT_3D_UNDISTORT_SCALE = 1062,   //3D图像反畸变系数
    LX_INT_2D_UNDISTORT_SCALE = 1520,   //2D图像反畸变系数
    LX_INT_ALGORITHM_MODE = 1065,       //设置内置应用算法, 部分型号支持,对应的值参考LX_ALGORITHM_MODE
                                        //算法上移时（LX_INT_CALCULATE_UP），不允许开启内置应用算法
    LX_INT_MODBUS_ADDR = 1066,          //modbus地址，部分型号支持MODBUS协议通过串口输出
    LX_INT_HEART_TIME = 1067,           //与设备间心跳时间,单位ms
    LX_INT_GVSP_PACKET_SIZE = 1068,     //GVSP单包数据分包大小, 单位字节
    LX_INT_CALCULATE_UP = 1070,         //允许tof或rgb算法上下移，节省上位机或相机算力，可能影响帧率和延时
                                        //0-0xFFFF，四个F代表意义：保留位，滤波上移，RGB上移，TOF上移
                                        //内置应用算法（LX_INT_ALGORITHM_MODE）开启时不允许上移
    LX_INT_CAN_BAUD_RATE = 1072,         //can的波特率值, 单位bps 
    LX_INT_SAVE_PARAMS_GROUP = 1075,    //将相机当前配置保存为指定的参数组
    LX_INT_LOAD_PARAMS_GROUP = 1076,     //一键加载指定索引的参数组

    LX_INT_TRIGGER_MODE = 1069,         //触发模式,对应的值参考LX_TRIGGER_MODE
    LX_INT_HARDWARE_TRIGGER_FILTER_TIME = 1085, //硬触发滤波时间, 单位us
    LX_INT_TRIGGER_MIN_PERIOD_TIME = 1086,      //触发最小时间间隔, 单位us
    LX_INT_TRIGGER_DELAY_TIME = 1087,           //触发延迟时间,单位us, 当值<=1000时表示立刻生效(预留功能,若大于1000时表示延时生效)
    LX_INT_TRIGGER_FRAME_COUNT = 1088,          //单次触发帧数
    LX_INT_IO_WORK_MODE = 1530,           //GPIO信号输出控制模式, 参考LX_IO_WORK_MODE
    LX_INT_IO_OUTPUT_STATE = 1531,       //GPIO信号输出的用户控制模式, 参考LX_IO_OUTPUT_STATE

    LX_INT_FILTER_MODE = 1090,          //滤波模式,参考LX_FILTER_MODE
    LX_INT_FILTER_SMOOTH_LEVEL = 1091,  //当LX_INT_FILTER_MODE为FILTER_NORMAL时,可设置滤波平滑等级，[0, 3]，值越大，滤波越强
    LX_INT_FILTER_NOISE_LEVEL = 1092,   //当LX_INT_FILTER_MODE为FILTER_NORMAL时,可设置滤波噪声等级，[0, 3]，值越大，滤波越强
    LX_INT_FILTER_TIME_LEVEL = 1093,    //当LX_INT_FILTER_MODE为FILTER_NORMAL时,可设置滤波时域等级，[0, 3]，值越大，滤波越强

    /*float feature*/
    LX_FLOAT_FILTER_LEVEL = 2001,      //当LX_INT_FILTER_MODE为FILTER_SIMPLE时,可设置滤波等级，[0, 1]，值越大，滤波越强，等于0表示关闭滤波，
    LX_FLOAT_EST_OUT_EXPOSURE = 2002,  //是否评估过曝数据，[0, 1]，为1则过曝数据无效
    LX_FLOAT_LIGHT_INTENSITY = 2003,   //光强度，[0, 1]，部分型号支持
    LX_FLOAT_3D_DEPTH_FPS = 2004,      //深度图当前帧率，只可获取
    LX_FLOAT_3D_AMPLITUDE_FPS = 2005,  //强度图当前帧率，只可获取
    LX_FLOAT_2D_IMAGE_FPS = 2006,      //RGB图当前帧率，只可获取
    LX_FLOAT_DEVICE_TEMPERATURE = 2007,//相机当前温度, 只可获取

    /*bool feature*/
    LX_BOOL_CONNECT_STATE = 3001,           //当前连接状态
    LX_BOOL_ENABLE_3D_DEPTH_STREAM = 3002,  //开启/关闭深度数据流(部分相机支持)
    LX_BOOL_ENABLE_3D_AMP_STREAM = 3003,    //开启/关闭强度数据流(部分相机支持)
    LX_BOOL_ENABLE_3D_AUTO_EXPOSURE = 3006, //3D自动曝光使能
    LX_BOOL_ENABLE_3D_UNDISTORT = 3007,     //3D反畸变使能
    LX_BOOL_ENABLE_ANTI_FLICKER = 3008,     //抗频闪使能，LED环境照明可能导致数据存在明显波纹，部分型号支持

    LX_BOOL_ENABLE_2D_STREAM = 3011,        //开启/关闭2D数据流
    LX_BOOL_ENABLE_2D_AUTO_EXPOSURE = 3012, //2D自动曝光使能  
    LX_BOOL_ENABLE_2D_UNDISTORT = 3015,     //2D图像反畸变使能
    LX_BOOL_ENABLE_2D_TO_DEPTH = 3016,      //2D3D图像对齐使能，深度图像分辨率会与2D图像保持一致            
    LX_BOOL_ENABLE_BACKGROUND_AMP = 3017,   //强度背景光使能
    LX_BOOL_ENABLE_MULTI_MACHINE = 3018,    //多机模式使能，TOF会有多机干扰，开启此模式可以自动检测并缓解多机干扰
    LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR = 3019,  //HDR（多曝光高动态范围模式）使能
    LX_BOOL_ENABLE_SYNC_FRAME = 3020,          //是否开启强制帧同步, 默认数据实时性优先，若需要RGBD同步, 需要开启该模式

    /*string feature*/
    LX_STRING_DEVICE_VERSION = 4001,        //设备版本号
    LX_STRING_DEVICE_LOG_NAME = 4002,       //日志文件名，用于获取设备日志
    LX_STRING_FIRMWARE_NAME = 4003,         //固件文件名，用于升级设备版本，部分型号需要重新打开相机
    LX_STRING_FILTER_PARAMS = 4004,         //滤波算法参数,json格式的字符串
    LX_STRING_ALGORITHM_PARAMS = 4005,      //内置算法参数，根据当前设置的LX_ALGORITHM_MODE，返回对应的json格式字符串
    LX_STRING_ALGORITHM_VERSION = 4006,     //内置算法版本号，根据当前设置的LX_ALGORITHM_MODE，返回对应的版本号
    LX_STRING_DEVICE_OS_VERSION = 4007,     //设备系统镜像版本号
    LX_STRING_IMPORT_PARAMS_FROM_FILE = 4008,   //从本地文件加载参数到相机
    LX_STRING_EXPORT_PARAMS_TO_FILE = 4009,      //将相机当前参数导出到本地文件

    /*command feature*/
    LX_CMD_GET_NEW_FRAME = 5001,    //主动更新当前最新数据，调用之后才可以获取相关数据指针。回调方式不需调用
    LX_CMD_RETURN_VERSION = 5002,   //回退上一版本
    LX_CMD_RESTART_DEVICE = 5003,   //重启相机，部分型号需要重新打开相机
    LX_CMD_WHITE_BALANCE = 5004,    //自动白平衡
    LX_CMD_RESET_PARAM = 5007,      //恢复默认参数

    /*ptr feature*/
    LX_PTR_2D_IMAGE_DATA = 6001, //获取2D图像数据指针，数据长度由2D图像尺寸、通道数和数据格式（LX_INT_2D_IMAGE_DATA_TYPE）确定
    LX_PTR_3D_AMP_DATA = 6002,   //获取3D强度图数据指针，数据长度由3D图像尺寸、通道数和数据格式（LX_INT_3D_AMPLITUDE_DATA_TYPE）确定
    LX_PTR_3D_DEPTH_DATA = 6003, //获取3D深度图数据指针，数据长度由3D图像尺寸、通道数和数据格式（LX_INT_3D_DEPTH_DATA_TYPE）确定
    LX_PTR_XYZ_DATA = 6004,      //获取点云数据指针，float*类型三通道(x, y, z为一组数据，依次循环)
                                 //数据长度为LX_INT_3D_IMAGE_WIDTH*LX_INT_3D_IMAGE_HEIGHT*sizeof(float)*3   
    LX_PTR_2D_INTRIC_PARAM = 6005,  //获取2D图像内参，float*类型指针，长度固定为9*sizeof(float)(fx,fy,cx,cy,k1,k2,p1,p2,k3)
    LX_PTR_3D_INTRIC_PARAM = 6006, //获取3D图像内参, float*类型指针，长度固定为9*sizeof(float)(fx,fy,cx,cy,k1,k2,p1,p2,k3)
    LX_PTR_3D_EXTRIC_PARAM = 6007, //获取3D图像外参，float*类型指针 ，长度固定为12*sizeof(float)(前9个表示旋转矩阵，后3个表示平移向量)
    LX_PTR_ALGORITHM_OUTPUT = 6008, //获取内置算法输出
                                    //当开启模式为MODE_AVOID_OBSTACLE，输出结果为LxAvoidanceOutput指针，参考struct LxAvoidanceOutput，
                                    //当开启模式为MODE_PALLET_LOCATE，输出结果为LxPalletPose指针，参考struct LxPalletPose,
                                    //当开启模式为MODE_VISION_LOCATION，输出结果为LxLocation指针，参考struct LxLocation
                                    //当开启模式为MODE_AVOID_OBSTACLE2，输出结果为LxAvoidanceOutputN指针，参考struct LxAvoidanceOutputN
    LX_PTR_FRAME_DATA = 6009,       //获取完整一帧数据，输出结果参考结构体FrameInfo
}LX_CAMERA_FEATURE;

#endif
