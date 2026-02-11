using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Timers;
using System.Drawing;
using System.Windows.Input;
using System.IO;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;//调用C++dll必须引用的命名空间

namespace lx
{
    public unsafe class LxCamera
    {
        [DllImport("LxCameraApi.dll", EntryPoint = "DcGetApiVersion", ExactSpelling = false)]
        private static extern IntPtr _DcGetApiVersion();

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetInfoOutput", ExactSpelling = false)]
        public static extern LX_STATE DcSetInfoOutput(int print_level, bool enable_screen_print, string log_path);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcLog", ExactSpelling = false)]
        public static extern LX_STATE DcLog(string str);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcGetDeviceList", ExactSpelling = false)]
        public static extern LX_STATE DcGetDeviceList(out IntPtr devlist, ref int devnum);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcOpenDevice")]
        public static extern LX_STATE DcOpenDevice(LX_OPEN_MODE openMode, string param, ref long handle, IntPtr info);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcCloseDevice", ExactSpelling = false)]
        public static extern LX_STATE DcCloseDevice(long handle);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcStartStream", ExactSpelling = false)]
        public static extern LX_STATE DcStartStream(long handle);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcStopStream", ExactSpelling = false)]
        public static extern LX_STATE DcStopStream(long handle);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSaveXYZ", ExactSpelling = false)]
        public static extern LX_STATE DcSaveXYZ(long handle, IntPtr filename);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetCameraIp", ExactSpelling = false)]
        public static extern LX_STATE DcSetCameraIp(long handle, IntPtr ip, IntPtr netmask, IntPtr gateway);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetIntValue", ExactSpelling = false)]
        public static extern LX_STATE DcSetIntValue(long handle, int cmd, int value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcGetIntValue", ExactSpelling = false)]
        public static extern LX_STATE DcGetIntValue(long handle, int cmd, ref LxIntValueInfo value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetFloatValue", ExactSpelling = false)]
        public static extern LX_STATE DcSetFloatValue(long handle, int cmd, float value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcGetFloatValue", ExactSpelling = false)]
        public static extern LX_STATE DcGetFloatValue(long handle, int cmd, ref LxFloatValueInfo value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetBoolValue", ExactSpelling = false)]
        public static extern LX_STATE DcSetBoolValue(long handle, int cmd, bool value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcGetBoolValue", ExactSpelling = false)]
        public static extern LX_STATE DcGetBoolValue(long handle, int cmd, ref bool value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetStringValue", ExactSpelling = false)]
        public static extern LX_STATE DcSetStringValue(long handle, int cmd, string value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcGetStringValue", ExactSpelling = false)]
        public static extern LX_STATE DcGetStringValue(long handle, int cmd, out IntPtr value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcGetPtrValue", ExactSpelling = false)]
        public static extern LX_STATE DcGetPtrValue(long handle, int cmd, out IntPtr value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetCmd", ExactSpelling = false)]
        public static extern LX_STATE DcSetCmd(long handle, int cmd);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSpecialControl", ExactSpelling = false)]
        public static extern LX_STATE DcSpecialControl(long handle, string param, ref IntPtr value);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetROI", ExactSpelling = false)]
        public static extern LX_STATE DcSetROI(long handle, int offsetx, int offsety, int width, int height, int img_type);

        public enum LX_STATE
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
			LX_E_FRAME_IMAGE_ERROR = -30, //帧数据中检测到相机设备图像相关异常错误

			LX_W_NOT_SUPPORT               = 2, //功能不支持
			LX_W_LOAD_DATAPROCESSLIB_ERROR = 25, //加载图像处理算法库失败，不影响其他功能
        };
        public enum LX_DEVICE_TYPE
        {
            LX_DEVICE_M2 = 1001,
            LX_DEVICE_M3,
            LX_DEVICE_M4Pro,
            LX_DEVICE_M4_MEGA,
            LX_DEVICE_M4,
            LX_DEVICE_M4V1_1 = 1006,
            LX_DEVICE_M4ProV1_1 = 1007,

            LX_DEVICE_S1 = 2001,
            LX_DEVICE_S2,
            LX_DEVICE_S2MaxV1,
            LX_DEVICE_S2MaxV2,
            LX_DEVICE_S2MaxV1_1 = 2005,
            LX_DEVICE_S3 = 2101,

            LX_DEVICE_I1 = 3001,
            LX_DEVICE_I2,
            LX_DEVICE_T1 = 4001,
            LX_DEVICE_T2,
            LX_DEVICE_H3 = 5001,
            LX_DEVICE_V1Pro = 6001,

            LX_DEVICE_NULL = 0
        };
        public enum LX_BINNING_MODE
        {
            LX_BINNING_1X1 = 0,
            LX_BINNING_2X2 = 1,
            LX_BINNING_4X4 = 2
        };
        public enum LX_ALGORITHM_MODE
        {
            MODE_ALL_OFF = 0,                //关闭内置避障算法
            MODE_AVOID_OBSTACLE = 1,          //内置避障算法
            MODE_PALLET_LOCATE = 2,          //内置托盘对接算法
            MODE_VISION_LOCATION = 3,         //内置视觉定位算法
            MODE_AVOID_OBSTACLE2 = 4,         //内置避障算法V2
        };
        public enum LX_CAMERA_WORK_MODE
        {
            KEEP_HEARTBEAT = 0,             //正常模式，按需采图，默认为该状态
            WORK_FOREVER = 1,       		//始终采图
        };
        public enum LX_FILTER_MODE {
            FILTER_SIMPLE = 1,
            FILTER_NORMAL = 2,
            FILTER_EXPERT = 3,
        };
		//RGBD对齐模式
		public enum LX_RGBD_ALIGN_MODE {
			RGBD_ALIGN_OFF = 0,   //关闭rgbd对齐，rgb和depth分别以各自分别和坐标系
			DEPTH_TO_RGB = 1,     //depth对齐rgb，分辨率和坐标以rgb为准，rgb强制反畸变
			RGB_TO_DEPTH = 2,     //rgb对齐depth，分辨率和坐标以depth为准
			DEPTH_TO_RGB_SPARSE=3,//算力优化版DEPTH_TO_RGB，分辨率和坐标以rgb为准，但不改变有效点云数据量
		};

		//畸变模型
		public enum LX_DISTORTION_MODEL {
			LX_DISTORTION_UNDEFINE  =0, //无效畸变模型
			LX_DISTORTION_RADTAN_5  =5, //常规径向切向模型[k1,k2,p1,p2,k3]
			LX_DISTORTION_RADTAN_14 =14, //大畸变径向切向模型[k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,t1,t2]
			LX_DISTORTION_FISHEYE   =15, //鱼眼畸变模型[k1,k2,k3,k4]
			LX_DISTORTION_SCARAMUZZA=16
		};


        public enum LX_CAMERA_FEATURE
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
			LX_INT_3D_FREQ_MODE = 1027,         //3D频率模式,详见LX_3D_FREQ_MODE定义
			LX_INT_3D_UNDISTORT_SCALE = 1062,   //3D图像反畸变系数，[0,100]，值越大，保留视野越多，0为保持原内参，1为无黑边

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
			LX_INT_2D_IMAGE_CHANNEL = 1046,     //2D图像通道数，单色和jpeg为1，彩色为3，YUV422为2
			LX_INT_2D_IMAGE_DATA_TYPE = 1047,   //2D图像数据格式，只能获取,对应的值参考LX_DATA_TYPE
			LX_INT_2D_IMAGE_OUTPUT_FORMAT = 1048,//2D图像输出格式，参考LX_2D_OUTPUT_FORMAT
			LX_INT_2D_ENCODE_RATIO = 1049,      //默认2D图像以jpeg压缩，设置压缩比例。
												//[1-99]，值越小，压缩率越高，带宽越小，解码速度越快，但图像质量更差。StartStream后有效

			LX_INT_2D_UNDISTORT_SCALE = 1520,   //2D图像反畸变系数，[0,100]，值越大，保留视野越多，0为保持原内参，1为无黑边

			LX_INT_2D_MANUAL_EXPOSURE = 1051,   //2D手动曝光时的曝光值
			LX_INT_2D_MANUAL_GAIN = 1052,       //2D手动曝光时的增益值
			LX_INT_2D_AUTO_EXPOSURE_LEVEL = 1054,//2D图像自动曝光时曝光等级[0-100], 0-整体更暗， 100-整体更亮

			//其他接口协议
			LX_INT_MODBUS_ADDR = 1066,          //modbus地址，部分型号支持MODBUS协议通过串口输出
			LX_INT_CAN_BAUD_RATE = 1072,         //can的波特率值, 单位bps 
			LX_INT_CAN_NODE_ID = 1073,           //can地址 
			LX_INT_CAN_PROTOCOL_TYPE = 1074,     //can协议类型, 参考枚举LX_CAN_PROTOCOL_TYPE 

			//硬件触发和IO
			LX_INT_TRIGGER_MODE = 1069,         //触发模式,对应的值参考LX_TRIGGER_MODE
			LX_INT_HARDWARE_TRIGGER_FILTER_TIME = 1085, //硬触发滤波时间, 单位us
			LX_INT_TRIGGER_MIN_PERIOD_TIME = 1086,      //触发最小时间间隔, 单位us
			LX_INT_TRIGGER_DELAY_TIME = 1087,           //触发延迟时间,单位us
			LX_INT_TRIGGER_FRAME_COUNT = 1088,          //单次触发帧数
			LX_INT_IO_WORK_MODE = 1530,           //GPIO信号输出控制模式, 参考LX_IO_WORK_MODE
			LX_INT_IO_OUTPUT_STATE = 1531,       //GPIO信号输出的用户控制模式, 参考LX_IO_OUTPUT_STATE
			LX_INT_IO_HARDWARE_WORK_MODE = 1532, //GPIO信号输出的硬件工作模式，参考LX_IO_HARDWARE_MODE
			LX_INT_IO_INPUT_STATUS = 1533,       //IO输入状态,只读
			LX_INT_IO_OUTPUT_STATUS = 1534,      //IO输出状态,只读

			//滤波
			LX_INT_FILTER_MODE = 1090,          //滤波模式,参考LX_FILTER_MODE
			LX_INT_FILTER_SMOOTH_LEVEL = 1091,  //当LX_INT_FILTER_MODE为FILTER_NORMAL时,可设置滤波平滑等级，[0, 3]，值越大，滤波越强
			LX_INT_FILTER_NOISE_LEVEL = 1092,   //当LX_INT_FILTER_MODE为FILTER_NORMAL时,可设置滤波噪声等级，[0, 3]，值越大，滤波越强
			LX_INT_FILTER_TIME_LEVEL = 1093,    //当LX_INT_FILTER_MODE为FILTER_NORMAL时,可设置滤波时域等级，[0, 3]，值越大，滤波越强
			LX_INT_FILTER_FILL_LEVEL = 1095,    //当LX_INT_FILTER_MODE为FILTER_NORMAL时,可设置滤波填充等级，[0, 3]，值越大，滤波越强
			LX_INT_FILTER_DETECT_LOW_SIGNAL = 1094, //小信号测量，可能检测到信噪比较低的数据，包括远距离数据导致多周期问题

			//附加功能
			LX_INT_TOF_GLOBAL_OFFSET = 1061,    //3D深度数据偏移
			LX_INT_3D_GLARE_LEVEL = 1738,       //炫光等级，0关闭,等级1,2,3
			LX_INT_ALGORITHM_MODE = 1065,       //设置内置应用算法, 部分型号支持,对应的值参考LX_ALGORITHM_MODE
												//算法上移时（LX_INT_CALCULATE_UP），不允许开启内置应用算法
			LX_INT_CALCULATE_UP = 1070,         //允许2d或3d算法上下移，节省上位机或相机算力，可能影响帧率和延时
												//0-0xFFFF，四个F代表意义：保留位，滤波上移，2d上移，3d上移
												//内置应用算法（LX_INT_ALGORITHM_MODE）开启时不允许上移
			LX_INT_RGBD_ALIGN_MODE = 1077,      //rgbd对齐的方式，参考LX_RGBD_ALIGN_MODE，替代LX_BOOL_ENABLE_2D_TO_DEPTH

			LX_INT_3D_FPS = 1510,               //深度、强度的帧率
			LX_INT_POWER_MODE = 1755,           //相机功耗模式，详见LX_POWER_MODE_TYPE定义，部分型号支持
			LX_INT_XYZ_COORDINATE   = 1078,     //点云坐标系，0：相机坐标系，xyz为右下前。1：机器人坐标系，xyz为前左上.
			//仅影响LX_PTR_XYZ_DATA和LX_PTR_XYZIRT_DATA的数据，如果有额外的外参转换需谨慎使用
			LX_INT_XYZ_UNIT         = 1079,     //点云单位，0：mm，1：m。仅影响LX_PTR_XYZ_DATA和LX_PTR_XYZIRT_DATA的数据
			LX_INT_IMU_ACCELERATION_LEVEL   = 1101, //IMU加速度量程等级，[0-3]，值越大量程越大，每一级变化一倍
			LX_INT_IMU_ANGULAR_RANGE_LEVEL  = 1102, //IMU角速度量程等级，[0-4]，值越大量程越大，每一级变化一倍
			LX_INT_MAX_TOLERANT_LOST_PACKETS = 1103, //一帧数据中允许丢包的数量，超出阈值则丢弃该帧，否则丢的包用0填充，默认为5
			LX_INT_HEART_TIME = 1067,           //与设备间心跳时间,单位ms
			LX_INT_GVSP_PACKET_SIZE = 1068,     //GVSP单包数据分包大小, 单位字节
			LX_INT_SAVE_PARAMS_GROUP = 1075,    //将相机当前配置保存为指定的参数组
			LX_INT_LOAD_PARAMS_GROUP = 1076,    //一键加载指定索引的参数组
			LX_INT_GVCP_TIME_OUT = 1080,        //设备控制,信息获取最大超时时间 默认1800ms 单位ms

			/*float feature*/
			LX_FLOAT_FILTER_LEVEL = 2001,      //当LX_INT_FILTER_MODE为FILTER_SIMPLE时,可设置滤波等级，[0, 1]，值越大，滤波越强，等于0表示关闭滤波，
			LX_FLOAT_EST_OUT_EXPOSURE = 2002,  //是否评估过曝数据，[0, 1]，为1则过曝数据无效
			LX_FLOAT_LIGHT_INTENSITY = 2003,   //光强度，[0, 1]，部分型号支持
			LX_FLOAT_3D_DEPTH_FPS = 2004,      //深度图当前帧率，只可获取
			LX_FLOAT_3D_AMPLITUDE_FPS = 2005,  //强度图当前帧率，只可获取
			LX_FLOAT_2D_IMAGE_FPS = 2006,      //RGB图当前帧率，只可获取
			LX_FLOAT_DEVICE_TEMPERATURE = 2007,//相机当前温度, 只可获取
			LX_FLOAT_CONFIDENCE = 2008,        //置信度，[0-1]，越小则保留越多数据

			/*bool feature*/
			LX_BOOL_CONNECT_STATE = 3001,           //当前连接状态
			LX_BOOL_ENABLE_3D_DEPTH_STREAM = 3002,  //开启/关闭深度数据流(部分相机支持)
			LX_BOOL_ENABLE_3D_AMP_STREAM = 3003,    //开启/关闭强度数据流(部分相机支持)
			LX_BOOL_ENABLE_3D_AUTO_EXPOSURE = 3006, //3D自动曝光使能
			LX_BOOL_ENABLE_3D_UNDISTORT = 3007,     //3D反畸变使能
			LX_BOOL_ENABLE_ANTI_FLICKER = 3008,     //抗频闪使能，LED环境照明可能导致数据存在明显波纹，部分型号支持
			LX_BOOL_ENABLE_MULTI_MACHINE = 3018,    //多机模式使能，TOF会有多机干扰，开启此模式可以自动检测并缓解多机干扰
			LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR = 3019,  //HDR（多曝光高动态范围模式）使能

			LX_BOOL_ENABLE_2D_STREAM = 3011,        //开启/关闭2D数据流
			LX_BOOL_ENABLE_2D_AUTO_EXPOSURE = 3012, //2D自动曝光使能  
			LX_BOOL_ENABLE_2D_UNDISTORT = 3015,     //2D图像反畸变使能
			LX_BOOL_ENABLE_BACKGROUND_AMP = 3017,   //强度背景光使能

			LX_BOOL_ENABLE_SYNC_FRAME = 3020,       //是否开启强制帧同步, 默认数据实时性优先，若需要RGBD同步, 需要开启该模式
			LX_BOOL_ENABLE_TERM_RESISTOR = 3021,    //端接电阻使能
			LX_BOOL_ENABLE_IMU = 3023,              //使能IMU数据，通过回调DcRegisterImuDataCallback获取数据

			LX_BOOL_ENABLE_LASER_BRIGHT = 3110,     //激光器亮灭使能

			/*string feature*/
			LX_STRING_DEVICE_VERSION = 4001,        //设备版本号
			LX_STRING_FIRMWARE_NAME = 4003,         //固件文件名，用于升级设备版本，部分型号需要重新打开相机
			LX_STRING_ALGORITHM_PARAMS = 4005,      //内置算法参数，根据当前设置的LX_ALGORITHM_MODE，返回对应的json格式字符串
			LX_STRING_ALGORITHM_VERSION = 4006,     //内置算法版本号，根据当前设置的LX_ALGORITHM_MODE，返回对应的版本号
			LX_STRING_DEVICE_OS_VERSION = 4007,     //设备系统镜像版本号
			LX_STRING_IMPORT_PARAMS_FROM_FILE = 4008,   //从本地文件加载参数到相机
			LX_STRING_EXPORT_PARAMS_TO_FILE = 4009,      //将相机当前参数导出到本地文件
			LX_STRING_CUSTOM_ID = 4010,             //客户自定义设备ID标识

			/*command feature*/
			LX_CMD_GET_NEW_FRAME = 5001,    //主动更新当前最新数据，调用之后才可以获取相关数据指针。
											//开启多个数据流时，每个数据流更新都会返回正常，除非设置LX_BOOL_ENABLE_SYNC_FRAME为true
											//建议采用回调方式DcRegisterFrameCallback更新数据，此时不需调用此接口
			LX_CMD_GET_NEW_FRAME_3D = 5009, //仅处理并更新3D数据(depth/amp)，需要的时候调用可降低计算资源占用
			LX_CMD_GET_NEW_FRAME_2D = 5010, //仅处理并更新2D数据(rgb)，需要的时候调用可降低计算资源占用
			LX_CMD_RETURN_VERSION = 5002,   //回退上一版本
			LX_CMD_RESTART_DEVICE = 5003,   //重启相机，部分型号需要重新打开相机
			LX_CMD_RESET_PARAM = 5007,      //恢复默认参数
			LX_CMD_SOFTWARE_TRIGGER = 5008, //软触发执行指令

			/*ptr feature*/
			LX_PTR_FRAME_DATA = 6009,       //获取完整一帧数据，输出结果参考结构体FrameInfo
			LX_PTR_XYZ_DATA = 6004,      //获取点云数据指针，float*类型三通道(x, y, z为一组数据，依次循环)
										 //数据长度为LX_INT_3D_IMAGE_WIDTH*LX_INT_3D_IMAGE_HEIGHT*sizeof(float)*3   
			LX_PTR_XYZIRT_DATA = 6020, //获取雷达点云结构体，参考LxPointCloudData
			LX_PTR_3D_EXTRIC_PARAM = 6007, //获取3D图像外参，float*类型指针，长度固定为12*sizeof(float)(前9个表示旋转矩阵，后3个表示平移向量)
			LX_PTR_IMU_EXTRIC_PARAM = 6015, //获取IMU图像外参，float*类型指针，长度固定为12*sizeof(float)(前9个表示旋转矩阵，后3个表示平移向量)
			LX_PTR_2D_INTRINSIC_PARAMETERS = 6013,  //获取2D图像内参，参考LxIntrinsicParameters
			LX_PTR_3D_INTRINSIC_PARAMETERS = 6014,  //获取3D图像内参，参考LxIntrinsicParameters


			/*以下不建议使用*/
			LX_BOOL_ENABLE_2D_TO_DEPTH = 3016,  //建议使用LX_INT_RGBD_ALIGN_MODE
			LX_STRING_FILTER_PARAMS = 4004,     //建议使用FILTER_NORMAL
			LX_PTR_2D_IMAGE_DATA = 6001, //建议使用LX_PTR_FRAME_DATA
			LX_PTR_3D_AMP_DATA = 6002,   //建议使用LX_PTR_FRAME_DATA
			LX_PTR_3D_DEPTH_DATA = 6003, //建议使用LX_PTR_FRAME_DATA
			LX_PTR_ALGORITHM_OUTPUT = 6008, //建议使用LX_PTR_FRAME_DATA，获取内置算法输出
											//当开启模式为MODE_AVOID_OBSTACLE，输出结果为LxAvoidanceOutput指针，参考struct LxAvoidanceOutput，
											//当开启模式为MODE_PALLET_LOCATE，输出结果为LxPalletPose指针，参考struct LxPalletPose,
											//当开启模式为MODE_VISION_LOCATION，输出结果为LxLocation指针，参考struct LxLocation
											//当开启模式为MODE_AVOID_OBSTACLE2，输出结果为LxAvoidanceOutputN指针，参考struct LxAvoidanceOutputN
			LX_PTR_2D_INTRIC_PARAM = 6005,  //建议使用LX_PTR_2D_INTRINSIC_PARAMETERS,float*类型指针，长度固定为9(4内参5畸变）
			LX_PTR_3D_INTRIC_PARAM = 6006, //建议使用LX_PTR_3D_INTRINSIC_PARAMETERS,float*类型指针，长度固定为9(4内参5畸变）
			LX_PTR_2D_NEW_INTRIC_PARAM = 6010,  //建议使用LX_PTR_2D_INTRINSIC_PARAMETERS,float*类型指针，长度固定为18(4内参14畸变）
			LX_PTR_3D_NEW_INTRIC_PARAM = 6011, //建议使用LX_PTR_3D_INTRINSIC_PARAMETERS,float*类型指针，长度固定为18(4内参14畸变）

        };

        [StructLayout(LayoutKind.Sequential)]
        public struct LxDeviceInfo
        {
            public long handle;               //设备唯一标识
            public LX_DEVICE_TYPE dev_type;       //设备类型
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] id;                   //设备id
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] ip;                   //设备ip:port
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] sn;                   //设备序列号
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] mac;                  //设备mac地址
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] firmware_ver;         //设备软件版本号
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] algor_ver;            //设备算法版本号
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] name;                 //设备名称，如：camera_M3_192.168.11.13_9803
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] reserve;              //预留字段
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public char[] reserve2;             //预留字段2
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
            public char[] reserve3;             //预留字段3
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)]
            public char[] reserve4;            //预留字段4
        }

        //数据格式。3D强度图和深度图有不同的数据格式，获取数据指针需要使用对应的feature
        public enum LX_DATA_TYPE
        {
            LX_DATA_UNSIGNED_CHAR = 0,
            LX_DATA_UNSIGNED_SHORT = 2,
            LX_DATA_SIGNED_SHORT = 3,
            LX_DATA_FLOAT = 5,
            LX_DATA_OBSTACLE = 16,
            LX_DATA_PALLET = 17,
            LX_DATA_LOCATION = 18,
            LX_DATA_OBSTACLE2 = 19,
        }
        public struct FrameDataInfo
        {
            public LX_DATA_TYPE frame_data_type;
            public int frame_width;
            public int frame_height;
            public int frame_channel;
            public IntPtr frame_data;
            public ulong sensor_timestamp;     //sensor出图时间戳
            public ulong recv_timestamp;       //接收完帧数据时的时间戳
        };

        //数据帧具体信息
        public struct FrameInfo
        {
            public LX_STATE frame_state;
            public ulong handle;

            public FrameDataInfo depth_data;   //深度图 
            public FrameDataInfo amp_data;     //强度图
            public FrameDataInfo rgb_data;     //rgb图
            public FrameDataInfo app_data;     //算法输出结果
            public IntPtr reserve_data;         //扩展预留字段
        };
        public struct FrameExtendInfo
        {
            public uint depth_frame_id;
            public uint amp_frame_id;
            public uint rgb_frame_id;
            public uint app_frame_id;
            public IntPtr reserve_data1; // 预留字段
            public IntPtr reserve_data2; // 预留字段
            public IntPtr reserve_data3; // 预留字段
        }

		//雷达点数据结构体，注意！！！开启3D反畸变或者rgbd对齐后时间戳不准！！！区域索引和行列位置也会相应变化
		public struct LxPointXYZIRT {
			public float x;
			public float y;
			public float z;
			public uint intensity;  //该点对应的强度数据，需要使能AMP图之后才有效
			public uint offset_time;//时间戳相对于timebase的时间偏移单位微秒，实际使用：timebase+offset_time
			public uchar  roi_index;  //roi区域索引序号
			public ushort row_pos;    //点在图像中的行位置，从0开始
			public ushort col_pos;    //点在图像中的列位置，从0开始
			public uchar  tag;        //预留 
		};

		//雷达点云数据结构
		public struct LxPointCloudData {
			public ulong timebase;    // 基础时间戳，该帧图像开始曝光的时间戳（us）
			public uint point_num;   // 有效点个数
			public LxPointXYZIRT* points;// 点数组（内部管理内存）
		};

		//相机内参的结构体
		public struct {
			uint width;              // 图像宽度
			uint height;             // 图像高度
			float intrinsics[9];        // 内参3x3矩阵[fx 0 cx; 0 fy cy; 0 0 1]
			LX_DISTORTION_MODEL distortion_model; // 畸变模型类型
			float* distortion_coeffs;   // 畸变系数指针（内部管理内存）
			uchar num_distortion_coeffs; // 畸变系数的数量
		};

        public enum LX_OPEN_MODE
        {
            OPEN_BY_INDEX = 0,     //按搜索列表中索引下标方式打开，对应的参数为索引号，当搜索到的设备列表发生变化时，选择打开的设备也会不一样
            OPEN_BY_IP = 1,        //按搜索列表中对应ip方式打开，对应的参数为设备ip或ip:port
            OPEN_BY_SN = 2,        //按搜索列表中对应sn方式打开，对应的参数为设备sn
            OPEN_BY_ID = 3,        //按搜索列表中对应id方式打开，对应的参数为设备id
        };
        public struct LxIntValueInfo
        {
            public bool set_available;   //当前值是否可设置, true-可设置，false-不可设置
            public int cur_value;        //当前值
            public int max_value;        //最大值
            public int min_value;        //最小值
            public int reserve1;       //预留字段
            public int reserve2;       //预留字段
            public int reserve3;       //预留字段
            public int reserve4;       //预留字段
        }
        public struct LxFloatValueInfo
        {
            public bool set_available;     //当前值是否可设置, true-可设置，false-不可设置
            public float cur_value;        //当前值
            public float max_value;        //最大值
            public float min_value;        //最小值
            public float reserve1;       //预留字段
            public float reserve2;       //预留字段
            public float reserve3;       //预留字段
            public float reserve4;       //预留字段
        }

        public static string DcGetApiVersion()
        {
            string str = Marshal.PtrToStringAnsi(_DcGetApiVersion());
            return str;
        }
    }
}