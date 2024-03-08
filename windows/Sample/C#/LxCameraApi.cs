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
    public class LxCamera
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
        public static extern LX_STATE DcSaveXYZ(long handle, IntPtr filename, int datatype = 0);

        [DllImport("LxCameraApi.dll", EntryPoint = "DcSetCameraIp", ExactSpelling = false)]
        public static extern LX_STATE DcSetCameraIp(long handle, IntPtr ip, int port, IntPtr IntPtr);

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
        public static extern LX_STATE DcGetPtrValue(long handle, int cmd, ref IntPtr value);

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
            LX_E_DEVICE_ERROR = -6,         //设备故障
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
        };
        public enum LX_DEVICE_TYPE
        {
            LX_DEVICE_M2 = 1001,
            LX_DEVICE_M3,
            LX_DEVICE_M4,
            LX_DEVICE_M5,
            LX_DEVICE_S1 = 2001,
            LX_DEVICE_S2,
            LX_DEVICE_I1 = 3001,
            LX_DEVICE_I2,
            LX_DEVICE_T1 = 4001,
            LX_DEVICE_T2,
            LX_DEVICE_WK = 5001,
            LX_DEVICE_NULL
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
        };
        public enum LX_CAMERA_WORK_MODE
        {
            MODE_ONLY_GIGE = 0,             //正常模式，按需采图，默认为该状态
            MODE_COLLECT_FOREVER = 1,       //始终采图
        };
        public enum LX_CAMERA_FEATURE
        {
            /*int feature*/
            LX_INT_FIRST_EXPOSURE = 1001,   //默认高积分曝光值，单位us, 针对多积分情况为第一个积分的曝光时间
            LX_INT_SECOND_EXPOSURE = 1002,  //默认低积分曝光值，单位us, 针对多积分情况为第二个积分的曝光时间
            LX_INT_THIRD_EXPOSURE = 1003,   //针对多积分情况为第三个积分的曝光时间
            LX_INT_FOURTH_EXPOSURE = 1004,  //针对多积分情况为第四个积分的曝光时间
            LX_INT_GAIN = 1005,             //增益，与曝光效果等价。会引入噪声，可适当调节增益防止曝光参数过大

            LX_INT_MIN_DEPTH = 1011,        //最小深度值，设置<=0则恢复默认值
            LX_INT_MAX_DEPTH = 1012,        //最大深度值，设置<=0则恢复默认值
            LX_INT_MIN_AMPLITUDE = 1013,    //有效信号最小强度值
            LX_INT_MAX_AMPLITUDE = 1014,    //有效信号最大强度值
            LX_INT_CONTRAST = 1015,         //强度对比度，可去除低信号数据，减小噪声
            LX_INT_CODE_MODE = 1016,        //结构光相机编码模式，参考LX_STRUCT_LIGHT_CODE_MODE
            LX_INT_BIT_DEPTH = 1017,        //像素深度，结构光支持8bit和10bit。10bit可以实现更大动态范围，但是会降低相机帧率
            LX_INT_WORK_MODE = 1018,        //工作模式，0-按需采图,默认为该状态 1-始终采图
            LX_INT_LINK_SPEED = 1019,       //协商的网卡网速 100-百兆，1000-千兆, 只支持获取，不可设置

            //3D图像参数
            LX_INT_3D_IMAGE_WIDTH = 1021,       //3D图像分辨率宽度,(ROI/Binning/2D对齐后会变化,需重新获取)
            LX_INT_3D_IMAGE_HEIGHT = 1022,      //3D图像分辨率高度,(ROI/Binning/2D对齐后会变化,需重新获取)
            LX_INT_3D_IMAGE_OFFSET_X = 1023,    //ROI水平偏移像素，ROI设置中WIDTH HEIGHT OFFSET_X OFFSET_Y一起设置, 设置参数请用DcSetROI
            LX_INT_3D_IMAGE_OFFSET_Y = 1024,    //ROI垂直偏移像素，ROI设置中WIDTH HEIGHT OFFSET_X OFFSET_Y一起设置, 设置参数请用DcSetROI
            LX_INT_3D_BINNING_MODE = 1025,      //3D图像binning，参考LX_BINNING_MODE
            LX_INT_3D_DEPTH_DATA_TYPE = 1026,   //深度图像数据格式，只能获取，对应的值参考LX_DATA_TYPE

            //3D强度图像，尺寸与3D深度图一致
            LX_INT_3D_AMPLITUDE_CHANNEL = 1031,  //3D强度图像通道数，单色为1，彩色为3
            LX_INT_3D_AMPLITUDE_GET_TYPE = 1032, //获取强度图像的方式，参考LX_GET_AMPLITUDE_MODE(部分型号支持)
            LX_INT_3D_AMPLITUDE_EXPOSURE = 1033, //LX_INT_3D_AMPLITUDE_GET_TYPE为MANUAL_EXPOSURE时的曝光值
            LX_INT_3D_AMPLITUDE_INTENSITY = 1034,//LX_INT_3D_AMPLITUDE_GET_TYPE为AUTO_EXPOSURE时的目标亮度
            LX_INT_3D_AMPLITUDE_DATA_TYPE = 1035,//强度图像数据格式，只能获取，对应的值参考LX_DATA_TYPE

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
            LX_INT_2D_AUTO_EXPOSURE_LEVEL,      //2D图像自动曝光时曝光等级[0-100], 0-整体更暗， 100-整体更亮

            LX_INT_TOF_GLOBAL_OFFSET = 1061,    //TOF深度数据偏移
            LX_INT_TOF_MATRIX_SCALE = 1062,     //TOF图像反畸变系数
            LX_INT_STROBE_EXPOSURE = 1063,      //闪光灯曝光时间，部分型号支持
            LX_INT_STROBE_MODE = 1064,          //闪光灯开启模式，部分型号支持,对应的值参考LX_STROBE_MODE
            LX_INT_ALGORITHM_MODE = 1065,       //内置算法开启模式, 部分型号支持,对应的值参考LX_ALGORITHM_MODE
            LX_INT_MODBUS_ADDR = 1066,          //modbus地址

            /*float feature*/
            LX_FLOAT_FILTER_LEVEL = 2001,      //滤波等级，[0, 1]，值越大，滤波越强，等于0表示关闭滤波，//更详细滤波配置参考LX_STRING_FILTER_PARAMS
            LX_FLOAT_EST_OUT_EXPOSURE = 2002,  //是否评估过曝数据，[0, 1]，为1则过曝数据无效
            LX_FLOAT_LIGHT_INTENSITY = 2003,   //光强度，[0, 1]，部分型号支持
            LX_FLOAT_3D_DEPTH_FPS = 2004,      //深度图当前帧率，普通相机的深度，强度，原图，RGB图帧率相同，gige相机的作区分, 只可获取
            LX_FLOAT_3D_AMPLITUDE_FPS = 2005,  //强度图当前帧率，普通相机的深度，强度，原图，RGB图帧率相同，gige相机的作区分, 只可获取
            LX_FLOAT_2D_IMAGE_FPS = 2006,      //RGB图当前帧率，普通相机的深度，强度，原图，RGB图帧率相同，gige相机的作区分, 只可获取
            LX_FLOAT_DEVICE_TEMPERATURE = 2007,//相机当前温度, 只可获取
            LX_FLOAT_CONFIDENCE = 2008,        //wk置信度，[0-1]，越小则保留越多数据

            /*bool feature*/
            LX_BOOL_CONNECT_STATE = 3001,           //当前连接状态
            LX_BOOL_ENABLE_3D_DEPTH_STREAM = 3002,  //开启/关闭深度数据流(部分相机支持)
            LX_BOOL_ENABLE_3D_AMP_STREAM = 3003,    //开启/关闭强度数据流(部分相机支持)
            LX_BOOL_ENABLE_3D_HFLIP = 3004,         //3D图像水平翻转
            LX_BOOL_ENABLE_3D_VFLIP = 3005,         //3D图像垂直翻转
            LX_BOOL_ENABLE_3D_AUTO_EXPOSURE = 3006, //3D自动曝光使能
            LX_BOOL_ENABLE_3D_UNDISTORT = 3007,     //3D反畸变使能
            LX_BOOL_ENABLE_ANTI_FLICKER = 3008,     //抗频闪使能，LED环境照明可能导致数据存在明显波纹，部分型号支持
            LX_BOOL_ENABLE_HDR = 3009,              //HDR（多曝光高动态范围模式）使能，如关闭则曝光参数只有LX_INT_FIRST_EXPOSURE有效

            LX_BOOL_ENABLE_2D_STREAM = 3011,        //开启/关闭2D数据流
            LX_BOOL_ENABLE_2D_AUTO_EXPOSURE = 3012, //2D自动曝光使能
            LX_BOOL_ENABLE_2D_HFLIP = 3013,         //2D图像水平翻转
            LX_BOOL_ENABLE_2D_VFLIP = 3014,         //2D图像垂直翻转    
            LX_BOOL_ENABLE_2D_UNDISTORT = 3015,     //2D图像反畸变使能
            LX_BOOL_ENABLE_2D_TO_DEPTH = 3016,      //2D3D图像对齐使能，3D图像分辨率会改变            

            /*string feature*/
            LX_STRING_DEVICE_VERSION = 4001,        //设备版本号
            LX_STRING_DEVICE_LOG_NAME = 4002,       //日志文件名，用于获取设备日志
            LX_STRING_FIRMWARE_NAME = 4003,         //固件文件名，用于升级设备版本，部分型号需要重新打开相机
            LX_STRING_FILTER_PARAMS = 4004,         //滤波算法参数,json格式的字符串，//关闭滤波或配置简单的滤波参考LX_FLOAT_FILTER_LEVEL
            LX_STRING_ALGORITHM_PARAMS = 4005,      //内置算法参数,不同的开启模式，对应不同的json格式字符串,前提需要设置过LX_ALGORITHM_MODE
            LX_STRING_ALGORITHM_VERSION = 4006,     //内置算法版本号,不同的开启模式，返回对应的版本号,只能获取,前提需要设置过LX_ALGORITHM_MODE
            LX_STRING_DEVICE_OS_VERSION = 4007,     //设备系统镜像版本号

            /*command feature*/
            LX_CMD_GET_NEW_FRAME = 5001,    //更新一次数据
            LX_CMD_RETURN_VERSION = 5002,   //回退上一版本
            LX_CMD_RESTART_DEVICE = 5003,   //重启相机，部分型号需要重新打开相机
            LX_CMD_WHITE_BALANCE = 5004,    //自动白平衡
            LX_CMD_RESET_PARAM = 5007,      //恢复默认参数

            /*ptr feature*/
            LX_PTR_2D_IMAGE_DATA = 6001, //获取2D图像数据指针(使用前先调用LX_INT_2D_IMAGE_DATA_TYPE获取数据类型)
                                         //当2D图像数据类型为unsigned char类型，参考LX_INT_2D_IMAGE_DATA_TYPE
                                         //2D图像数据长度，LX_INT_2D_IMAGE_WIDTH * LX_INT_2D_IMAGE_HEIGHT * LX_INT_2D_IMAGE_CHANNEL * sizeof(unsigned char)
                                         //当2D图像数据类型为unsigned short类型，参考LX_INT_2D_IMAGE_DATA_TYPE
                                         //2D图像数据长度，LX_INT_2D_IMAGE_WIDTH * LX_INT_2D_IMAGE_HEIGHT * LX_INT_2D_IMAGE_CHANNEL * sizeof(unsigned short)
            LX_PTR_3D_AMP_DATA = 6002,   //获取3D图像强度图数据指针(使用前先调用LX_INT_3D_AMPLITUDE_DATA_TYPE获取数据类型)
                                         //结构光相机强度图像为unsigned char格式，参考LX_INT_AMPLITUDE_DATA_TYPE
                                         //获取3D传感器的强度图像,LX_INT_3D_IMAGE_WIDTH*LX_INT_3D_IMAGE_HEIGHT*LX_INT_3D_AMPLITUDE_CHANNEL*sizeof(unsigned char)
                                         //TOF相机强度图像为unsigned short格式，参考LX_INT_AMPLITUDE_DATA_TYPE
                                         //获取3D传感器的强度图像,LX_INT_3D_IMAGE_WIDTH*LX_INT_3D_IMAGE_HEIGHT*LX_INT_3D_AMPLITUDE_CHANNEL*sizeof(unsigned short)   
            LX_PTR_3D_DEPTH_DATA = 6003, //获取3D图像强度图数据指针(使用前先调用LX_INT_3D_DEPTH_DATA_TYPE获取数据类型)
                                         //TOF相机深度图像为unsigned short格式，参考LX_INT_AMPLITUDE_DATA_TYPE
                                         //获取3D深度图像,LX_INT_3D_IMAGE_WIDTH*LX_INT_3D_IMAGE_HEIGHT*sizeof(unsigned short)   
                                         //结构光相机深度图像为float格式，参考LX_INT_AMPLITUDE_DATA_TYPE
                                         //获取3D深度图像,LX_INT_3D_IMAGE_WIDTH*LX_INT_3D_IMAGE_HEIGHT*sizeof(float)    
            LX_PTR_XYZ_DATA = 6004,      //获取点云数据，一定为float*类型指针(x, y, z为一组数据，依次循环)
                                         //数据长度为对应LX_INT_3D_IMAGE_WIDTH*LX_INT_3D_IMAGE_HEIGHT*sizeof(float)*3   
            LX_PTR_2D_INTRIC_PARAM = 6005,  //获取2D图像内参，float*类型指针，长度固定为9*sizeof(float)(fx,fy,cx,cy,k1,k2,k3,p1,p2)
            LX_PTR_3D_INTRIC_PARAM = 6006, //获取3D图像内参, float*类型指针，长度固定为9*sizeof(float)(fx,fy,cx,cy,k1,k2,k3,p1,p2)
            LX_PTR_3D_EXTRIC_PARAM = 6007, //获取3D图像外参，float*类型指针 ，长度固定为12*sizeof(float)(前9个表示旋转矩阵，后3个表示平移向量)
            LX_PTR_ALGORITHM_OUTPUT = 6008, //获取内置算法输出，当开启模式为MODE_AVOID_OBSTACLE，输出结果为LxAvoidanceOutput指针，参考struct LxAvoidanceOutput，
                                            //当开启模式为MODE_PALLET_LOCATE，输出结果为LxPalletPose指针，参考struct LxPalletPose
            LX_PTR_FRAME_DATA = 6009,       //获取完整一帧数据，输出结果参考结构体FrameInfo
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