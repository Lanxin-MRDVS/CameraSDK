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
#include <stdint.h>

// Error type definitions
typedef enum LX_STATE
{
    LX_SUCCESS = 0,                 // Success
    LX_ERROR = -1,                  // Unknown error
    LX_E_NOT_SUPPORT = -2,          // Function not supported
    LX_E_NETWORK_ERROR = -3,        // Network communication error
    LX_E_INPUT_ILLEGAL = -4,        // Invalid input
    LX_E_RECONNECTING = -5,         // Device reconnecting
    LX_E_DEVICE_ERROR = -6,         // Device error or no response
    LX_E_DEVICE_NEED_UPDATE = -7,   // Device firmware too old; update required
    LX_E_API_NEED_UPDATE = -8,      // API version too old
    LX_E_CTRL_PERMISS_ERROR = -9,   // Exclusive control permission failed
    LX_E_GET_DEVICEINFO_ERROR = -10,// Failed to get device info
    LX_E_IMAGE_SIZE_ERROR = -11,     // Image size mismatch; reopen required
    LX_E_IMAGE_PARTITION_ERROR = -12, // Image parsing failed (pixformat incorrect); try reopening
    LX_E_DEVICE_NOT_CONNECTED = -13, // Camera not connected
    LX_E_DEVICE_INIT_FAILED = -14,   // Camera initialization failed
    LX_E_DEVICE_NOT_FOUND = -15,     // Camera not found (or no matching camera)
    LX_E_FILE_INVALID = -16,         // File error (name/type/format invalid or open failed)
    LX_E_CRC_CHECK_FAILED = -17,     // File CRC/MD5 check failed
    LX_E_TIME_OUT = -18,             // Timeout
    LX_E_FRAME_LOSS = -19,           // Frame loss
    LX_E_ENABLE_ANYSTREAM_FAILED = -20, // Failed to enable stream
    LX_E_NOT_RECEIVE_STREAM = -21,      // No stream data received
    LX_E_PARSE_STREAM_FAILED = -22,     // Stream started but parse failed
    LX_E_PROCESS_IMAGE_FAILED = -23,    // Image processing failed
    LX_E_SETTING_NOT_ALLOWED = -24,     // Setting not allowed in always-on mode
    LX_E_LOAD_DATAPROCESSLIB_ERROR = -25, // Failed to load data processing library
    LX_E_FUNCTION_CALL_LOGIC_ERROR = -26, // Function call logic error
    LX_E_IPAPPDR_UNREACHABLE_ERROR = -27, // IP unreachable or network configuration error
    LX_E_FRAME_ID_NOT_MATCH        = -28, // Frame not synchronized within timeout
    LX_E_FRAME_MULTI_MACHINE       = -29, // Multi-camera interference detected
    LX_E_FRAME_IMAGE_ERROR = -30, // Image-related device error in frame data

    LX_W_LOAD_DATAPROCESSLIB_ERROR = 25, // Data process lib load failed; other functions unaffected
}LX_STATE;

#define LX_API LX_EXTC LX_EXPORT LX_STATE LX_STDC
#define LX_API_STR LX_EXTC LX_EXPORT const char* LX_STDC

typedef unsigned long long DcHandle;

// Device model
typedef enum LX_DEVICE_TYPE {
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
    LX_DEVICE_S10 = 2108,
    LX_DEVICE_S10PRO = 2109,

    LX_DEVICE_S11 = 2201,

    LX_DEVICE_I1 = 3001,
    LX_DEVICE_I2,
    LX_DEVICE_T1 = 4001,
    LX_DEVICE_T2,

    LX_DEVICE_H3 = 5001,
    LX_DEVICE_H4 = 5002,

    LX_DEVICE_V1Pro = 6001,
    LX_DEVICE_V2Pro = 6002,

    LX_DEVICE_NULL = 0
}LX_DEVICE_TYPE;

// Device series (unused)
typedef enum LX_DEVICE_SERIALS {
    LX_SERIAL_GIGE = 1,
    LX_SERIAL_WK,
    LX_SERIAL_OTHER,
    LX_SERIAL_ALL,
    LX_SERIAL_LOCAL_LOOP, // Local loopback (SDK and camera app on the same host)
}LX_DEVICE_SERIALS;

// Device details
typedef struct LxDeviceInfo
{
    DcHandle handle;               // Unique device identifier
    LX_DEVICE_TYPE dev_type;       // Device type
    char id[32];                   // Device ID
    char ip[32];                   // Device IP:port
    char sn[32];                   // Device serial number
    char mac[32];                  // Device MAC address
    char firmware_ver[32];         // Device firmware version
    char algor_ver[32];            // Device algorithm version
    char name[32];                 // Device name, e.g. camera_M3_192.168.11.13_9803
    char reserve[32];              // Reserved, subnet mask // add at 20231120
    char reserve2[32];             // Reserved2, gateway IP  // add at 20231120
    char reserve3[64];             // Reserved3, [0:3] indicates current access mode
    char reserve4[128];            // Reserved4
}LxDeviceInfo;

// Integer parameter struct
typedef struct LxIntValueInfo
{
    bool set_available;   // Whether current value is settable; true=settable, false=not
    int cur_value;        // Current value
    int max_value;        // Max value
    int min_value;        // Min value
    int reserve[4];       // Reserved
}LxIntValueInfo;

// Float parameter struct
typedef struct LxFloatValueInfo
{
    bool set_available;     // Whether current value is settable; true=settable, false=not
    float cur_value;        // Current value
    float max_value;        // Max value
    float min_value;        // Min value
    float reserve[4];       // Reserved
}LxFloatValueInfo;

// Camera open mode
typedef enum LX_OPEN_MODE
{
    OPEN_BY_INDEX = 0,     // Open by index in the search list; index changes if the list changes
    OPEN_BY_IP = 1,        // Open by IP in the search list; param is device IP or IP:port
    OPEN_BY_SN = 2,        // Open by SN in the search list; param is device SN
    OPEN_BY_ID = 3,        // Open by ID in the search list; param is device ID
}LX_OPEN_MODE;

// Data types. 3D amplitude and depth use different formats; use the matching feature to get pointers.
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
    LX_DATA_CUSTOM = 100,// Custom data type defined by camera application
}LX_DATA_TYPE;

// Image binning mode
typedef enum LX_BINNING_MODE
{
    LX_BINNING_1X1 = 0,
    LX_BINNING_2X2 = 1,
    LX_BINNING_4X4 = 2
}LX_BINNING_MODE;

// Structured-light camera coding mode
typedef enum LX_STRUCT_LIGHT_CODE_MODE
{
    LX_CODE_NORMAL = 1,// Normal
    LX_CODE_STATBLE = 2,// Stable
    LX_CODE_ENHANCE = 3, // High-precision enhanced
}LX_STRUCT_LIGHT_CODE_MODE;

// Built-in camera algorithms
typedef enum LX_ALGORITHM_MODE 
{
    MODE_ALL_OFF = 0,                 // Disable built-in obstacle avoidance
    MODE_AVOID_OBSTACLE = 1,          // Built-in obstacle avoidance
    MODE_PALLET_LOCATE = 2,           // Built-in pallet docking
    MODE_VISION_LOCATION = 3,         // Built-in visual localization
    MODE_AVOID_OBSTACLE2 = 4,         // Built-in obstacle avoidance V2
    MODE_GENERIC_DATA = 5,        // Custom data type (application algorithm platform)
}LX_ALGORITHM_MODE;

// Camera work mode
typedef enum LX_CAMERA_WORK_MODE 
{
    KEEP_HEARTBEAT = 0,             // Keep heartbeat; camera auto-standby when SDK heartbeat stops
    WORK_FOREVER = 1,               // Always on; some parameters cannot be set
}LX_CAMERA_WORK_MODE;

// Camera trigger mode
typedef enum LX_TRIGGER_MODE 
{
    LX_TRIGGER_MODE_OFF,          // Trigger off, streaming mode (default)
    LX_TRIGGER_SOFTWARE,          // Software trigger mode
    LX_TRIGGER_HARDWARE,          // Hardware trigger mode
}LX_TRIGGER_MODE;

// IO work mode
typedef enum LX_IO_WORK_MODE {
    ALGORITHM_IO_MODE = 0,  // IO for built-in algorithm I/O
    USER_IO_MODE = 1,       // User controls IO state; see LX_IO_OUT_USERCTRL_MODE
    TRIGGER_IO_MODE = 2,    // IO as trigger signal output; only in trigger mode
}LX_IO_WORK_MODE;

// Custom IO output state
typedef enum LX_IO_OUTPUT_STATE {
    OUT1_0_OUT2_0 = 0,
    OUT1_0_OUT2_1 = 1,
    OUT1_1_OUT2_0 = 2,
    OUT1_1_OUT2_1 = 3,
}LX_IO_OUTPUT_STATE;

typedef enum LX_IO_HARDWARE_MODE {
    OPEN_DRAIN_MODE = 0,  // Open-drain always-on mode
    PUSH_PULL_MODE = 1,   // Push-pull high-level mode
}LX_IO_HARDWARE_MODE;

// Filter mode
typedef enum LX_FILTER_MODE {
    FILTER_SIMPLE = 1,
    FILTER_NORMAL = 2,
    FILTER_EXPERT = 3,
}LX_FILTER_MODE;

// 3D frequency mode
typedef enum LX_3D_FREQ_MODE {
    FREQ_SINGLE_3D = 0,// Single frequency
    FREQ_MULTI_3D = 1,// Multi frequency
    FREQ_TRIPLE_3D = 2,// Triple frequency (not supported by new firmware)
}LX_3D_FREQ_MODE;

// RGB alignment mode
typedef enum LX_RGBD_ALIGN_MODE {
    RGBD_ALIGN_OFF = 0,   // Disable RGBD alignment; RGB and depth use their own frames
    DEPTH_TO_RGB = 1,     // Align depth to RGB; resolution/coords follow RGB
    RGB_TO_DEPTH = 2,     // Align RGB to depth; resolution/coords follow depth
}LX_RGBD_ALIGN_MODE;

typedef enum LX_CAN_PROTOCOL_TYPE 
{
    CAN_MRDVS = 0,
    CAN_KECONG = 1,
}LX_CAN_PROTOCOL_TYPE;

typedef enum LX_POWER_MODE_TYPE 
{
    POWER_MODE_LOW = 0,       // Low power mode (disable some functions)
    POWER_MODE_DEFAULT = 1,   // Default power mode (balanced)
    POWER_MODE_HIGH,          // High performance mode (enable all functions)
    POWER_MODE_STANDBY,       // Standby mode (lowest power, pause image capture)
}LX_POWER_MODE_TYPE;

// Image display related info
typedef struct FrameDataInfo
{
    LX_DATA_TYPE frame_data_type;
    int frame_width;
    int frame_height;
    int frame_channel;
    void* frame_data;
    unsigned long long sensor_timestamp;     // Sensor output timestamp
    unsigned long long recv_timestamp;       // Timestamp when frame data received
}FrameDataInfo;


// Frame data details
typedef struct FrameInfo {
    LX_STATE frame_state;
    DcHandle handle;

    FrameDataInfo depth_data;   // Depth image
    FrameDataInfo amp_data;     // Amplitude image
    FrameDataInfo rgb_data;     // RGB image
    FrameDataInfo app_data;     // Algorithm output
    void* reserve_data;         // Reserved extension
}FrameInfo;

// Frame extended info struct
typedef struct FrameExtendInfo {
    unsigned int depth_frame_id;
    unsigned int amp_frame_id;
    unsigned int rgb_frame_id;
    unsigned int app_frame_id;
    void* reserve_data1;           // Reserved
    void* reserve_data2;           // Reserved
    void* reserve_data3;           // Reserved
}FrameExtendInfo;

// Current camera status
typedef enum LX_CAMERA_STATUS
{
    STATUS_CLOSED,              // Camera not opened or closed
    STATUS_OPENED_UNSTARTED,    // Camera opened but stream not started
    STATUS_STARTED,             // Camera opened and stream started
    STATUS_CONNECTING,          // Camera reconnecting after disconnect
    STATUS_CONNECT_SUCCESS      // Camera reconnect success
}LX_CAMERA_STATUS;

// Camera status callback info
typedef struct CameraStatus {
    LX_CAMERA_STATUS camera_state;
    unsigned long long status_time;
    void* reserve_data;           // Reserved extension
}CameraStatus;

// Frame callback
typedef void(*LX_FRAME_CALLBACK)(FrameInfo*, void*);

// Camera status callback
typedef void(*LX_CAMERA_STATUS_CALLBACK)(CameraStatus*, void*);

typedef struct ImuRawData {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gry_x;
    int16_t gry_y;
    int16_t gry_z;
    int32_t status;
    uint64_t sensor_time;
    int16_t resveredData[16];
}ImuRawData; // IMU raw data

typedef struct ImuData {
    float acc_x;
    float acc_y;
    float acc_z;
    float gry_x;
    float gry_y;
    float gry_z;
    unsigned long long sensor_timestamp;     // Sensor output timestamp
    unsigned long long recv_timestamp;       // Timestamp when frame data received
}ImuData; // IMU data after range conversion

typedef struct LxImuData
{
    LX_STATE frame_state;
    DcHandle handle;
    ImuRawData imu_raw_data;
    ImuData imu_data;
}LxImuData;

// IMU sensor data callback
typedef void(*LX_IMUDATA_CALLBACK)(LxImuData*, void*);

typedef enum LX_CAMERA_FEATURE
{
    /*int feature*/
    LX_INT_FIRST_EXPOSURE = 1001,   // Default high-integration exposure (us); first exposure in multi-integration
    LX_INT_SECOND_EXPOSURE = 1002,  // Default low-integration exposure (us); second exposure in multi-integration
    LX_INT_THIRD_EXPOSURE = 1003,   // Third exposure in multi-integration (us)
    LX_INT_FOURTH_EXPOSURE = 1004,  // Fourth exposure in multi-integration (us)
    LX_INT_GAIN = 1005,             // Gain (equivalent to exposure effect); adds noise, adjust to avoid overexposure

    LX_INT_MIN_DEPTH = 1011,        // Minimum depth
    LX_INT_MAX_DEPTH = 1012,        // Maximum depth
    LX_INT_MIN_AMPLITUDE = 1013,    // Minimum valid amplitude
    LX_INT_MAX_AMPLITUDE = 1014,    // Maximum valid amplitude
    LX_INT_CODE_MODE = 1016,        // Structured-light code mode; see LX_STRUCT_LIGHT_CODE_MODE
    LX_INT_WORK_MODE = 1018,        // Work mode; see LX_CAMERA_WORK_MODE
    LX_INT_LINK_SPEED = 1019,       // Negotiated NIC speed: 100=100Mbps, 1000=1Gbps (read-only)

    // 3D image parameters
    LX_INT_3D_IMAGE_WIDTH = 1021,       // 3D image width (changes after ROI/Binning/2D align; re-read)
    LX_INT_3D_IMAGE_HEIGHT = 1022,      // 3D image height (changes after ROI/Binning/2D align; re-read)
    LX_INT_3D_IMAGE_OFFSET_X = 1023,    // ROI horizontal offset; set WIDTH/HEIGHT/OFFSET_X/OFFSET_Y together (use DcSetROI)
    LX_INT_3D_IMAGE_OFFSET_Y = 1024,    // ROI vertical offset; set with DcSetROI
    LX_INT_3D_BINNING_MODE = 1025,      // 3D binning; see LX_BINNING_MODE
    LX_INT_3D_DEPTH_DATA_TYPE = 1026,   // Depth data type (read-only); see LX_DATA_TYPE
    LX_INT_3D_FREQ_MODE = 1027,         // 3D frequency mode; see LX_3D_FREQ_MODE

    // 3D amplitude image (same size as 3D depth)
    LX_INT_3D_AMPLITUDE_CHANNEL = 1031,  // Amplitude channels; shared with depth (mono=1, color=3)
    LX_INT_3D_AMPLITUDE_DATA_TYPE = 1035,// Amplitude data type (read-only); see LX_DATA_TYPE
    LX_INT_3D_AUTO_EXPOSURE_LEVEL = 1036,// 3D auto exposure level; exposure/gain cannot be set while enabled
    LX_INT_3D_AUTO_EXPOSURE_MAX = 1037,  // 3D auto exposure max
    LX_INT_3D_AUTO_EXPOSURE_MIN = 1038,  // 3D auto exposure min

    // 2D image
    LX_INT_2D_IMAGE_WIDTH = 1041,       // 2D image width (changes after ROI/Binning)
    LX_INT_2D_IMAGE_HEIGHT = 1042,      // 2D image height (changes after ROI/Binning)
    LX_INT_2D_IMAGE_OFFSET_X = 1043,    // 2D ROI horizontal offset; set with WIDTH/HEIGHT/OFFSET_X/OFFSET_Y (DcSetROI)
    LX_INT_2D_IMAGE_OFFSET_Y = 1044,    // 2D ROI vertical offset; set with DcSetROI
    LX_INT_2D_BINNING_MODE = 1045,      // 2D binning; see LX_BINNING_MODE
    LX_INT_2D_IMAGE_CHANNEL = 1046,     // 2D channels (mono=1, color=3)
    LX_INT_2D_IMAGE_DATA_TYPE = 1047,   // 2D data type (read-only); see LX_DATA_TYPE

    LX_INT_2D_MANUAL_EXPOSURE = 1051,   // 2D manual exposure value
    LX_INT_2D_MANUAL_GAIN = 1052,       // 2D manual gain value
    LX_INT_2D_AUTO_EXPOSURE_LEVEL = 1054,// 2D auto exposure level [0-100], 0=dark, 100=bright

    LX_INT_TOF_GLOBAL_OFFSET = 1061,    // TOF depth data offset
    LX_INT_3D_UNDISTORT_SCALE = 1062,   // 3D undistort scale [0,100]; higher keeps more data but adds black borders
    LX_INT_2D_UNDISTORT_SCALE = 1520,   // 2D undistort scale [0,100]; higher keeps more data but adds black borders
    LX_INT_ALGORITHM_MODE = 1065,       // Set built-in algorithm; some models; see LX_ALGORITHM_MODE
                                        // When LX_INT_CALCULATE_UP is enabled, built-in algorithms cannot be enabled
    LX_INT_MODBUS_ADDR = 1066,          // Modbus address; some models support MODBUS over serial
    LX_INT_HEART_TIME = 1067,           // Heartbeat interval with device (ms)
    LX_INT_GVSP_PACKET_SIZE = 1068,     // GVSP packet size (bytes)
    LX_INT_CALCULATE_UP = 1070,         // Allow 2D/3D algorithms to run on host/camera; may affect FPS/latency
                                        // 0-0xFFFF: four bits mean reserved, filter up, 2D up, 3D up
                                        // Built-in algorithms (LX_INT_ALGORITHM_MODE) cannot be enabled when set
    LX_INT_CAN_BAUD_RATE = 1072,         // CAN baud rate (bps)
    LX_INT_CAN_NODE_ID = 1073,           // CAN node ID
    LX_INT_CAN_PROTOCOL_TYPE = 1074,     // CAN protocol type; see LX_CAN_PROTOCOL_TYPE
    LX_INT_SAVE_PARAMS_GROUP = 1075,    // Save current configuration to parameter group
    LX_INT_LOAD_PARAMS_GROUP = 1076,     // Load parameter group by index
    LX_INT_RGBD_ALIGN_MODE = 1077,          // RGBD alignment mode; see LX_RGBD_ALIGN_MODE (replaces LX_BOOL_ENABLE_2D_TO_DEPTH)

    LX_INT_TRIGGER_MODE = 1069,         // Trigger mode; see LX_TRIGGER_MODE
    LX_INT_HARDWARE_TRIGGER_FILTER_TIME = 1085, // Hardware trigger filter time (us)
    LX_INT_TRIGGER_MIN_PERIOD_TIME = 1086,      // Trigger minimum period (us)
    LX_INT_TRIGGER_DELAY_TIME = 1087,           // Trigger delay (us)
    LX_INT_TRIGGER_FRAME_COUNT = 1088,          // Frames per trigger
    LX_INT_IO_WORK_MODE = 1530,           // GPIO output control mode; see LX_IO_WORK_MODE
    LX_INT_IO_OUTPUT_STATE = 1531,       // GPIO output user-control mode; see LX_IO_OUTPUT_STATE
    LX_INT_IO_HARDWARE_WORK_MODE = 1532, // GPIO hardware mode; see LX_IO_HARDWARE_MODE
    LX_INT_IO_INPUT_STATUS = 1533,       // IO input status (read-only)
    LX_INT_IO_OUTPUT_STATUS = 1534,      // IO output status (read-only)
    LX_INT_3D_GLARE_LEVEL = 1738,        // Glare level: 0 off, 1/2/3 levels

    LX_INT_FILTER_MODE = 1090,          // Filter mode; see LX_FILTER_MODE
    LX_INT_FILTER_SMOOTH_LEVEL = 1091,  // FILTER_NORMAL: smooth level [0,3], higher = stronger
    LX_INT_FILTER_NOISE_LEVEL = 1092,   // FILTER_NORMAL: noise level [0,3], higher = stronger
    LX_INT_FILTER_TIME_LEVEL = 1093,    // FILTER_NORMAL: temporal level [0,3], higher = stronger
    LX_INT_FILTER_DETECT_LOW_SIGNAL = 1094, // Low-signal measurement; may detect low SNR data, incl. long-range multi-cycle
    LX_INT_FILTER_FILL_LEVEL = 1095,    // FILTER_NORMAL: fill level [0,3], higher = stronger
    LX_INT_3D_FPS = 1510,             // Depth/amplitude FPS
    LX_INT_POWER_MODE = 1755,         // Camera power mode; see LX_POWER_MODE_TYPE (some models)

    /*float feature*/
    LX_FLOAT_FILTER_LEVEL = 2001,      // FILTER_SIMPLE: filter level [0,1], higher = stronger; 0 disables filtering
    LX_FLOAT_EST_OUT_EXPOSURE = 2002,  // Evaluate overexposure [0,1]; 1 means overexposed data invalid
    LX_FLOAT_LIGHT_INTENSITY = 2003,   // Light intensity [0,1], some models
    LX_FLOAT_3D_DEPTH_FPS = 2004,      // Current depth FPS (read-only)
    LX_FLOAT_3D_AMPLITUDE_FPS = 2005,  // Current amplitude FPS (read-only)
    LX_FLOAT_2D_IMAGE_FPS = 2006,      // Current RGB FPS (read-only)
    LX_FLOAT_DEVICE_TEMPERATURE = 2007,// Current camera temperature (read-only)
    LX_FLOAT_CONFIDENCE = 2008,        // Confidence [0-1]; lower keeps more data

    /*bool feature*/
    LX_BOOL_CONNECT_STATE = 3001,           // Current connection status
    LX_BOOL_ENABLE_3D_DEPTH_STREAM = 3002,  // Enable/disable depth stream (some models)
    LX_BOOL_ENABLE_3D_AMP_STREAM = 3003,    // Enable/disable amplitude stream (some models)
    LX_BOOL_ENABLE_3D_AUTO_EXPOSURE = 3006, // Enable 3D auto exposure
    LX_BOOL_ENABLE_3D_UNDISTORT = 3007,     // Enable 3D undistort
    LX_BOOL_ENABLE_ANTI_FLICKER = 3008,     // Enable anti-flicker; LED lighting may cause ripples (some models)

    LX_BOOL_ENABLE_2D_STREAM = 3011,        // Enable/disable 2D stream
    LX_BOOL_ENABLE_2D_AUTO_EXPOSURE = 3012, // Enable 2D auto exposure
    LX_BOOL_ENABLE_2D_UNDISTORT = 3015,     // Enable 2D undistort
    LX_BOOL_ENABLE_2D_TO_DEPTH = 3016,      // Enable 2D/3D alignment; replaced by LX_INT_RGBD_ALIGN_MODE
    LX_BOOL_ENABLE_BACKGROUND_AMP = 3017,   // Enable background amplitude
    LX_BOOL_ENABLE_MULTI_MACHINE = 3018,    // Enable multi-camera mode; mitigates TOF interference
    LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR = 3019,  // Enable HDR (multi-exposure high dynamic range)
    LX_BOOL_ENABLE_SYNC_FRAME = 3020,          // Enable forced frame sync; default favors real-time; enable for RGBD sync
    LX_BOOL_ENABLE_TERM_RESISTOR = 3021,      // Enable termination resistor

    /*string feature*/
    LX_STRING_DEVICE_VERSION = 4001,        // Device version
    LX_STRING_FIRMWARE_NAME = 4003,         // Firmware filename for upgrade; some models require reopen
    LX_STRING_FILTER_PARAMS = 4004,         // Filter parameters (JSON string)
    LX_STRING_ALGORITHM_PARAMS = 4005,      // Built-in algorithm params for current LX_ALGORITHM_MODE (JSON string)
    LX_STRING_ALGORITHM_VERSION = 4006,     // Built-in algorithm version for current LX_ALGORITHM_MODE
    LX_STRING_DEVICE_OS_VERSION = 4007,     // Device OS image version
    LX_STRING_IMPORT_PARAMS_FROM_FILE = 4008,   // Import params from local file to camera
    LX_STRING_EXPORT_PARAMS_TO_FILE = 4009,      // Export current params to local file
    LX_STRING_CUSTOM_ID = 4010,             // Customer-defined device ID

    /*command feature*/
    LX_CMD_GET_NEW_FRAME = 5001,    // Manually update latest data; call before getting data pointers
                                    // With multiple streams, any stream update returns OK unless LX_BOOL_ENABLE_SYNC_FRAME is true
                                    // Prefer callback DcRegisterFrameCallback; then this command is not needed
    LX_CMD_RETURN_VERSION = 5002,   // Roll back to previous version
    LX_CMD_RESTART_DEVICE = 5003,   // Restart camera; some models require reopen
    LX_CMD_WHITE_BALANCE = 5004,    // Auto white balance
    LX_CMD_RESET_PARAM = 5007,      // Reset to default parameters
    LX_CMD_SOFTWARE_TRIGGER = 5008, // Software trigger command

    /*ptr feature*/
    LX_PTR_2D_IMAGE_DATA = 6001, // Use LX_PTR_FRAME_DATA; get 2D image pointer (length by size/channels/type)
    LX_PTR_3D_AMP_DATA = 6002,   // Use LX_PTR_FRAME_DATA; get 3D amplitude pointer (length by size/channels/type)
    LX_PTR_3D_DEPTH_DATA = 6003, // Use LX_PTR_FRAME_DATA; get 3D depth pointer (length by size/channels/type)
    LX_PTR_XYZ_DATA = 6004,      // Get point cloud pointer, float* of xyz triplets
                                 // Length = LX_INT_3D_IMAGE_WIDTH * LX_INT_3D_IMAGE_HEIGHT * sizeof(float) * 3
    LX_PTR_2D_INTRIC_PARAM = 6005,  // Get 2D intrinsics, float* length 9*sizeof(float) (fx,fy,cx,cy,k1,k2,p1,p2,k3)
    LX_PTR_3D_INTRIC_PARAM = 6006, // Get 3D intrinsics, float* length 9*sizeof(float) (fx,fy,cx,cy,k1,k2,p1,p2,k3)
    LX_PTR_3D_EXTRIC_PARAM = 6007, // Get 3D extrinsics, float* length 12*sizeof(float) (first 9 rotation, last 3 translation)
    LX_PTR_ALGORITHM_OUTPUT = 6008, // Use LX_PTR_FRAME_DATA; get built-in algorithm output
                                    // MODE_AVOID_OBSTACLE -> LxAvoidanceOutput*
                                    // MODE_PALLET_LOCATE -> LxPalletPose*
                                    // MODE_VISION_LOCATION -> LxLocation*
                                    // MODE_AVOID_OBSTACLE2 -> LxAvoidanceOutputN*
    LX_PTR_FRAME_DATA = 6009,       // Get complete frame data; see FrameInfo
    LX_PTR_2D_NEW_INTRIC_PARAM = 6010,  // Get new 2D intrinsics, float* length 18*sizeof(float) (fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,tau_x,tau_y)
    LX_PTR_3D_NEW_INTRIC_PARAM = 6011, // Get new 3D intrinsics, float* length 18*sizeof(float) (fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,tau_x,tau_y)
}LX_CAMERA_FEATURE;

#endif
