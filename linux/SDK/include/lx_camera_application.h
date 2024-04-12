
#ifndef _LX_CAMERA_APPLICATION_H_
#define _LX_CAMERA_APPLICATION_H_

#include <stdint.h>
//! 地面平面，满足 ax + by + cz + d = 0
typedef struct {
    float a, b, c, d;
} LxGroundPlane;

//2D 点
typedef struct {
    float x, y;
} LxPoint2d;

//! 3D 点
typedef struct {
    float x, y, z;
    uint8_t r, g, b;
} LxPoint3dWithRGB;

//! 6自由度速度
typedef struct {
    float linear[3];   //!< 线速度
    float angular[3];  //!< 角速度
} LxVelocity;

typedef float LxRotation[9];     //!< 旋转矩阵
typedef float LxTranslation[3];  //!< 平移向量
typedef struct {
    LxRotation R;        //!< 旋转矩阵
    LxTranslation T;     //!< 平移向量
    uint64_t timestamp;  //!< unix时间戳
} LxPose;

//! 障碍物信息
typedef struct {
    LxPose pose;                 //!< 障碍物 Box 中心和旋转矩阵
    float width, height, depth;  //!< 障碍物 Box 宽、高、深
    LxTranslation center;        //!< 障碍物质心
    int32_t type;                //!< 障碍物语义，暂未实现
    int64_t id;                  //!< 障碍物序号，暂未实现
    LxVelocity velocity;         //!< 障碍物速度，暂未实现
} LxObstacleBox;

typedef struct {
    LxPose pose;                 //!< 障碍物 Box 中心和旋转矩阵
    float width, height, depth;  //!< 障碍物 Box 宽、高、深
    LxTranslation center;        //!< 障碍物质心
    int32_t type_idx;            //!< 障碍物语义 idx
    int64_t id;                  //!< 障碍物 ID
    int64_t prev_id;             //!< 障碍物 Prev_ID
    float box_2d_x_min;          //!< 障碍物在2D图上左上角点的X坐标与图像宽度的比值
    float box_2d_y_min;          //!< 障碍物在2D图上左上角点的Y坐标与图像高度的比值
    float box_2d_x_max;          //!< 障碍物在2D图上右下角点的X坐标与图像宽度的比值
    float box_2d_y_max;          //!< 障碍物在2D图上右下角点的Y坐标与图像高度的比值
    LxVelocity velocity;         //!< 障碍物速度，暂未实现
    int32_t type_name_len;       //!< 障碍物语义类别长度
    char type_name[512];         //!< 障碍物语义类别
    char reserved[1024];         //!< 预留位
} LxObstacleBoxN;


typedef enum {
    //! 执行成功
    LxAvSuccess = 0,
    //! 未设置地面先验，请调用 LxAvSetGroundPriorPlane
    LxAvGroundPriorNotSet = -1,
    //! 未设置输入，请调用 LxAvSetInput_1 或 LxAvSetInput_2
    LxAvInputNotSet = -2,
    //! 设置的范围内无点云，如需更改检测范围，请调用 LxAvSetRange
    LxAvNoPointInRange = -3,
    //! 过滤地面后点云为空
    LxAvNoPointAfterFilterGround = -4,
    /*! 半径滤波后点云为空，通过调用 LxAvSetNoiseSituation 来设置合适的
       参数可能会改善这个问题 */
    LxAvNoPointAfterRadiusFilter = -5,
    //! 语义功能已开启，但未得到分割结果
    LxAvObstaclesNotSegmented = -6,
    //! 未定义的错误
    LxAvUndefinedError = -99,
} LxAvState;

//! 避障输出结构体
typedef struct {
  LxAvState state;                 //!< 返回状态
  LxGroundPlane groundPlane;       //!< 检测到的距离最近的地面
  uint32_t number_3d;              //!< 输出点个数
  LxPoint3dWithRGB* cloud_output;  //!< 输出点云（已过滤地面与噪声）
  uint32_t number_box;             //!< 障碍物个数
  LxObstacleBox* obstacleBoxs;     //!< 障碍物 Box
} LxAvoidanceOutput;

//! V2避障输出结构体
typedef struct {
    LxAvState state;                 //!< 返回状态
    LxGroundPlane groundPlane;       //!< 检测到的距离最近的地面
    uint32_t number_3d;              //!< 输出点个数
    LxPoint3dWithRGB* cloud_output;  //!< 输出点云（已过滤地面与噪声）
    uint32_t number_box;             //!< 障碍物个数
    LxObstacleBoxN* obstacleBoxs;     //!< 障碍物 Box
    char resveredData[4096];          //!< 预留位置
} LxAvoidanceOutputN;

//! 托盘定位结构体
//@Update Since 0.1.2: add return value and a float-array
// extents: 可扩展数据，总长度128
// 当前数组置0，待扩展// x: 托盘中心距离相机光心距离，负值（叉车插臂方向为后退方向）
// y: 托盘中心距离相机光心右侧偏移距离，一般在（-500, 500)范围
// yaw: 托盘倾斜角度，水平为0，单位为角度
// extents: 扩展信息(int类型，长度为8192)
typedef struct {
    int return_val;
    float x, y, yaw;
    int extents[8192];
} LxPalletPose;


//重定位pose结构体
typedef struct {
    double x, y, theta;
}LxRelocPose;

//odom结构体
typedef struct {
    int64_t timestamp;
    double x, y, theta;
}LxOdomData;

//激光数据结构体
typedef struct {
    int64_t timestamp;       //时间戳
    float time_increment;    //扫描时间间隔
    float angle_min;        //开始扫描的角度(角度)
    float angle_max;        //结束扫描的角度(角度)
    float angle_increment;  //每一次扫描增加的角度(角度)
    float range_min;        //距离最小值(m)
    float range_max;        //距离最大值(m)
    float reserved;         //预留字段
    float range_size;       //距离数组大小
    float* ranges;          //距离数组
}LxLaser;

//激光pose结构体
typedef struct {
    int64_t timestamp;
    double x, y, theta;
}LxLaserPose;


//! 定位算法输出结果结构体
typedef struct {
    int64_t timestamp;  //时间戳
    int32_t status;     //算法返回值:正在加载地图，加载地图错误，正在重定位，重定位错误，初始化错误，参数异常，图像异常，识别异常等
    float x, y, theta;
    int32_t extents[8192]; //自定义内容，扩展信息
}LxLocation;

#endif
