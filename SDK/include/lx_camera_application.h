
#ifndef _LX_CAMERA_APPLICATION_H_
#define _LX_CAMERA_APPLICATION_H_

#include <stdint.h>
//! 地面平面，满足 ax + by + cz + d = 0 | Ground plane satisfying ax + by + cz + d = 0
typedef struct {
    float a, b, c, d;
} LxGroundPlane;

//2D 点 | 2D point
typedef struct {
    float x, y;
} LxPoint2d;

//! 3D 点 | 3D point
typedef struct {
    float x, y, z;
    uint8_t r, g, b;
} LxPoint3dWithRGB;

//! 6自由度速度 | Six-degree-of-freedom velocity
typedef struct {
    float linear[3];   //!< 线速度 | Linear velocity
    float angular[3];  //!< 角速度 | Angular velocity
} LxVelocity;

typedef float LxRotation[9];     //!< 旋转矩阵 | Rotation matrix
typedef float LxTranslation[3];  //!< 平移向量 | Translation vector
typedef struct {
    LxRotation R;        //!< 旋转矩阵 | Rotation matrix
    LxTranslation T;     //!< 平移向量 | Translation vector
    uint64_t timestamp;  //!< unix时间戳 | Unix timestamp
} LxPose;

//! 障碍物信息 | Obstacle information
typedef struct {
    LxPose pose;                 //!< 障碍物 Box 中心和旋转矩阵 | Center and rotation matrix of the obstacle bounding box
    float width, height, depth;  //!< 障碍物 Box 宽、高、深 | Width, height, and depth of the obstacle bounding box
    LxTranslation center;        //!< 障碍物质心 | Obstacle centroid
    int32_t type;                //!< 障碍物语义，暂未实现 | Obstacle semantic class; not yet implemented
    int64_t id;                  //!< 障碍物序号，暂未实现 | Obstacle index; not yet implemented
    LxVelocity velocity;         //!< 障碍物速度，暂未实现 | Obstacle velocity; not yet implemented
} LxObstacleBox;

typedef struct {
    LxPose pose;                 //!< 障碍物 Box 中心和旋转矩阵 | Center and rotation matrix of the obstacle bounding box
    float width, height, depth;  //!< 障碍物 Box 宽、高、深 | Width, height, and depth of the obstacle bounding box
    LxTranslation center;        //!< 障碍物质心 | Obstacle centroid
    int32_t type_idx;            //!< 障碍物语义 idx | Obstacle semantic class index
    int64_t id;                  //!< 障碍物 ID | Obstacle ID
    int64_t prev_id;             //!< 障碍物 Prev_ID | Previous obstacle ID
    float box_2d_x_min;          //!< 障碍物在2D图上左上角点的X坐标与图像宽度的比值
                                 // X coordinate of the obstacle's top-left corner in the 2D image, divided by image
                                 // width
    float box_2d_y_min;          //!< 障碍物在2D图上左上角点的Y坐标与图像高度的比值
                                 // Y coordinate of the obstacle's top-left corner in the 2D image, divided by image
                                 // height
    float box_2d_x_max;          //!< 障碍物在2D图上右下角点的X坐标与图像宽度的比值
                                 // X coordinate of the obstacle's bottom-right corner in the 2D image, divided by image
                                 // width
    float box_2d_y_max;          //!< 障碍物在2D图上右下角点的Y坐标与图像高度的比值
                                 // Y coordinate of the obstacle's bottom-right corner in the 2D image, divided by image
                                 // height
    LxVelocity velocity;         //!< 障碍物速度，暂未实现 | Obstacle velocity; not yet implemented
    int32_t type_name_len;       //!< 障碍物语义类别长度 | Length of the obstacle semantic class name
    char type_name[512];         //!< 障碍物语义类别 | Obstacle semantic class name
    char reserved[1024];         //!< 预留位 | Reserved
} LxObstacleBoxN;


typedef enum {
    //! 执行成功 | Operation successful
    LxAvSuccess = 0,
    //! 未设置地面先验，请调用 LxAvSetGroundPriorPlane | Ground plane prior not set; call LxAvSetGroundPriorPlane
    LxAvGroundPriorNotSet = -1,
    //! 未设置输入，请调用 LxAvSetInput_1 或 LxAvSetInput_2 | Input not set; call LxAvSetInput_1 or LxAvSetInput_2
    LxAvInputNotSet = -2,
    //! 设置的范围内无点云，如需更改检测范围，请调用 LxAvSetRange
    // No points within the configured range; call LxAvSetRange to change the detection range
    LxAvNoPointInRange = -3,
    //! 过滤地面后点云为空 | Point cloud is empty after ground removal
    LxAvNoPointAfterFilterGround = -4,
    /*! 半径滤波后点云为空，通过调用 LxAvSetNoiseSituation 来设置合适的
       参数可能会改善这个问题 
     * Point cloud is empty after radius filtering. Calling LxAvSetNoiseSituation to set suitable
     * parameters may help resolve this issue.
     */
    LxAvNoPointAfterRadiusFilter = -5,
    //! 语义功能已开启，但未得到分割结果 | Semantic processing is enabled, but no segmentation result is available
    LxAvObstaclesNotSegmented = -6,
    //! 未定义的错误 | Undefined error
    LxAvUndefinedError = -99,
} LxAvState;

//! 避障输出结构体 | Obstacle avoidance output structure
typedef struct {
  LxAvState state;                 //!< 返回状态 | Return status
  LxGroundPlane groundPlane;       //!< 检测到的距离最近的地面 | Nearest detected ground plane
  uint32_t number_3d;              //!< 输出点个数 | Number of output points
  LxPoint3dWithRGB* cloud_output;  //!< 输出点云（已过滤地面与噪声） | Output point cloud with ground and noise removed
  uint32_t number_box;             //!< 障碍物个数 | Number of obstacles
  LxObstacleBox* obstacleBoxs;     //!< 障碍物 Box | Obstacle bounding boxes
} LxAvoidanceOutput;

//! V2避障输出结构体 | V2 obstacle avoidance output structure
typedef struct {
    LxAvState state;                 //!< 返回状态 | Return status
    LxGroundPlane groundPlane;       //!< 检测到的距离最近的地面 | Nearest detected ground plane
    uint32_t number_3d;              //!< 输出点个数 | Number of output points
    LxPoint3dWithRGB* cloud_output;  //!< 输出点云（已过滤地面与噪声） | Output point cloud with ground and noise removed
    uint32_t number_box;             //!< 障碍物个数 | Number of obstacles
    LxObstacleBoxN* obstacleBoxs;     //!< 障碍物 Box | Obstacle bounding boxes
    char resveredData[4096];          //!< 预留位置 | Reserved
} LxAvoidanceOutputN;

//! 托盘定位结构体 | Pallet localization structure
//@Update Since 0.1.2: add return value and a float-array
// extents: 可扩展数据，总长度128 | extents: Extensible data, total length 128
// 当前数组置0，待扩展// x: 托盘中心距离相机光心距离，负值（叉车插臂方向为后退方向）
// The array is currently zero-filled for future extensions. x: Distance from the camera optical center to the pallet
// center; negative (the direction of fork insertion is the reverse direction).
// y: 托盘中心距离相机光心右侧偏移距离，一般在（-500, 500)范围
// y: Rightward offset of the pallet center from the camera optical center, typically within (-500, 500)
// yaw: 托盘倾斜角度，水平为0，单位为角度 | yaw: Pallet tilt angle in degrees; 0 when horizontal
/**
     * extents扩展数据定义： | extents extension data definitions:
     * 0 - 托盘高度（以相机光心为远点，向下为正） | 0 - Pallet height (camera optical center as the origin; positive downward)
     * 1 - 托盘长度（左腿中心到右腿中心的距离） | 1 - Pallet length (distance between the centers of the left and right legs)
     * 2 - 托盘腿的数量 | 2 - Number of pallet legs
     * 3 - 托盘腿1宽度 | 3 - Width of pallet leg 1
     * 4 - 托盘腿2宽度 | 4 - Width of pallet leg 2
     * 5 - 托盘腿2宽度 | 5 - Width of pallet leg 2
     * 6 - ----
     * 7 - 托盘孔宽度（左） | 7 - Pallet opening width (left)
     * 8 - 托盘孔宽度（右） | 8 - Pallet opening width (right)
     * 9 - ----
     * |
     * | @since 2024.06 (v0.7.6)
     * 10 - 托盘总数n（有效，从上到下，最大4） | 10 - Total number of valid pallets n (ordered top to bottom; maximum 4)
     * 11 12 13 14  p1: <x, y, theta * 1000, height>
     * 15 16 17 18  p2: <x, y, theta * 1000, height>
     * 19 20 21 22  p3: <x, y, theta * 1000, height>
     * 23 24 25 26  p4: <x, y, theta * 1000, height>
     * |
     * 27 28 <左侧腿 x, y> | 27 28 <left leg x, y>
     * 29 30 <右侧腿 x, y> | 29 30 <right leg x, y>
     * |
     * 32 [added @2024.01.31] 是否有货 （0 or 1） | 32 [added @2024.01.31] Whether a load is present (0 or 1)
     * |
     * 33 - 结果图片Width (256) | 33 - Result image width (256)
     * 34 - 结果图片Height (32) | 34 - Result image height (32)
     * 35 - 结果托盘channel (3) | 35 - Pallet result channel count (3)
     * 36 -- ptr [256*32*3] / 4
*/
typedef struct {
    int return_val;
    float x, y, yaw;
    int extents[8192];
} LxPalletPose;


//重定位pose结构体 | Relocalization pose structure
typedef struct {
    double x, y, theta;
}LxRelocPose;

//odom结构体 | Odometry structure
typedef struct {
    int64_t timestamp;
    double x, y, theta;
}LxOdomData;

//激光数据结构体 | Laser scan data structure
typedef struct {
    int64_t timestamp;       //时间戳 | Timestamp
    float time_increment;    //扫描时间间隔 | Scan time interval
    float angle_min;        //开始扫描的角度(角度) | Start angle of the scan (degrees)
    float angle_max;        //结束扫描的角度(角度) | End angle of the scan (degrees)
    float angle_increment;  //每一次扫描增加的角度(角度) | Angular increment per scan step (degrees)
    float range_min;        //距离最小值(m) | Minimum range (m)
    float range_max;        //距离最大值(m) | Maximum range (m)
    float reserved;         //预留字段 | Reserved field
    float range_size;       //距离数组大小 | Size of the range array
    float* ranges;          //距离数组 | Range array
}LxLaser;

//激光pose结构体 | Laser pose structure
typedef struct {
    int64_t timestamp;
    double x, y, theta;
}LxLaserPose;


//! 定位算法输出结果结构体 | Localization algorithm output structure
typedef struct {
    int64_t timestamp;  //时间戳 | Timestamp
    int32_t status;     //算法返回值:正在加载地图，加载地图错误，正在重定位，重定位错误，初始化错误，参数异常，图像异常，识别异常等
                        // Algorithm return status: loading map, map loading error, relocalizing, relocalization error,
                        // initialization error, invalid parameters, image error, recognition error, etc.
    float x, y, theta;
    int32_t extents[8192]; //自定义内容，扩展信息 | Custom content and extension information
}LxLocation;

#endif
