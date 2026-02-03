
#ifndef _LX_CAMERA_APPLICATION_H_
#define _LX_CAMERA_APPLICATION_H_

#include <stdint.h>
//! Ground plane satisfying ax + by + cz + d = 0
typedef struct {
    float a, b, c, d;
} LxGroundPlane;

// 2D point
typedef struct {
    float x, y;
} LxPoint2d;

//! 3D point
typedef struct {
    float x, y, z;
    uint8_t r, g, b;
} LxPoint3dWithRGB;

//! 6-DOF velocity
typedef struct {
    float linear[3];   //!< Linear velocity
    float angular[3];  //!< Angular velocity
} LxVelocity;

typedef float LxRotation[9];     //!< Rotation matrix
typedef float LxTranslation[3];  //!< Translation vector
typedef struct {
    LxRotation R;        //!< Rotation matrix
    LxTranslation T;     //!< Translation vector
    uint64_t timestamp;  //!< Unix timestamp
} LxPose;

//! Obstacle info
typedef struct {
    LxPose pose;                 //!< Obstacle box center and rotation matrix
    float width, height, depth;  //!< Obstacle box width/height/depth
    LxTranslation center;        //!< Obstacle centroid
    int32_t type;                //!< Obstacle semantic type (not implemented)
    int64_t id;                  //!< Obstacle index (not implemented)
    LxVelocity velocity;         //!< Obstacle velocity (not implemented)
} LxObstacleBox;

typedef struct {
    LxPose pose;                 //!< Obstacle box center and rotation matrix
    float width, height, depth;  //!< Obstacle box width/height/depth
    LxTranslation center;        //!< Obstacle centroid
    int32_t type_idx;            //!< Obstacle semantic idx
    int64_t id;                  //!< Obstacle ID
    int64_t prev_id;             //!< Obstacle prev ID
    float box_2d_x_min;          //!< X ratio of top-left corner in 2D image to image width
    float box_2d_y_min;          //!< Y ratio of top-left corner in 2D image to image height
    float box_2d_x_max;          //!< X ratio of bottom-right corner in 2D image to image width
    float box_2d_y_max;          //!< Y ratio of bottom-right corner in 2D image to image height
    LxVelocity velocity;         //!< Obstacle velocity (not implemented)
    int32_t type_name_len;       //!< Obstacle semantic label length
    char type_name[512];         //!< Obstacle semantic label
    char reserved[1024];         //!< Reserved
} LxObstacleBoxN;


typedef enum {
    //! Success
    LxAvSuccess = 0,
    //! Ground prior not set; call LxAvSetGroundPriorPlane
    LxAvGroundPriorNotSet = -1,
    //! Input not set; call LxAvSetInput_1 or LxAvSetInput_2
    LxAvInputNotSet = -2,
    //! No points in the configured range; call LxAvSetRange to change it
    LxAvNoPointInRange = -3,
    //! Point cloud is empty after ground filtering
    LxAvNoPointAfterFilterGround = -4,
    /*! Point cloud is empty after radius filtering.
       Use LxAvSetNoiseSituation to set appropriate parameters. */
    LxAvNoPointAfterRadiusFilter = -5,
    //! Semantic function enabled, but no segmentation result
    LxAvObstaclesNotSegmented = -6,
    //! Undefined error
    LxAvUndefinedError = -99,
} LxAvState;

//! Obstacle avoidance output struct
typedef struct {
  LxAvState state;                 //!< Return status
  LxGroundPlane groundPlane;       //!< Nearest detected ground plane
  uint32_t number_3d;              //!< Number of output points
  LxPoint3dWithRGB* cloud_output;  //!< Output point cloud (ground/noise filtered)
  uint32_t number_box;             //!< Number of obstacles
  LxObstacleBox* obstacleBoxs;     //!< Obstacle boxes
} LxAvoidanceOutput;

//! V2 obstacle avoidance output struct
typedef struct {
    LxAvState state;                 //!< Return status
    LxGroundPlane groundPlane;       //!< Nearest detected ground plane
    uint32_t number_3d;              //!< Number of output points
    LxPoint3dWithRGB* cloud_output;  //!< Output point cloud (ground/noise filtered)
    uint32_t number_box;             //!< Number of obstacles
    LxObstacleBoxN* obstacleBoxs;     //!< Obstacle boxes
    char resveredData[4096];          //!< Reserved space
} LxAvoidanceOutputN;

//! Pallet localization struct
//@Update Since 0.1.2: add return value and a float-array
// extents: extensible data, total length 128
// Set the current array to 0 for future extension
// x: distance from pallet center to camera optical center (negative means forks insert backwards)
// y: right offset from camera optical center to pallet center (typically in -500..500)
// yaw: pallet tilt angle in degrees (0 = level)
// extents: extended info (int type, length 8192)
typedef struct {
    int return_val;
    float x, y, yaw;
    int extents[8192];
} LxPalletPose;


// Relocation pose struct
typedef struct {
    double x, y, theta;
}LxRelocPose;

// Odometry struct
typedef struct {
    int64_t timestamp;
    double x, y, theta;
}LxOdomData;

// Laser data struct
typedef struct {
    int64_t timestamp;       // Timestamp
    float time_increment;    // Scan time increment
    float angle_min;        // Start angle (deg)
    float angle_max;        // End angle (deg)
    float angle_increment;  // Angle increment per scan (deg)
    float range_min;        // Min range (m)
    float range_max;        // Max range (m)
    float reserved;         // Reserved
    float range_size;       // Range array size
    float* ranges;          // Range array
}LxLaser;

// Laser pose struct
typedef struct {
    int64_t timestamp;
    double x, y, theta;
}LxLaserPose;


//! Localization algorithm output struct
typedef struct {
    int64_t timestamp;  // Timestamp
    int32_t status;     // Status: loading map, map load error, relocating, relocation error, init error, parameter error, image error, recognition error, etc.
    float x, y, theta;
    int32_t extents[8192]; // Custom content, extended info
}LxLocation;

#endif
