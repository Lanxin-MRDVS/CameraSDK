

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;


namespace lxApplication
{

    public unsafe class LxApplication
    {
        
        //地板平面，满足ax+by+cz+d=0
        public struct LxGroundPlane
        {
            public float a, b, c, d;
        }

        //2D点
        public struct LxPoint2d
        {
            public float x, y;
        }

        //!3D点
        public struct LxPoint3dWithRGB
        {
            public float x, y, z;
            public byte r, g, b;   //uint8_t
        }

        //!6自由度速度
        public struct LxVelocity
        {
            public float[] linear;
            public float[] angular;
            public LxVelocity()
            {
                linear = new float[3];
                angular = new float[3];
            }

        }

        public struct LxPose
        {
            public float[] R;   //!< 旋转矩阵
            public float[] T;   //!< 平移向量
            public LxPose()
            {
                R = new float[9];
                T = new float[3];
            }
            public ulong timestamp; //!< unix时间戳

        }

        //障碍物信息
        public struct LxObstacleBox
        {
            public LxPose pose;                 //!< 障碍物 Box 中心和旋转矩阵
            public float width, height, depth;    //!< 障碍物 Box 宽、高、深
            public float[] center;              //!< 障碍物质心
            public LxObstacleBox()
            {
                center = new float[3];
            }
            public byte type;                    //!< 障碍物语义，暂未实现
            public ulong id;                     //!< 障碍物序号，暂未实现
            LxVelocity velocity;                 //!< 障碍物速度，暂未实现
        }

        public struct LxObstacleBoxN
        {
            public LxPose pose;                  //!< 障碍物 Box 中心和旋转矩阵
            public float width, height, depth;     //!< 障碍物 Box 宽、高、深   
            public float[] center;               //!< 障碍物质心
            public char[] type_name;             //!< 障碍物语义类别
            public char[] reserved;              //!< 预留位
            public LxObstacleBoxN()
            {
                center = new float[3];
                type_name = new char[512];
                reserved = new char[1024];
            }
            public byte type_idx;                //!< 障碍物语义 idx
            public ulong id;                     //!< 障碍物 ID
            public ulong prev_id;                //!< 障碍物 Prev_ID
            public float box_2d_x_min;           //!< 障碍物在2D图上左上角点的X坐标与图像宽度的比值
            public float box_2d_y_min;           //!< 障碍物在2D图上左上角点的Y坐标与图像高度的比值
            public float box_2d_x_max;           //!< 障碍物在2D图上右下角点的X坐标与图像宽度的比值
            public float box_2d_y_max;           //!< 障碍物在2D图上右下角点的Y坐标与图像高度的比值
            public LxVelocity velocity;          //!< 障碍物速度，暂未实现
            public byte type_name_len;           //!< 障碍物语义类别长度

        }

        public enum LxAvState
        {
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
        };

        //! 避障输出结构体
        //×××××××××××××××××××
        public struct LxAvoidanceOutput
        {
            public LxAvState state;                 //!< 返回状态
            public LxGroundPlane  groundPlane;       //!< 检测到的距离最近的地面
            public byte number_3d;                  //!< 输出点个数
            public LxPoint3dWithRGB * cloud_output = null;  //!< 输出点云（已过滤地面与噪声）
            public byte number_box;                 //!< 障碍物个数
        };

        //! V2避障输出结构体
        //×××××××××××××××××××      
        public struct LxAvoidanceOutputN
        {
            public LxAvState state;                 //!< 返回状态
            public LxGroundPlane groundPlane;       //!< 检测到的距离最近的地面
            public byte number_3d;                  //!< 输出点个数
            public LxPoint3dWithRGB * cloud_output = null;   //!< 输出点云（已过滤地面与噪声）
            public byte number_box;                 //!< 障碍物个数     
            public LxAvoidanceOutputN()
            {
                
            }
        }

        //! 托盘定位结构体
        //@Update Since 0.1.2: add return value and a float-array
        // extents: 可扩展数据，总长度128
        // 当前数组置0，待扩展// x: 托盘中心距离相机光心距离，负值（叉车插臂方向为后退方向）
        // y: 托盘中心距离相机光心右侧偏移距离，一般在（-500, 500)范围
        // yaw: 托盘倾斜角度，水平为0，单位为角度
        // extents: 扩展信息(int类型，长度为8192)
        public struct LxPalletPose
        {
            public int return_val;
            public float x, y, yaw;
            public int[] extents;
            public LxPalletPose()
            {
                extents = new int[8192];
            }

        }

        //重定位pose结构体
        public struct LxRelocPOSE
        {
            public double x,y,theta;
        }

        //odom结构体
        public struct LxOdomData
        {
            public ulong timestamp;
            public double x,y,theta;
        }

        //激光数据结构体
        //×××××××××××××××××××
        public struct LxLaser
        {
            public ulong timestamp;       //时间戳
            public float time_increment;    //扫描时间间隔
            public float angle_min;        //开始扫描的角度(角度)
            public float angle_max;        //结束扫描的角度(角度)
            public float angle_increment;  //每一次扫描增加的角度(角度)
            public float range_min;        //距离最小值(m)
            public float range_max;        //距离最大值(m)
            public float reserved;         //预留字段
            public float range_size;       //距离数组大小
            public float[] ranges;          //距离数组
        };

        //激光pose结构体
        public struct LxLaserPose
        {
            public ulong  timestamp;
            public double x, y,theta;
        };

        //! 定位算法输出结果结构体
        public struct LxLocation
        {
            public ulong timestamp;  //时间戳
            public byte status;     //算法返回值:正在加载地图，加载地图错误，正在重定位，重定位错误，初始化错误，参数异常，图像异常，识别异常等
            public float x, y, theta;
            public byte[] extents; //自定义内容，扩展信息
            public LxLocation() 
            {
                extents= new byte[8192];
            }
        };

    }

}
