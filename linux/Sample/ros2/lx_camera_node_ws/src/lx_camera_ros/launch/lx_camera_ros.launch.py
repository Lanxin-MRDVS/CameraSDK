from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # 是否开启rviz显示
  enable_rviz =  LaunchConfiguration('enable_rviz')
  
  return LaunchDescription([
    # 声明参数，可以通过命令行传递参数值
    DeclareLaunchArgument('enable_rviz', default_value='false', description='Whether to launch rviz2'),

    # 启动节点lx_camera_node
    Node(
        package="lx_camera_ros",
        executable="lx_camera_node",
        namespace="lx_camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            # <!-- base config:gpu, jpeg decode,ip、log -->
            {"enable_gpu": 0},
            {"ip": ""},
            {"log_level": 1},
            {"log_path": "./log/"},

            # <!-- stream -->
            {"is_depth": 1},
            # <!-- pointcloud, 0:close, 1:xyz, with rgb when RGBD_ALIGN_MODE enabled
		    #   2:xyz with intensity(enable 3D_AMP_STREAM) and timestamp -->
            {"is_xyz": 1},
            {"LX_BOOL_ENABLE_3D_AMP_STREAM": 0},
            {"LX_BOOL_ENABLE_2D_STREAM": 1},
            {"LX_BOOL_ENABLE_IMU": 0},
            #<!-- 0:mm,  1:m-->
            {"LX_INT_XYZ_UNIT": 0},

            # <!-- 2D配置，如需生效，取消其注释 -->
            #{"LX_INT_RGBD_ALIGN_MODE": 1},
            #{"LX_INT_ALGORITHM_MODE": 0},
            #{"LX_INT_WORK_MODE": 0},
            #{"LX_INT_3D_FPS": 20},
            #{"LX_BOOL_ENABLE_2D_UNDISTORT": 1},
            #{"LX_INT_2D_UNDISTORT_SCALE": 1},
            #{"LX_INT_2D_BINNING_MODE": 1},
            #{"LX_BOOL_ENABLE_3D_UNDISTORT": 1},
            #{"LX_INT_3D_UNDISTORT_SCALE": 1},
            #{"LX_INT_3D_BINNING_MODE": 1},
            #{"LX_BOOL_ENABLE_MULTI_MACHINE": 0},
            #{"LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR": 1},
            #{"LX_INT_FIRST_EXPOSURE": 1000},
            #{"LX_INT_SECOND_EXPOSURE": 50},
            #{"LX_INT_MIN_DEPTH": 0},
            #{"LX_INT_MAX_DEPTH": 8000},
            
            # <!-- 相机位姿配置 -->
            {"x": 0.0},
            {"y": 0.0},
            {"z": 0.0},
            {"roll": 0.0},
            {"pitch": 0.0},
            {"yaw": 0.0}
            ]
    ),

    # 启动节点rviz2（只有在enable_rviz为True时才会启动）
    Node(
        package='rviz2',
        executable='rviz2',
        name='lx_camera',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('lx_camera_ros'),'rviz','lx_camera.rviz')],
        condition=IfCondition(enable_rviz))
])
