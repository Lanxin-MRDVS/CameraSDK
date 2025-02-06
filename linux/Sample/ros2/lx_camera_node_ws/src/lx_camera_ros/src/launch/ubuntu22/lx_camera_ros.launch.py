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
    DeclareLaunchArgument('enable_rviz', default_value='true', description='Whether to launch rviz2'),

    # 启动节点lx_camera_node
    Node(
        package="lx_camera_ros",
        executable="lx_camera_node",
        namespace="lx_camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            # <!-- ip、日志路径、流配置、算法、工作模式配置、点云单位 -->
        	  {"ip": "0"},
            {"log_path": "/var/log/"},
            {"is_xyz": 1},
            {"is_depth": 1},
            {"is_amp": 1},
            {"is_rgb": 1},
            {"lx_work_mode": 0},
            {"lx_application": 0},
            {"lx_tof_unit": 1},

            # <!-- 相机位姿配置 -->
            {"x": 0.0},
            {"y": 0.0},
            {"z": 0.0},
            {"roll": 0.0},
            {"pitch": 0.0},
            {"yaw": 0.0},

            # <!-- 是否使用launch配置 -->
            {"raw_param": 0},

            # <!-- 2D配置 -->
            {"lx_2d_binning": 0},
            {"lx_2d_undistort": 0},
            {"lx_2d_undistort_scale": 51},
            {"lx_2d_auto_exposure": 0},
            {"lx_2d_auto_exposure_value": 11},
            {"lx_2d_exposure": 10001},
            {"lx_2d_gain": 101},

            # <!-- 3D配置 -->
            {"lx_rgb_to_tof": 0},
            {"lx_3d_binning": 0},
            {"lx_mulit_mode": 0},
            {"lx_3d_undistort": 0},
            {"lx_3d_undistort_scale": 0},
            {"lx_hdr": 0},
            {"lx_3d_auto_exposure": 1},
            {"lx_3d_auto_exposure_value": 50},
            {"lx_3d_first_exposure": 1100},
            {"lx_3d_second_exposure": 200},
            {"lx_3d_gain": 11},

            # <!-- 深度 -->
            {"lx_min_depth": 0},
            {"lx_max_depth": 8000}]),

    # 启动节点rviz2（只有在enable_rviz为True时才会启动）
    Node(
        package='rviz2',
        executable='rviz2',
        name='lx_camera',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('lx_camera_ros'),'rviz','lx_camera.rviz')],
        condition=IfCondition(enable_rviz))
])
