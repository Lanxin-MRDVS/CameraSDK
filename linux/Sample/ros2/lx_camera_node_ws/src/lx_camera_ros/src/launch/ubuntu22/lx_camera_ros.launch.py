from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Whether to enable rviz display
  enable_rviz =  LaunchConfiguration('enable_rviz')
  
  return LaunchDescription([
    # Declare parameters; can be overridden via command line
    DeclareLaunchArgument('enable_rviz', default_value='true', description='Whether to launch rviz2'),

    # Start lx_camera_node
    Node(
        package="lx_camera_ros",
        executable="lx_camera_node",
        namespace="lx_camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            # <!-- IP, log path, stream config, algorithm, work mode, point cloud unit -->
        	  {"ip": "0"},
            {"log_path": "/var/log/"},
            {"is_xyz": 1},
            {"is_depth": 1},
            {"is_amp": 1},
            {"is_rgb": 1},
            {"lx_work_mode": 0},
            {"lx_application": 0},
            {"lx_tof_unit": 1},

            # <!-- Camera pose config -->
            {"x": 0.0},
            {"y": 0.0},
            {"z": 0.0},
            {"roll": 0.0},
            {"pitch": 0.0},
            {"yaw": 0.0},

            # <!-- Whether to use launch parameters -->
            {"raw_param": 0},

            # <!-- 2D config -->
            {"lx_2d_binning": 0},
            {"lx_2d_undistort": 0},
            {"lx_2d_undistort_scale": 51},
            {"lx_2d_auto_exposure": 0},
            {"lx_2d_auto_exposure_value": 11},
            {"lx_2d_exposure": 10001},
            {"lx_2d_gain": 101},

            # <!-- 3D config -->
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

            # <!-- Depth -->
            {"lx_min_depth": 0},
            {"lx_max_depth": 8000}]),

    # Start rviz2 (only when enable_rviz is True)
    Node(
        package='rviz2',
        executable='rviz2',
        name='lx_camera',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('lx_camera_ros'),'rviz','lx_camera.rviz')],
        condition=IfCondition(enable_rviz))
])
