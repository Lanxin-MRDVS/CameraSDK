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
    # Start lx_localization_node
    Node(
					package="lx_camera_ros",
					executable="lx_localization_node",
					namespace="lx_localization_node",
					output="screen",
					emulate_tty=True,
					parameters=[
		#<!-- Camera IP; empty uses device index -->
		{"ip": "192.168.100.82"},
		#<!-- Image display enable -->
		{"is_show": False},
		#<!-- ROS topic names (subscribe/publish) -->
		{"LxCamera_UploadScan": "/sim/scan"},
		{"LxCamera_UploadOdom": "/sim/odom"},
		{"LxCamera_UploadLaserPose": "/sim/scan_pose"},
		{"LxCamera_Error": "LxCamera_Error"},
		{"LxCamera_Command": "LxCamera_Command"},
		{"LxCamera_Mapping": "LxCamera_Mapping"},
		{"LxCamera_Location": "LxCamera_Location"},
		{"LxCamera_SetParam": "LxCamera_SetParam"},
		{"LxCamera_SwitchMap": "LxCamera_SwitchMap"},
		{"LxCamera_DownloadMap": "LxCamera_DownloadMap"},
		{"LxCamera_UploadMap": "LxCamera_UploadMap"},
		{"LxCamera_LocationResult": "LxCamera_LocationResult"},
		{"LxCamera_UploadReloc": "LxCamera_UploadReloc"},
		#<!-- Auto exposure value [0-100], default: 50 -->
		{"auto_exposure_value": 50},
		#<!-- Mapping enable, true or false -->
		{"mapping_mode": False},
		#<!-- Localization enable, true or false -->
		{"localization_mode": True},
		#<!-- Important: map name must exist on the camera; for mapping, use default \"example_map1\" if no map exists -->
		{"map_name": "xz9_231216"},
		#<!-- Camera extrinsics to robot, units: meters/degrees, format [x, y, z, yaw, pitch, roll] -->
		{"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
		#<!-- LiDAR extrinsics to robot, units: meters/degrees, format [x, y, yaw] -->
		{"laser_extrinsic_param": [0.34, 0.11, 0.0]} ]),
    
    # Start rviz2 (only when enable_rviz is True)
    Node(
        package='rviz2',
        executable='rviz2',
        name='localization',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('lx_camera_ros'),'rviz','localization.rviz')],
        condition=IfCondition(enable_rviz))
    ])
