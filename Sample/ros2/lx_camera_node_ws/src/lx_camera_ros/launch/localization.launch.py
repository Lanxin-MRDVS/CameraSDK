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
    # 启动节点lx_localization_node
    Node(
					package="lx_camera_ros",
					executable="lx_localization_node",
					namespace="lx_localization_node",
					output="screen",
					emulate_tty=True,
					parameters=[
		#<!-- 相机IP，默认为空时按设备索引获取 -->
		{"ip": "192.168.100.82"},
		#<!-- 图像显示使能 -->
		{"is_show": False},
		#<!-- 接收或发布ROS话题名 -->
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
		#<!-- 自动曝光期望值，范围[0-100], 默认:50 -->
		{"auto_exposure_value": 50},
		#<!-- 建图使能, true or false -->
		{"mapping_mode": False},
		#<!-- 定位使能, true or false -->
		{"localization_mode": True},
		#<!-- 重要：定位时地图名需有效，已上传相机并存在；建图时,如果相机中无地图可默认输入"example_map1" -->
		{"map_name": "xz9_231216"},
		#<!-- 相机至小车外参, 单位:米, 度数, 格式[x, y, z, yaw, pitch, roll] -->
		{"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
		#<!-- 激光雷达至小车外参, 单位:米, 度数, 格式[x, y, yaw] -->
		{"laser_extrinsic_param": [0.34, 0.11, 0.0]} ]),
    
    # 启动节点rviz2（只有在enable_rviz为True时才会启动）
    Node(
        package='rviz2',
        executable='rviz2',
        name='localization',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('lx_camera_ros'),'rviz','localization.rviz')],
        condition=IfCondition(enable_rviz))
    ])

