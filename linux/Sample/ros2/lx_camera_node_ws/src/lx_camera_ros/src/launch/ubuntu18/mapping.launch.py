from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lx_camera_ros",
            node_executable="lx_localization_node",
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
		{"mapping_mode": True},
		#<!-- 定位使能, true or false -->
		{"localization_mode": False},
		#<!-- 相机至小车外参, 单位:米, 度数, 格式[x, y, z, yaw, pitch, roll] -->
		{"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
		#<!-- 激光雷达至小车外参, 单位:米, 度数, 格式[x, y, yaw] -->
		{"laser_extrinsic_param": [0.34, 0.11, 0.0]} ])
    ])

