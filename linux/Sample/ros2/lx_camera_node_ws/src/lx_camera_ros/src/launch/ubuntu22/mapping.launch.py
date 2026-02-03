from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
		{"mapping_mode": True},
		#<!-- Localization enable, true or false -->
		{"localization_mode": False},
		#<!-- Camera extrinsics to robot, units: meters/degrees, format [x, y, z, yaw, pitch, roll] -->
		{"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
		#<!-- LiDAR extrinsics to robot, units: meters/degrees, format [x, y, yaw] -->
		{"laser_extrinsic_param": [0.34, 0.11, 0.0]} ])
    ])
