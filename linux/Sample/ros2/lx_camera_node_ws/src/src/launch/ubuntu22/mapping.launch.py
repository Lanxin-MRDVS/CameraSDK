from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lx_camera_ros",
            executable="location_node",
            output="screen",
            emulate_tty=True,
            parameters=[
		#<!-- 相机IP，默认为空时按设备索引获取 -->
		{"ip": "192.168.100.82"}
		#<!-- 接收或发布ROS话题名 -->
		{"LxCameraUploadScan": "/sim/scan"}
		{"LxCameraUploadOdom": "/sim/odom"}
		{"LxCameraUploadLaserPose": "/sim/scan_pose"}
		{"LxCameraError": "LxCamera_Error"}
		{"LxCameraCommand": "LxCamera_Command"}
		{"LxCameraMapping": "LxCamera_Mapping"}
		{"LxCameraLocation": "LxCamera_Location"}
		{"LxCameraSetParam": "LxCamera_SetParam"}
		{"LxCameraSwitchMap": "LxCamera_SwitchMap"}
		{"LxCameraDownloadMap": "LxCamera_DownloadMap"}
		{"LxCameraUploadMap": "LxCamera_UploadMap"}
		{"LxCameraLocationResult": "LxCamera_LocationResult"}
		{"LxCameraUploadReloc": "LxCamera_UploadReloc"}
		#<!-- 自动曝光期望值，范围[0-100], 默认:50 -->
		{"autoExposureValue": 50}
		#<!-- 建图使能, true or false -->
		{"mappingMode": true}
		#<!-- 定位使能, true or false -->
		{"localizationMode": false}
		#<!-- 重要：定位时地图名需有效，已上传相机并存在；建图时,如果相机中无地图可默认输入"example_map1" -->
		{"mapName": "example_map1"}
		#<!-- 相机至小车外参, 单位:米, 度数, 格式[x, y, z, yaw, pitch, roll] -->
		<rosparam param="cameraExtrinsicParam"> [0.34, 0.00, 1.3, -90, 0, 0] </rosparam>
		#<!-- 激光雷达至小车外参, 单位:米, 度数, 格式[x, y, yaw] -->
		<rosparam param="laserExtrinsicParam"> [0.34, 0.11, 0.0] </rosparam>
            ]
        )
    ])

