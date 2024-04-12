from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lx_camera_ros",
            executable="sensor_sim_node",
            output="screen",
            emulate_tty=True,
            parameters=[
		#<!-- -135° -->
		{"minAng": -2.35619449}
		#<!-- 135° -->
		{"maxAng": 2.35619449}
		{"angleIncrement": 0.00582}
		{"timeIncrement": 0.00006167129}
		{"rangeMin": 0.05}
		{"rangeMax": 101.0}
		{"initRange": 100}
		#<!-- <remap from="scan" to="scan_back"/> -->

		#<!-- 虚拟激光数据topic -->
		{"laserFrameId": "laser_link"}
		{"laserFopicFame": "/sim/scan"}
		#<!-- 虚拟里程计数据topic -->
		{"odomFrameFd": "base"}
		{"odomTopicName": "/sim/odom"}
		#<!-- 虚拟激光位姿topic -->
		{"laserposeFrameId": "base"}
		{"laserposeTopicName": "/sim/scan_pose"}

            ]
        )
    ])

