from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lx_camera_ros",
            node_executable="sensor_sim_node",
            output="screen",
            emulate_tty=True,
            parameters=[
		#<!-- -135° -->
		{"min_ang": -2.35619449},
		#<!-- 135° -->
		{"max_ang": 2.35619449},
		{"angle_increment": 0.00582},
		{"time_increment": 0.00006167129},
		{"range_min": 0.05},
		{"range_max": 101.0},
		{"init_range": 100.0},

		#<!-- 虚拟激光数据topic -->
		{"laser_frame_id": "laser_link"},
		{"laser_topic_name": "/sim/scan"},
		#<!-- 虚拟里程计数据topic -->
		{"odom_frame_id": "base"},
		{"odom_topic_name": "/sim/odom"},
		#<!-- 虚拟激光位姿topic -->
		{"laserpose_frame_id": "base"},
		{"laserpose_topic_name": "/sim/scan_pose"}])
    ])

