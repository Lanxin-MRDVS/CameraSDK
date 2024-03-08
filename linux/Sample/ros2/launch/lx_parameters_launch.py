from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lx_camera_ros2",
            node_executable="lx_camera_node",
            node_name="const_lx_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"ip": "0"},
                {"is_show": 1},
                {"is_xyz": 1},
                {"is_depth": 1},
                {"is_amp": 1},
                {"is_rgb": 1},
                {"application": 0}
            ]
        )
    ])

