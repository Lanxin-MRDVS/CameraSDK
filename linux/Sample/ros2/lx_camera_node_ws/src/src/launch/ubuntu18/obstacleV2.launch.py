from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lx_camera_ros",
            node_executable="lx_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
			#<!-- ip、日志路径、流配置、算法、工作模式配置、点云单位 -->
			{"ip" : "0"},
			{"log_path" : "/var/log/"},
			{"is_xyz" : 1 },
			{"is_depth" : 1 },
			{"is_amp" : 1 },
			{"is_rgb" : 1 },
			{"lx_work_mode" : 0},
			{"lx_application" : 4},
			{"lx_tof_unit" : 1},

			#<!-- 相机位姿配置 -->
			{"x" : 0},
			{"y" : 0},
			{"z" : 0},
			{"yaw" : 0},
			{"roll" : 0},
			{"pitch" : 90},
			
			#<!-- 是否使用launch配置 -->
			{"raw_param": 0},

			#<!-- 2D配置 -->
			{"lx_2d_binning" : 0 },
			{"lx_2d_undistort" : 0 },
			{"lx_2d_undistort_scale" : 51 },
			{"lx_2d_auto_exposure" : 0 },
			{"lx_2d_auto_exposure_value" : 11 },
			{"lx_2d_exposure" : 10001},
			{"lx_2d_gain" : 101 },

			#<!-- 3D配置 -->
			{"lx_rgb_to_tof" : 0 },
			{"lx_3d_binning" : 0},
			{"lx_mulit_mode" : 0 },
			{"lx_3d_undistort" : 0},
			{"lx_3d_undistort_scale" : 0},
			{"lx_hdr" : 0},
			{"lx_3d_auto_exposure" : 1},
			{"lx_3d_auto_exposure_value" : 50},
			{"lx_3d_first_exposure" : 1100},
			{"lx_3d_second_exposure" : 200},
			{"lx_3d_gain" : 11},
			
			#<!-- 深度 -->
			{"lx_min_depth" : 0},
			{"lx_max_depth" : 8000}
            ]
        )
    ])

