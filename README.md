# Lx Camera SDK (Linux / ROS2 / C++)

This repo is trimmed to **Linux + ROS2 + C++ only**. Non-Linux, non-ROS2, and non-C++ content has been removed.

## Contents

- `linux/SDK/include`: SDK headers
- `linux/SDK/lib`: SDK shared libraries for Linux
- `linux/Sample/C`: C/C++ examples
- `linux/Sample/ros2/lx_camera_node_ws`: ROS2 driver workspace

## Compatibility Check (as of Feb 3, 2026)

- **Ubuntu 22.04 + ROS2 Humble**: Documented as tested in the vendor docs (`linux/Sample/ros2/README` and `linux/Sample/ros2/README_Localization(ROS2).md`). Considered compatible.
- **Ubuntu 24.04 + ROS2 (Jazzy)**: No vendor test statement in this repo. The build uses the Ubuntu 22.04 launch set for any OS newer than 18.04. It may build, but **is not officially verified** here.

## Prerequisites

- Ubuntu 22.04 (recommended) or 24.04 (unverified)
- ROS2 Humble (22.04) / ROS2 Jazzy (24.04)
- `colcon`, `ament_cmake`, and ROS2 dependencies (OpenCV, PCL, cv_bridge, etc.)

Install ROS2 dependencies via rosdep:

```bash
cd linux/Sample/ros2/lx_camera_node_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Install SDK

This copies SDK headers/libs into `/opt/Lanxin-MRDVS` and sets `LD_LIBRARY_PATH`.

```bash
cd linux
sudo ./install.sh
```

## Build ROS2 Driver

```bash
cd linux/Sample/ros2/lx_camera_node_ws
colcon build
source install/setup.bash
```

Or use the script:

```bash
cd linux/Sample/ros2/lx_camera_node_ws
./build.sh
```

## Run (ROS2)

### Default launch

```bash
cd linux/Sample/ros2/lx_camera_node_ws
source install/setup.bash
ros2 launch lx_camera_ros lx_camera_ros.launch.py
```

### Other launch files

```bash
ros2 launch lx_camera_ros obstacle.launch.py
ros2 launch lx_camera_ros obstacleV2.launch.py
ros2 launch lx_camera_ros pallet.launch.py
ros2 launch lx_camera_ros mapping.launch.py
ros2 launch lx_camera_ros localization.launch.py
ros2 launch lx_camera_ros sensor_sim.launch.py
```

## Where to Change Parameters

Launch files are copied during build. **Edit the source launch files** under:

```
linux/Sample/ros2/lx_camera_node_ws/src/lx_camera_ros/src/launch/ubuntu22/
```

Then rebuild:

```bash
cd linux/Sample/ros2/lx_camera_node_ws
colcon build
```

## Typical Parameter Updates (examples)

In `mapping.launch.py` and `localization.launch.py` you will commonly change:

- Camera IP: `ip`
- Topic names: `/scan`, `/odom`, `/scan_pose`
- Extrinsics: `camera_extrinsic_param`, `laser_extrinsic_param`
- Mode flags: `mapping_mode`, `localization_mode`

Example snippet (adjust values for your robot):

```python
parameters=[
  {"ip": "192.168.100.82"},
  {"LxCamera_UploadScan": "/scan"},
  {"LxCamera_UploadOdom": "/odom"},
  {"LxCamera_UploadLaserPose": "/scan_pose"},
  {"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
  {"laser_extrinsic_param": [0.34, 0.11, 0.0]},
  {"mapping_mode": True},
  {"localization_mode": False}
]
```

## Notes

- On Ubuntu 22.04+ the driver enables **dynamic loading** to avoid FastDDS conflicts (see `CMakeLists.txt`).
- Launch files are copied into `launch/` during build from `src/launch/ubuntu22/`.
- Localization module docs are in `linux/Sample/ros2/README_Localization(ROS2).md`.

