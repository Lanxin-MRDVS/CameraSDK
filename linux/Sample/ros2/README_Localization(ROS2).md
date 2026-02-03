# Preface

This document describes how to use the V1Pro camera localization algorithm module. The example is based on ROS2 and has been tested on Ubuntu 22.04 (Humble). It uses the camera SDK for mapping (data collection) and localization. Before reading this document, please read the SDK guide **LxCameraApi (C, C++) Developer Guide.pdf**. On Linux, run `install.sh` to configure the environment.

> Note: If you need to change ROS2 launch parameters, edit the source launch files under `src/launch/ubuntu22/` and rebuild. Editing generated files under `install/` will be overwritten.

# Localization Algorithm Module

## Status / Error Codes

```
// status
// Algorithm return values: status codes
Unknown       = -1,  //!< Relocating
Tracking      = 0,   //!< Normal localization
RelocFailed   = 1,   //!< Relocation failed
Slipping      = 2,   //!< Slipping
Relocating    = 6,   //!< Relocating
NoTopImage    = 7,   //!< Camera image input error
NoOdom        = 8,   //!< Odometry timeout
Mapping       = 20,  //!< Mapping in progress; mapping requires odom, image, and laser data (laser data or laser pose must be present; virtual data is allowed)
MappingFailed = 21,  //!< Mapping error
NoLaser       = 22,  //!< Laser timeout (used during mapping)
NoLaserPose   = 23,  //!< Laser pose timeout (used during mapping)

// Additional return values: relocation result codes
-1: Unknown          //!< Relocating (if pose (x,y,yaw) is all zeros, it is treated as an invalid request and returns -1)
0: Normal            //!< Relocation success
1: No image          //!< Relocation failed: image input error
2: No detection      //!< Relocation failed: feature detection error
3: No expectation    //!< Relocation failed: expected map error
4: No both           //!< Relocation failed: both detection and expected map errors
5: No match          //!< Relocation failed: no match between detection and map
6: Off>30cm          //!< Relocation failed: match exists but distance exceeds threshold
7: invalid request   //!< Relocation failed: invalid requested pose (e.g., NaN)
```

## Localization Confidence

When status is normal (code 0), confidence can be used to evaluate localization quality. Confidence ranges from 0 to 4:

- 0: poor
- 1: low
- 2-3: medium
- 4: good

It is recommended to use localization results with confidence >= 1.

## Mapping

- Inputs
  - Robot odometry (absolute), type `nav_msgs/Odometry`
  - Robot laser scan (optional in some scenarios; contact support to confirm), type `sensor_msgs/LaserScan`
  - Robot laser pose (required if you need to align visual and laser map frames), type `geometry_msgs/PoseStamped`

- Outputs
  - Export raw data recorded by the camera, build the visual map offline, then import the visual map into the camera

## Localization

- Inputs
  - Robot odometry (absolute), type `nav_msgs::msg::Odometry`

- Outputs
  - Send relocation request; after relocation succeeds, the camera outputs the robot pose in the visual map frame (`geometry_msgs::msg::PoseStamped`)

# Build

ROS2 node build steps:

1. Enter the example workspace directory: `linux/Sample/ros2/lx_camera_node_ws`
2. `colcon build`
3. `source install/setup.bash`

# Run

Before running the ROS example, if the camera has ROS master/slave settings enabled, disable them to avoid communication issues. If the camera has a ROS2 (Humble) environment, wait about 20 seconds after power-on before running the example.

## Communication Test

You can validate SDK communication using virtual data.

1. `source install/setup.bash`

2. `ros2 launch lx_camera_ros sensor_sim.launch.py`

   This node publishes virtual laser data `/sim/scan`, odometry `/sim/odom`, and laser pose `/sim/scan_pose`. The camera SDK receives these topics to verify communication.

   ```python
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
               # <!-- -135° -->
               {"min_ang": -2.35619449},
               # <!-- 135° -->
               {"max_ang": 2.35619449},
               {"angle_increment": 0.00582},
               {"time_increment": 0.00006167129},
               {"range_min": 0.05},
               {"range_max": 101.0},
               {"init_range": 100.0},
   
               # <!-- Virtual laser topic -->
               {"laser_frameId": "laser_link"},
               {"laser_topic_name": "/sim/scan"},
               # <!-- Virtual odom topic -->
               {"odom_frame_id": "base"},
               {"odom_topic_name": "/sim/odom"},
               # <!-- Virtual laser pose topic -->
               {"laserpose_frame_id": "base"},
               {"laserpose_topic_name": "/sim/scan_pose"}])
       ])
   ```

3. `ros2 launch lx_camera_ros mapping.launch.py`

   This node receives the virtual sensor data and passes it to the camera SDK.

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   
   def generate_launch_description():
       return LaunchDescription([
           Node(
               package="lx_camera_ros",
               executable="lx_localization_node",
               output="screen",
               emulate_tty=True,
               parameters=[
               # <!-- Camera IP (empty = use device index) -->
               {"ip": "192.168.100.82"},
               # <!-- Image display enable -->
               {"is_show": False},
               # <!-- ROS topic names -->
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
               # <!-- Auto exposure value [0-100], default 50 -->
               {"auto_exposure_value": 50},
               # <!-- Mapping enable -->
               {"mapping_mode": True},
               # <!-- Localization enable -->
               {"localization_mode": False},
               # <!-- Camera extrinsics (m, deg): [x, y, z, yaw, pitch, roll] -->
               {"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
               # <!-- Laser extrinsics (m, deg): [x, y, yaw] -->
               {"laser_extrinsic_param": [0.34, 0.11, 0.0]} ])
       ])
   ```

4. When the algorithm returns status code **20 (Mapping)**, communication is OK.

## Mapping

1. Prepare required sensor data

   - Odometry topic (absolute), type `nav_msgs::msg::Odometry`
   - Laser scan topic (optional; contact support to confirm), type `sensor_msgs::msg::LaserScan`
   - Laser pose topic (required if you need unified visual/laser frames), type `geometry_msgs::msg::PoseStamped`

2. **Edit `mapping.launch.py` as required** and verify the values: set input topic names (e.g., `/scan`, `/odom`, `/scan_pose`), map name (use existing name or default `"example_map1"`), camera extrinsics, laser extrinsics, etc.

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   
   def generate_launch_description():
       return LaunchDescription([
           Node(
               package="lx_camera_ros",
               executable="lx_localization_node",
               output="screen",
               emulate_tty=True,
               parameters=[
               # <!-- Camera IP (empty = use device index) -->
               {"ip": "192.168.100.82"},
               # <!-- Image display enable -->
               {"is_show": False},
               # <!-- ROS topic names -->
               {"LxCamera_UploadScan": "/scan"},
               {"LxCamera_UploadOdom": "/odom"},
               {"LxCamera_UploadLaserPose": "/scan_pose"},
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
               # <!-- Auto exposure value [0-100], default 50 -->
               {"auto_exposure_value": 50},
               # <!-- Mapping enable -->
               {"mapping_mode": True},
               # <!-- Localization enable -->
               {"localization_mode": False},
               # <!-- Camera extrinsics (m, deg): [x, y, z, yaw, pitch, roll] -->
               {"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
               # <!-- Laser extrinsics (m, deg): [x, y, yaw] -->
               {"laser_extrinsic_param": [0.34, 0.11, 0.0]} ])
       ])
   ```

3. `source install/setup.bash`

4. `ros2 launch lx_camera_ros mapping.launch.py`

5. When status code **20 (Mapping)** is returned, you can move the robot to record data.

6. Export recorded data. This can be done via SDK APIs, **but mapping must be disabled first**. You can also use the ROS2 topic command below:

   Disable mapping:

   ```
   ros2 topic pub -1 /lx_localization_node/LxCamera_Mapping std_msgs/msg/String "{data: '0'}"
   ```

   Then export data (adjust the path for your environment):

   ```
   ros2 topic pub -1 /lx_localization_node/LxCamera_DownloadMap std_msgs/String "{data: 'your_dir/Lanxin-MRDVS/Sample/ROS2/lx_camera_node_ws/src/lx_camera_ros/map/download_map'}"
   ```

   Check status via `/lx_localization_node/LxCamera_Message`:

   ```
   ros2 topic echo /lx_localization_node/LxCamera_Message | grep DownloadMap
   ```

   If the result is like below and the map file appears (e.g., `download_map.zip`), export succeeded. **Please contact MRDVS support and provide the mapping data.**

   ```
   data: "{\"cmd\":\"LxCamera_DownloadMap\",\"result\":0}
   ```

7. After mapping, **disable mapping first**, then import the map file into the camera.
   - SDK versions **before 2024-08-26**: map suffix is `.zip`
   - SDK versions **on/after 2024-08-26**: map suffix is `.bin`

   Example:

   ```
   ros2 topic pub -1 /lx_localization_node/LxCamera_UploadMap std_msgs/String "{data: '/home/fr1511b/v1pro/Lanxin-MRDVS/Sample/ros-v1pro/map/xz9_231216.bin'}"
   ```

   Check status:

   ```
   ros2 topic echo /lx_localization_node/LxCamera_Message | grep UploadMap
   ```

   Success example:

   ```
   data: "{\"cmd\":\"LxCamera_UploadMap\",\"result\":0}
   ```

## Localization

1. Prepare required sensor data

   - Odometry topic, type `nav_msgs::msg::Odometry`

2. **Edit `localization.launch.py`** and verify values: map name (an existing map, e.g. the one just imported), camera intrinsics/extrinsics, etc.

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   
   def generate_launch_description():
       return LaunchDescription([
           Node(
               package="lx_camera_ros",
               executable="lx_localization_node",
               output="screen",
               emulate_tty=True,
               parameters=[
               # <!-- Camera IP (empty = use device index) -->
               {"ip": "192.168.100.82"},
               # <!-- Image display enable -->
               {"is_show": False},
               # <!-- ROS topic names -->
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
               # <!-- Auto exposure value [0-100], default 50 -->
               {"auto_exposure_value": 50},
               # <!-- Mapping enable -->
               {"mapping_mode": False},
               # <!-- Localization enable -->
               {"localization_mode": True},
               # <!-- Important: map name must exist on the camera. For mapping, use default "example_map1" if no map exists -->
               {"map_name": "xz9_231216"},
               # <!-- Camera extrinsics (m, deg): [x, y, z, yaw, pitch, roll] -->
               {"camera_extrinsic_param": [0.34, 0.00, 1.3, -90, 0, 0]},
               # <!-- Laser extrinsics (m, deg): [x, y, yaw] -->
               {"laser_extrinsic_param": [0.34, 0.11, 0.0]} ])
       ])
   ```

3. `source install/setup.bash`

4. Send a relocation request. Move the robot to a target location and send that pose in the visual map. Usually the starting pose of mapping is (x=0, y=0, yaw=0).
   **Important:** a relocation request at (0,0,0) is treated as invalid. Use a small offset such as (0.01, 0.00, 0).

   Example topic command:

   ```
   ros2 topic pub -1 /lx_localization_node/LxCamera_UploadReloc geometry_msgs/msg/PoseWithCovarianceStamped "{
     header: {
       stamp: { sec: 0, nanosec: 0 },
       frame_id: 'map'
     },
     pose: {
       pose: {
         position: { x: 0.01, y: 0.00, z: 0.0 },
         orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
       },
       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     }
   }"
   ```

   Check status:

   ```
   ros2 topic echo /lx_localization_node/LxCamera_Message | grep UploadReloc
   ```

   Success example:

   ```
   data: "{\"cmd\":\"LxCamera_UploadReloc\",\"result\":0}
   ```

5. When status code **0 (Tracking)** is returned and `/lx_localization_node/LxCamera_LocationResult` is published, localization is OK. The camera outputs the robot pose in the visual map.

# FAQ

When issues occur, check the SDK logs first. Common cases:

1. If ROS example stutters or communication is unstable, check and disable ROS master/slave settings on the camera.
2. If restarting the ROS example fails, it may be a stream lock; wait 10-20 seconds and retry.
3. If SDK library load fails or packets drop frequently, check whether `install.sh` was run successfully.
4. If the program crashes after relocation succeeds, it may be a camera internal configuration issue; contact support.

# References

- LxCameraApi (C, C++) Developer Guide.pdf
- LxCameraViewer User Manual.pdf
- V1Pro User Manual.pdf
