<p align="center">
  <img src="./assets/mrdvs_logo.png" alt="MRDVS Logo" width="300">
</p>

<h1 align="center">LxCameraSDK</h1>

<p align="center">
  Industrial camera SDK for MRDVS visual hardware products.<br>
  面向迈尔微视视觉硬件产品的工业相机开发套件。
</p>

<p align="center">
  <a href="https://hub.mrdvs.cn/">MRDVS Hub 2.0</a> |
  <a href="#english">English</a> |
  <a href="#中文">中文</a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/SDK-2.4.60.260126-2563EB" alt="SDK Version">
  <img src="https://img.shields.io/badge/platform-Linux%20%7C%20Windows-0F172A" alt="Platform">
  <img src="https://img.shields.io/badge/API-C%2FC%2B%2B%20%7C%20Python%20%7C%20C%23-475569" alt="APIs">
  <img src="https://img.shields.io/badge/robotics-ROS%20%7C%20ROS2-0891B2" alt="ROS and ROS2">
</p>

---

# English

## Contents

- [Overview](#overview)
- [Product Portfolio](#product-portfolio)
- [Documents](#documents)
- [Supported Platforms](#supported-platforms)
- [Package Layout](#package-layout)
- [Path Notes](#path-notes)
- [Quick Start](#quick-start)
- [Examples](#examples)
- [Core Development Flow](#core-development-flow)
- [ROS and ROS2 Entry](#ros-and-ros2-entry)
- [Runtime Notes](#runtime-notes)
- [Version](#version)
- [Support and Feedback](#support-and-feedback)

## Overview

LxCameraSDK provides the runtime libraries, C/C++ headers, Python package, C# wrapper samples, and ROS/ROS2 workspaces required to integrate MRDVS camera devices into industrial vision systems.

Typical integration tasks covered by this package include:

- Discovering and opening camera devices by index, IP address, SN, or device ID.
- Starting and stopping data streams.
- Reading RGB, depth, amplitude, full-frame, point-cloud, lidar-point, IMU, and application output data.
- Configuring camera parameters through integer, float, boolean, string, command, pointer, and ROI APIs.
- Running sample flows for single camera, multiple cameras, callback acquisition, pallet positioning, obstacle avoidance, visual localization, lidar-point callbacks, and IMU callbacks.
- Publishing camera data through ROS and ROS2 nodes for robotics systems.

## Product Portfolio

MRDVS is dedicated to helping robots understand the world through 3D vision and AI. Zhejiang MRDVS Technology Co., Ltd. focuses on the development of visual sensors for mobile robots and provides integrated hardware and software solutions that combine 3D vision with AI algorithms. The company's core technical team was established in 2016, and MRDVS is a wholly owned subsidiary of Hangzhou Lanxin Robot Technology Co., Ltd.

MRDVS serves embodied intelligence robots, lawn-mowing robots, commercial cleaning robots, low-speed autonomous vehicles, intelligent wheelchairs, industrial mobile robots, and warehouse logistics automation scenarios. Based on iToF, dToF, binocular structured light, AI algorithms, and multi-sensor fusion, MRDVS continuously iterates visual hardware and algorithm solutions to help robots operate more safely, stably, and intelligently.

For official product information, visit [MRDVS Hub 2.0](https://hub.mrdvs.cn/).

### Visual Hardware Products

| Product line | Description | Typical applications | Image |
| --- | --- | --- | --- |
| S Series dToF LiDAR-Vision Fusion Sensor | Designed for complex indoor and outdoor environments, the S Series combines solid-state LiDAR and visual perception. It supports high-precision ranging, semantic obstacle avoidance, and SLAM mapping, with a maximum ranging distance of 42 m and a maximum field of view of 140 degrees. | Mobile robot SLAM, semantic obstacle avoidance, indoor and outdoor environment perception | <img src="./assets/S-series.png" alt="S Series" width="2000"> |
| M Series ToF Depth Camera | An RGB-D depth camera based on Sony iToF chips. It is optimized for working distances up to 5 m and adapts to different lighting conditions, object textures, and real-time detection requirements. | Pallet recognition, bin stacking, volume measurement, robotic arm visual guidance, depalletizing and grasping | <img src="./assets/M-series.png" alt="M Series" width="2000"> |
| V Series Visual Navigation Camera | A full-stack embedded positioning system that fuses 2D LiDAR, vision, and IMU. It provides stable and continuous localization data without relying on external preset markers. | Indoor unmanned forklifts, commercial cleaning robots, digital retrofit of traditional manual forklifts, warehouse localization | <img src="./assets/V-series.png" alt="V Series" width="2000"> |
| H Series High-Precision Structured-Light Camera | A high-precision 3D camera based on binocular coded structured light, designed for high-precision recognition, grasping, and real-time 3D feedback. | Automated production, logistics sorting, robotic operation, high-precision recognition and grasping | <img src="./assets/H-series.png" alt="H Series" width="2000"> |

### Industrial Solutions

| Solution | Description | Image |
| --- | --- | --- |
| Industrial Mobile Robot Visual Perception | Designed for industrial mobile robot scenarios, this solution provides an integrated 3D vision hardware and software system, supporting autonomous localization, intelligent obstacle avoidance, pallet recognition, and bin stacking. | <img src="./assets/industrial_mobile_robot.jpg" alt="Industrial Mobile Robot Visual Perception" width="2000"> |
| Warehouse Logistics Automation Vision | Designed for warehouse sensing systems, this solution provides comprehensive 3D perception capabilities, supporting automated warehouse safety monitoring, personnel safety protection, cargo volume measurement, and storage location status detection to improve warehouse logistics management efficiency. | <img src="./assets/warehouse_logistics_automation.webp" alt="Warehouse Logistics Automation Vision" width="2000"> |
| Intelligent Lawn-Mowing Robot Perception | Integrating solid-state LiDAR and visual perception, this solution supports surrounding perception, low-obstacle detection, centimeter-level mapping, real-time localization, and safe path planning, improving reliability in complex lawn environments. | <img src="./assets/lawn_mowing_robot_perception.webp" alt="Intelligent Lawn-Mowing Robot Perception" width="2000"> |
| Outdoor Low-Speed Autonomous Vehicle Perception | Designed for outdoor low-speed autonomous vehicles, this solution provides RGB-D perception with strong anti-interference capability, supporting wide-field coverage, low-latency point cloud output, and high-precision spatiotemporal synchronization. | <img src="./assets/outdoor_low_speed_autonomous_vehicle.webp" alt="Outdoor Low-Speed Autonomous Vehicle Perception" width="2000"> |
| Humanoid and Quadruped Robot Perception | Designed for humanoid and quadruped robots, this solution provides panoramic perception, close-range obstacle avoidance, dynamic gait adaptation, 3D terrain modeling, and obstacle recognition, enhancing flexible interaction and environmental understanding in complex scenarios. | <img src="./assets/humanoid_quadruped_robot.webp" alt="Humanoid and Quadruped Robot Perception" width="2000"> |
| UAV Full-Scenario Visual Perception | Based on a multimodal perception architecture integrating solid-state LiDAR, vision cameras, and IMU, this solution supports high-precision mapping and localization, full-scenario 3D perception, semantic object recognition, and safe obstacle avoidance, improving UAV flight safety and stability. | <img src="./assets/uav_full_scenario_perception.webp" alt="UAV Full-Scenario Visual Perception" width="2000"> |


## Documents


- LxCameraSDK C/C++ Developer Guide [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_EN/MRDVS_LxCameraSDK_C-Cpp_DeveloperGuide_V1.0_20260604.pdf)
- LxCameraSDK Python Developer Guide [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_EN/LxCameraSDK-Python%20User%20Manual_EN.PDF)
- LxCameraViewer User Manual [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_EN/LxCameraViewer%20User%20Manual.pdf)


## Supported Platforms

| Platform | Architectures in package | Runtime libraries | Samples |
| --- | --- | --- | --- |
| Linux | `linux_x64`, `linux_aarch64`, `linux_arm32` | `libLxCameraApi.so`, `libLxDataProcess.so` | C/C++, Python, ROS, ROS2 |
| Windows | `win_x64`, `win_x86` | `LxCameraApi.dll`, `LxCameraApi.lib`, `LxDataProcess.dll` | C/C++, Python, C# |

## Package Layout

```text
CameraSDK/
|-- Document_EN/
|-- Document_CN/
|-- linux/
|   |-- install.sh
|   |-- set_socket_buffer_size.sh
|   |-- set_firewall.sh
|   |-- SDK/
|   |   |-- include/
|   |   `-- lib/
|   |       |-- linux_x64/
|   |       |-- linux_aarch64/
|   |       `-- linux_arm32/
|   `-- Sample/
|       |-- C/
|       |-- python/
|       |-- ROS/
|       `-- ROS2/
|-- windows/
|   |-- SDK/
|   |   |-- include/
|   |   `-- lib/
|   |       |-- win_x64/
|   |       `-- win_x86/
|   `-- Sample/
|       |-- C/
|       |-- python/
|       `-- C#/
|-- README.md
`-- release_note.txt
```

## Path Notes

Commands in this README assume they are executed from the parent directory of `CameraSDK`, where `CameraSDK` is the directory name after `git clone`:

```bash
git clone https://github.com/Lanxin-MRDVS/CameraSDK.git
```

If you are already in the SDK root directory, omit the `CameraSDK/` prefix in the commands. If you use an extracted archive or installer package directory, such as `CameraSDK-master`, replace `CameraSDK` in the commands with your actual directory name.

## Quick Start

### Linux

Install SDK runtime files and configure the shared library path:

```bash
cd CameraSDK/linux
source install.sh
```

`install.sh` deploys headers and libraries to `/opt/MRDVS` and configures `/opt/MRDVS/lib` as the runtime library path. The script requires root permission; when run as a non-root user, it prompts whether to use `sudo`.

Build C/C++ samples:

```bash
cd CameraSDK/linux/Sample/C
cmake -S . -B build
cmake --build build
```

Run a sample:

```bash
./build/bin/demo_single_camera2
```

Install and run the Python sample:

```bash
cd CameraSDK/linux/Sample/python
python3 -m pip install ./lx_camera_py-1.3.3-py3-none-any.whl
python3 demo.py
```

Before running `demo.py`, change the SDK library path in the script to `/opt/MRDVS/lib/libLxCameraApi.so`, and replace the sample camera IP with the actual camera IP.

### Windows

Build C/C++ samples:

```powershell
cd CameraSDK\windows\Sample\C
cmake -S . -B build
cmake --build build --config Release
```

Before running a sample, make sure the DLL directory for the target architecture can be loaded. Use `win_x64` for 64-bit programs and `win_x86` for 32-bit programs.

```powershell
$env:PATH = "C:\path\to\CameraSDK\windows\SDK\lib\win_x64;$env:PATH"
```

Install and run the Python sample:

```powershell
cd CameraSDK\windows\Sample\python
py -3 -m pip install .\lx_camera_py-1.3.3-py3-none-any.whl
py -3 .\demo.py
```

Before running `demo.py`, change the SDK library path in the script to the actual path, for example `C:\path\to\CameraSDK\windows\SDK\lib\win_x64\LxCameraApi.dll`, and replace the sample camera IP with the actual camera IP.

Build C# samples:

```powershell
cd CameraSDK\windows\Sample\C#
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

The C# sample calls `LxCameraApi.dll` through the `LxCameraApiNet.dll` wrapper. At runtime, make sure the directory containing `LxCameraApi.dll` is added to `PATH`, or copy the DLL to the application directory.

## Examples

### C/C++ Samples

| Sample | Linux | Windows | Description |
| --- | --- | --- | --- |
| `single_camera2` | Supported | Supported | Opens a single camera and obtains full-frame data. |
| `multi_cameras` | Supported | Supported | Opens multiple cameras and obtains stream data. |
| `frame_callback` | Supported | Supported | Obtains data through frame callbacks. |
| `application_obstacle_v1` | Supported | Supported | Enables obstacle avoidance and reads the result. |
| `application_obstacle_v2` | Supported | Supported | Enables obstacle avoidance V2 and reads the result. |
| `application_pallet` | Supported | Supported | Enables pallet positioning and reads pose output. |
| `application_location` | Supported | Supported | Enables visual localization and reads localization output. |
| `imu_callback` | Supported | Supported | Receives IMU data through callbacks. |
| `lidarpoint_callback` | Supported | Supported | Receives XYZIRT lidar point cloud data. |
| `arm_local_camera` | Supported | Not supported | Calls the SDK in the camera ARM-side environment. |

### Python and C# Samples

| Sample | Path | Description |
| --- | --- | --- |
| Python | `CameraSDK/linux/Sample/python`, `CameraSDK/windows/Sample/python` | Demonstrates device discovery, opening, streaming, image access, point cloud access, and frame rate statistics. |
| C# | `CameraSDK/windows/Sample/C#` | Demonstrates calling the wrapped APIs through `LxCameraApiNet.dll` for device opening, stream control, pointer data, and application algorithm output. |

## Core Development Flow

The basic native API call sequence is:

```text
DcGetDeviceList
DcOpenDevice
DcSetBoolValue / DcSetIntValue / DcSetFloatValue / DcSetStringValue
DcStartStream
DcSetCmd(LX_CMD_GET_NEW_FRAME) or DcRegisterFrameCallback
DcGetPtrValue
DcStopStream
DcCloseDevice
```

Common capabilities include:

| Capability | Description |
| --- | --- |
| Device management | Discover devices, open devices, close devices, and configure device IP. |
| Data acquisition | Obtain RGB images, depth images, amplitude images, full frames, and point cloud data. |
| Callback streaming | Supports frame data callbacks, camera status callbacks, and IMU data callbacks. |
| Parameter configuration | Supports integer, float, boolean, string, ROI, and extended control APIs. |
| Built-in algorithms | Supports obstacle avoidance, pallet positioning, visual localization, and custom application data output. |
| Runtime configuration | Supports logging, GPU switch, PTP switch, thread count, and JPEG decode method configuration. |

Supported camera open modes:

| Mode | Parameter |
| --- | --- |
| `OPEN_BY_INDEX` | Device index in the discovery list. When the device list changes, the same index may refer to another device. |
| `OPEN_BY_IP` | Camera IP address or `ip:port`. |
| `OPEN_BY_SN` | Camera SN. |
| `OPEN_BY_ID` | Camera device ID. |

For detailed API parameters, return values, enums, and structures, refer to `SDK/include/lx_camera_api.h`, `SDK/include/lx_camera_define.h`, `SDK/include/lx_camera_application.h`, and the corresponding developer guides.

## ROS and ROS2 Entry

| Framework | Workspace | Build command | Description |
| --- | --- | --- | --- |
| ROS | `CameraSDK/linux/Sample/ROS/lx_camera_node_ws` | `./build.sh` or `catkin_make` | Provides `lx_camera_node`, `lx_localization_node`, and `sensor_sim_node`. |
| ROS2 | `CameraSDK/linux/Sample/ROS2/lx_camera_node_ws` | `./build.sh` or `colcon build` | Provides ROS2 camera, localization, and simulation nodes. |

The ROS/ROS2 samples publish images, depth, amplitude, point clouds, IMU, frame rate, pallet, obstacle, and error information, and provide command, integer, float, boolean, and string service wrappers. For topics, services, and launch files, refer to the README files and source code in the corresponding workspaces.

## Runtime Notes

1. The camera and host must be on a reachable network segment. When using `OPEN_BY_IP`, confirm the camera IP, host NIC IP, and firewall rules.
2. On Linux, run `CameraSDK/linux/install.sh` first to keep headers, libraries, and `LD_LIBRARY_PATH` consistent.
3. On Windows, the process architecture must match the loaded DLLs. Use `win_x64` for 64-bit programs and `win_x86` for 32-bit programs.
4. The SDK library path and camera IP in Python samples are example values. Update them before running the sample.
5. After a device is opened, it is exclusively controlled by the current process. Call `DcStopStream` and `DcCloseDevice` before exit; if the process exits abnormally, the camera may not be available again until the heartbeat times out.
6. `CameraSDK/linux/set_socket_buffer_size.sh` configures Linux socket receive/send buffers; `CameraSDK/linux/set_firewall.sh` helps configure Linux firewall rules.

## Version

| Item | Version |
| --- | --- |
| SDK | `2.4.60.260126` |
| API macro | `V2.4.60,20260126` |
| Python wheel | `lx_camera_py-1.3.3-py3-none-any.whl` |
| ROS package | `1.0.0` |
| ROS2 package | `1.0.0` |
| LxDataProcess | `1.3.6.260113` |

## Support and Feedback

- Product knowledge base: [MRDVS Hub 2.0](https://hub.mrdvs.cn/)
- GitHub repository: [Lanxin-MRDVS/CameraSDK](https://github.com/Lanxin-MRDVS/CameraSDK)
- Issue feedback: [GitHub Issues](https://github.com/Lanxin-MRDVS/CameraSDK/issues)
- Product inquiry: [WhatsApp`+86 13370882355`](https://wa.link/9yw2hz{target=_blank})

---

# 中文

## 目录

- [概览](#概览)
- [产品线简介](#产品线简介)
- [文档索引](#文档索引)
- [平台支持](#平台支持)
- [目录结构](#目录结构)
- [路径说明](#路径说明)
- [快速开始](#快速开始)
- [示例程序](#示例程序)
- [核心开发流程](#核心开发流程)
- [ROS和ROS2入口](#ros和ros2入口)
- [运行注意事项](#运行注意事项)
- [版本信息](#版本信息)
- [支持与反馈](#支持与反馈)

## 概览

LxCameraSDK提供MRDVS相机设备二次开发所需的运行库、头文件、语言示例和ROS/ROS2驱动示例。开发者可以基于本SDK完成设备搜索、设备打开、数据流控制、图像和点云获取、IMU数据获取、内置应用算法结果读取，以及相机参数配置。

本文作为SDK入口文档，主要说明包结构、环境配置、示例编译和运行注意事项。接口参数、结构体定义和语言层封装细节，请参考对应开发指南和示例代码。

## 产品线简介

迈尔微视致力于让机器人“看懂”世界，专注移动机器人视觉传感器研发，提供 3D 视觉与 AI 算法相结合的软硬件一体化解决方案。浙江迈尔微视科技有限公司核心技术团队组建于 2016 年，是杭州蓝芯机器人技术股份有限公司的全资子公司。

迈尔微视面向具身智能机器人、割草机器人、商用清洁机器人、低速无人车、智能轮椅、工业移动机器人及仓储物流自动化等场景，提供稳定可靠的视觉感知能力。公司依托 iToF、dToF、双目结构光、AI 算法和多传感器融合技术，持续迭代视觉硬件与算法方案，助力机器人运行更安全、更稳定、更智能。

更多官方产品资料请访问 [MRDVS Hub 2.0](https://hub.mrdvs.cn/)。

### 视觉硬件产品


| 产品线 | 简介 | 典型应用 | 图片 |
| --- | --- | --- | --- |
| S 系列 dToF 雷视融合传感器 | 面向室内外复杂环境设计，融合固态激光雷达与视觉感知能力，支持高精度测距、语义避障和 SLAM 建图，最大测距 42 m，最大视场角 140°。 | 移动机器人 SLAM、语义避障、室内外环境感知 | <img src="./assets/S-series.png" alt="S Series" width="2000"> |
| M 系列 ToF 深度相机 | 基于 Sony iToF 芯片开发的 RGB-D 深度相机，面向最长 5 m 工作距离优化，可适应不同光照、纹理和实时检测需求。 | 托盘识别、料笼堆叠、体积测量、机械臂视觉引导、拆垛抓取 | <img src="./assets/M-series.png" alt="M Series" width="2000"> |
| V 系列视觉导航相机 | 融合 2D 激光雷达、视觉与 IMU 的全栈式嵌入式定位系统，不依赖外部预设标记即可输出稳定连续的定位数据。 | 室内无人叉车、商用清洁机器人、传统人工叉车数字化改造、仓储定位 | <img src="./assets/V-series.png" alt="V Series" width="2000"> |
| H 系列高精度结构光相机 | 基于双目编码结构光技术，面向高精度识别、抓取和实时三维反馈任务设计。 | 自动化生产、物流分拣、机器人操作、高精度识别与抓取 | <img src="./assets/H-series.png" alt="H Series" width="2000"> |


### 工业解决方案

| 解决方案 | 简介 | 图片 |
| --- | --- | --- |
| 工业移动机器人视觉感知 | 面向工业移动机器人场景，提供基于 3D 视觉感知的软硬件一体化方案，支持自主定位、智能避障、托盘识别与料笼堆叠等核心任务。 | <img src="./assets/industrial_mobile_robot.jpg" alt="工业移动机器人视觉感知" width="2000"> |
| 仓储物流自动化视觉 | 面向仓储感知系统场景，提供全面的三维感知方案，支持立库安全监控、人体安全防护、货物体积测量和库位状态检测，提升仓储物流管理效率。 | <img src="./assets/warehouse_logistics_automation.webp" alt="仓储物流自动化视觉" width="2000"> |
| 智能割草机器人感知 | 融合全固态激光雷达与视觉感知能力，支持车体周边感知、低矮障碍物检测、厘米级建图、实时定位和安全路径规划，提升割草机器人在复杂草坪环境中的可靠性。 | <img src="./assets/lawn_mowing_robot_perception.webp" alt="智能割草机器人感知" width="2000"> |
| 室外低速无人车感知 | 面向室外低速无人车场景，提供抗多机、抗多径和抗强光干扰的 RGB-D 感知能力，支持大视场覆盖、低延时点云输出和高精度时空同步。 | <img src="./assets/outdoor_low_speed_autonomous_vehicle.webp" alt="室外低速无人车感知" width="2000"> |
| 人形与四足机器人感知 | 面向人形和四足机器人，提供全景感知、近端避障、动态步态适配、地形三维建模和障碍目标识别能力，增强机器人在复杂场景中的灵活交互与环境理解能力。 | <img src="./assets/humanoid_quadruped_robot.webp" alt="人形与四足机器人感知" width="2000"> |
| 无人机全域视觉感知 | 采用纯固态激光雷达、视觉相机与 IMU 的多模态融合感知架构，支持高精度建图定位、全场景三维感知、语义级目标识别和安全避障，提升无人机飞行安全性与稳定性。 | <img src="./assets/uav_full_scenario_perception.webp" alt="无人机全域视觉感知" width="2000"> |

## 文档索引

- LxCameraSDK C/C++ 开发指南 [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_CN/MRDVS_LxCameraSDK_C-Cpp_%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97_V1.0_20260604.pdf) 
- LxCameraSDK Python 开发指南 [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_CN/python%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97.pdf)
- LxCameraSDK Linux 示例说明 [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_CN/Linux%20%E7%A4%BA%E4%BE%8B%E7%A8%8B%E5%BA%8F%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf) 
- LxCameraSDK 设备二次开发常见问题 [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_CN/Camera%20%E8%AE%BE%E5%A4%87%E4%BA%8C%E6%AC%A1%E5%BC%80%E5%8F%91%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E8%AF%B4%E6%98%8E.pdf)
- LxCameraViewer 用户手册 [Click here](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Document_CN/LxCameraViewer%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf)

## 平台支持

| 平台 | 包内架构 | 运行库 | 示例 |
| --- | --- | --- | --- |
| Linux | `linux_x64`、`linux_aarch64`、`linux_arm32` | `libLxCameraApi.so`、`libLxDataProcess.so` | C/C++、Python、ROS、ROS2 |
| Windows | `win_x64`、`win_x86` | `LxCameraApi.dll`、`LxCameraApi.lib`、`LxDataProcess.dll` | C/C++、Python、C# |

## 目录结构

```text
CameraSDK/
|-- Document_EN/
|-- Document_CN/
|-- linux/
|   |-- install.sh
|   |-- set_socket_buffer_size.sh
|   |-- set_firewall.sh
|   |-- SDK/
|   |   |-- include/
|   |   `-- lib/
|   |       |-- linux_x64/
|   |       |-- linux_aarch64/
|   |       `-- linux_arm32/
|   `-- Sample/
|       |-- C/
|       |-- python/
|       |-- ROS/
|       `-- ROS2/
|-- windows/
|   |-- SDK/
|   |   |-- include/
|   |   `-- lib/
|   |       |-- win_x64/
|   |       `-- win_x86/
|   `-- Sample/
|       |-- C/
|       |-- python/
|       `-- C#/
|-- README.md
`-- release_note.txt
```

## 路径说明

本文命令默认从`CameraSDK`父目录执行，`CameraSDK`为`git clone`后的目录名：

```bash
git clone https://github.com/Lanxin-MRDVS/CameraSDK.git
```

如果已经位于SDK根目录，请省略命令中的`CameraSDK/`前缀。如果使用压缩包或安装包解压目录，例如`CameraSDK-master`，将命令中的`CameraSDK`替换为实际目录名。

## 快速开始

### Linux

安装SDK运行文件并配置动态库路径：

```bash
cd CameraSDK/linux
source install.sh
```

`install.sh`会将头文件和库文件部署到`/opt/MRDVS`，并配置`/opt/MRDVS/lib`动态库路径。脚本需要root权限；非root执行时会提示是否使用`sudo`。

编译C/C++示例：

```bash
cd CameraSDK/linux/Sample/C
cmake -S . -B build
cmake --build build
```

运行示例：

```bash
./build/bin/demo_single_camera2
```

安装并运行Python示例：

```bash
cd CameraSDK/linux/Sample/python
python3 -m pip install ./lx_camera_py-1.3.3-py3-none-any.whl
python3 demo.py
```

运行`demo.py`前，请将脚本中的SDK库路径改为`/opt/MRDVS/lib/libLxCameraApi.so`，并将示例相机IP改为实际相机IP。

### Windows

编译C/C++示例：

```powershell
cd CameraSDK\windows\Sample\C
cmake -S . -B build
cmake --build build --config Release
```

运行示例前，请确认对应架构的DLL目录可被程序加载。64位程序使用`win_x64`，32位程序使用`win_x86`。

```powershell
$env:PATH = "C:\path\to\CameraSDK\windows\SDK\lib\win_x64;$env:PATH"
```

安装并运行Python示例：

```powershell
cd CameraSDK\windows\Sample\python
py -3 -m pip install .\lx_camera_py-1.3.3-py3-none-any.whl
py -3 .\demo.py
```

运行`demo.py`前，请将脚本中的SDK库路径改为实际路径，例如`C:\path\to\CameraSDK\windows\SDK\lib\win_x64\LxCameraApi.dll`，并将示例相机IP改为实际相机IP。

构建C#示例：

```powershell
cd CameraSDK\windows\Sample\C#
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

C#示例通过`LxCameraApiNet.dll`封装调用`LxCameraApi.dll`。运行时需要确保`LxCameraApi.dll`所在目录已加入`PATH`，或将DLL复制到程序目录。

## 示例程序

### C/C++示例

| 示例 | Linux | Windows | 说明 |
| --- | --- | --- | --- |
| `single_camera2` | 支持 | 支持 | 打开单台相机并获取完整帧数据。 |
| `multi_cameras` | 支持 | 支持 | 打开多台相机并获取数据流。 |
| `frame_callback` | 支持 | 支持 | 使用帧回调获取数据。 |
| `application_obstacle_v1` | 支持 | 支持 | 启用避障算法并读取结果。 |
| `application_obstacle_v2` | 支持 | 支持 | 启用避障算法V2并读取结果。 |
| `application_pallet` | 支持 | 支持 | 启用托盘定位算法并读取位姿输出。 |
| `application_location` | 支持 | 支持 | 启用视觉定位算法并读取定位输出。 |
| `imu_callback` | 支持 | 支持 | 使用回调接收IMU数据。 |
| `lidarpoint_callback` | 支持 | 支持 | 接收XYZIRT激光点云数据。 |
| `arm_local_camera` | 支持 | 不支持 | 在相机ARM侧环境调用SDK。 |

### Python和C#示例

| 示例 | 路径 | 说明 |
| --- | --- | --- |
| Python | `CameraSDK/linux/Sample/python`、`CameraSDK/windows/Sample/python` | 演示设备搜索、打开、启流、图像访问、点云访问和帧率统计。 |
| C# | `CameraSDK/windows/Sample/C#` | 演示通过`LxCameraApiNet.dll`调用封装接口，完成设备打开、流控制、指针数据和应用算法输出读取。 |

## 核心开发流程

原生接口的基本调用顺序如下：

```text
DcGetDeviceList
DcOpenDevice
DcSetBoolValue / DcSetIntValue / DcSetFloatValue / DcSetStringValue
DcStartStream
DcSetCmd(LX_CMD_GET_NEW_FRAME) 或 DcRegisterFrameCallback
DcGetPtrValue
DcStopStream
DcCloseDevice
```

常用能力包括：

| 能力 | 说明 |
| --- | --- |
| 设备管理 | 搜索设备、打开设备、关闭设备、配置设备IP。 |
| 数据采集 | 获取RGB图、深度图、强度图、完整帧和点云数据。 |
| 回调取流 | 支持帧数据回调、相机状态回调和IMU数据回调。 |
| 参数配置 | 支持整型、浮点、布尔、字符串、ROI和扩展控制接口。 |
| 内置算法 | 支持避障、托盘定位、视觉定位和自定义应用数据输出。 |
| 运行配置 | 支持日志、GPU开关、PTP开关、线程数和JPEG解码方式配置。 |

打开相机可使用以下模式：

| 模式 | 参数 |
| --- | --- |
| `OPEN_BY_INDEX` | 设备搜索列表中的索引。设备列表变化后，同一索引可能对应不同设备。 |
| `OPEN_BY_IP` | 相机IP或`ip:port`。 |
| `OPEN_BY_SN` | 相机SN。 |
| `OPEN_BY_ID` | 相机设备ID。 |

具体接口参数、返回值、枚举和结构体定义，请以`SDK/include/lx_camera_api.h`、`SDK/include/lx_camera_define.h`、`SDK/include/lx_camera_application.h`及对应开发指南为准。

## ROS和ROS2入口

| 框架 | 工作空间 | 构建命令 | 说明 |
| --- | --- | --- | --- |
| ROS | `CameraSDK/linux/Sample/ROS/lx_camera_node_ws` | `./build.sh`或`catkin_make` | 提供`lx_camera_node`、`lx_localization_node`和`sensor_sim_node`。 |
| ROS2 | `CameraSDK/linux/Sample/ROS2/lx_camera_node_ws` | `./build.sh`或`colcon build` | 提供ROS2版本相机节点、定位节点和模拟节点。 |

ROS/ROS2示例会发布图像、深度、强度、点云、IMU、帧率、托盘、避障和错误信息，并提供命令、整型、浮点、布尔、字符串等服务封装。具体话题、服务和launch文件请查看对应工作空间内的README和源码。

## 运行注意事项

1. 相机和主机需处于可通信网段。使用`OPEN_BY_IP`时，请确认相机IP、主机网卡IP和防火墙规则。
2. Linux下建议先执行`CameraSDK/linux/install.sh`，保证头文件、库文件和`LD_LIBRARY_PATH`配置一致。
3. Windows程序运行时必须加载与进程架构一致的DLL。64位程序使用`win_x64`，32位程序使用`win_x86`。
4. Python示例中的SDK库路径和相机IP是示例值，运行前必须按实际安装位置和设备IP修改。
5. 设备打开后会被当前进程独占。程序退出前应调用`DcStopStream`和`DcCloseDevice`释放资源；如果程序异常退出，相机可能需要等待心跳超时后才能重新打开。
6. `CameraSDK/linux/set_socket_buffer_size.sh`用于配置Linux socket收发缓冲区；`CameraSDK/linux/set_firewall.sh`用于辅助配置Linux防火墙规则。

## 版本信息

| 项目 | 版本 |
| --- | --- |
| SDK | `2.4.60.260126` |
| API宏定义 | `V2.4.60,20260126` |
| Python wheel | `lx_camera_py-1.3.3-py3-none-any.whl` |
| ROS package | `1.0.0` |
| ROS2 package | `1.0.0` |
| LxDataProcess | `1.3.6.260113` |

## 支持与反馈

- 产品知识库：[MRDVS Hub 2.0](https://hub.mrdvs.cn/)
- GitHub仓库：[Lanxin-MRDVS/CameraSDK](https://github.com/Lanxin-MRDVS/CameraSDK)
- 问题反馈：[GitHub Issues](https://github.com/Lanxin-MRDVS/CameraSDK/issues)
- 产品咨询：`400-025-6680`

