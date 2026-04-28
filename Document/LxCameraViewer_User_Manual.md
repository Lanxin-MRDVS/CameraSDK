**Table of Contents**

1. [Overview](#1-overview)
2. [Installation](#2-installation)
   - 2.1 [Recommended System Requirements](#21-recommended-system-requirements)
   - 2.2 [IP Address and Gateway Configuration](#22-ip-address-and-gateway-configuration)
   - 2.3 [Host Computer Installation](#23-host-computer-installation)
   - 2.4 [Firewall Configuration](#24-firewall-configuration)
3. [LxCameraViewer User Guide](#3-lxcameraviewer-user-guide)
   - 3.1 [Software Interface Overview](#31-software-interface-overview)
   - 3.2 [Open and Close Device](#32-open-and-close-device)
   - 3.3 [Current Information](#33-current-information)
   - 3.4 [Device Information](#34-device-information)
   - 3.5 [Image, Data Preview and Saving](#35-image-data-preview-and-saving)
   - 3.6 [Menu Bar](#36-menu-bar)
   - 3.7 [Function Bar](#37-function-bar)
4. [Accessing the Camera System](#4-accessing-the-camera-system)
5. [Tool Instructions](#5-tool-instructions)
   - 5.1 [IP Configuration Tool](#51-ip-configuration-tool)
   - 5.2 [Communication Tool](#52-communication-tool)
   - 5.3 [Network Card Configuration Tool](#53-network-card-configuration-tool)
6. [FAQ and Solutions](#6-faq-and-solutions)
7. [Help](#7-help)

---

## 1. Overview

LxCameraViewer is a Windows-based host application developed for MRDVS camera products and the camera SDK (C/C++ development kit).

The application provides camera users with convenient tools for viewing, camera configuration, algorithm settings, IP configuration, point cloud visualization, packet capture, communication, and firewall configuration. It supports simultaneous connection to multiple devices and can output depth images, amplitude images, point cloud images, and RGB images. It also supports reading and configuring basic camera parameters and algorithm settings.

Users can use this application to view and save high-precision image data captured from the camera, as well as detailed camera configuration and algorithm information.

## 2. Installation

### 2.1 Recommended System Requirements

**Windows**

| Item | Recommended Requirements |
|------|-------------------------|
| Operating System | Windows 10 (64-bit), Windows 11 (64-bit) |
| Memory | 4 GB or more |
| Network Card/Cable | Gigabit Ethernet or higher |

**Linux**

| Item | Recommended Requirements |
|------|-------------------------|
| Operating System | Ubuntu (64-bit) |
| Memory | 4 GB or more |
| Network Card/Cable | Gigabit Ethernet or higher |

### 2.2 IP Address and Gateway Configuration

- The camera uses a fixed IP address, which can be modified through this software when needed.
- Default camera IP address: `192.168.100.82`
- Default gateway: can be left empty
- Subnet mask: `255.255.0.0` (Windows); `255.255.255.0` (Linux)
- On the host computer, set the Ethernet adapter IP address connected to the camera to `192.168.100.x`, where x is any number between 3 and 254. The host IP and camera IP must not be the same.
- Example: When the camera IP is `192.168.100.12`, set the host IP address to `192.168.100.89` with a subnet mask of `255.255.0.0`, as shown below:

![IP Configuration Example](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.2/p1.png)

### 2.3 Host Computer Installation

1. Double-click the installer package: `Lanxin-MRDVS-x.x.x.xxxx.exe`
2. Click **Allow** when prompted to make changes to your device
3. Select the installation language (see Figure 1)
4. Select the installation path (using the default path is recommended) (see Figure 2)
5. Check **Create a desktop shortcut** (see Figure 3)
6. After installation completes, uncheck **Launch Lanxin-MRDVS** and click **Finish** (see Figure 4)
7. The first time you launch the software, run it as administrator. Right-click the desktop shortcut and select **Run as administrator** (see Figure 5)

| ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.3/p1.png) | ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.3/p2.png)  | ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.3/p3.png)  | ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.3/p4.png)  | ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.3/p5.png)  |
| :---: | :---: | :---: | :---: | :---: |
| *Figure 1* | *Figure 2* | *Figure 3* | *Figure 4*| *Figure 5*|

### 2.4 Firewall Configuration

To connect to the camera, the firewall must be disabled. The steps below use Windows 10 as an example:

1. Open the software and click **[Others]** at the top

   ![Firewall Step 1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.4/p1.png)
   
   *Figure 1*

2. Select **[Local network config]** in the panel that appears on the right

   ![Firewall Step 2](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.4/p2.png)
   
   *Figure 2*

3. In the Network Card Configuration Tool, click the **[Open/Close]** button shown below

   ![Firewall Step 3](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.4/p3.png)
   
   *Figure 3*

4. If a window like Figure 4 appears, the firewall is on. If a window like Figure 5 appears, the firewall is off.

   | ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.4/p4.png) | ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.4/p5.png) |
   | :---: | :---: |
   | *Figure 4*| *Figure 5* |

5. If the above method does not work, click **[Firewall Configuration Page]** and wait for the page to open.

   ![col1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/2.4/p6.png)

   *Figure 6*

> **The firewall must be turned off to use the camera.**


## 3. LxCameraViewer User Guide

### 3.1 Software Interface Overview

#### 3.1.1 Interface Overview (Camera Not Connected)

The initial interface consists of three main sections:

- **List Panel** (Area 1, blue)
  - Device List
  - Current Information
  - Device Information
- **Menu Bar** (Area 2, green)
  - Three toolbar buttons
  - Language switch button
- **Image Display Panel** (Area 4, red)
  - Point Cloud Display
  - Depth Image Display
  - Amplitude Image Display
  - RGB Image Display

Double-clicking the menu bar area maximizes or restores the window. Hold the left mouse button and drag to move the interface.

|![Interface Overview - Camera Not Connected](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.1/3.1.1.png)|![Interface Overview - Camera Connected](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.1/3.1.2.png)|
|:----:|:---:|
|Camera Not Connected|Camera Connected|

#### 3.1.2 Interface Overview (Camera Connected)

After opening a camera, an additional right-side toolbar appears (Area 3, magenta):

- **Basic Tools**, providing:
  - Image Display Settings
  - 2D Settings
  - 3D Settings
  - Filtering
  - Hardware Interface
  - Collection Settings
  - Function Settings
- **Apply Algorithms**, providing:
  - Working Mode Settings
  - Algorithm Mode Settings
  - Algorithm Version Display
  - Algorithm Detailed Settings
- **Others**, providing:
  - Load Local Point Cloud
  - IP Configuration Tool
  - Capture Tool
  - Network Card Configuration Tool
  - Communication Tool
  - GPU Enable
  - Log Level

### 3.2 Open and Close Device

#### 3.2.1 Device List

- The device list is located in the top-left corner of the interface (Area 1, red in Figure 1)
- The **Refresh** button is next to the device list (Area 2, red in Figure 2)
- The collapse/expand button is to the right of the Refresh button (Area 2, green in Figure 2). When collapsed, the list appears as shown in Figure 3
- The device list displays three properties:
  - Camera connection status
  - Camera ID (Area 2, yellow in Figure 2)
  - Camera IP (hover over the Camera ID to display the IP, shown in the red box in Figure 4)
- Each camera in the list has an **[Open/Close]** button to open or close the camera (blue box in Figure 2). The button also displays the current camera status (Figure 5)
- The selected camera is highlighted in blue; unselected cameras appear in white (Figure 6 shows the second camera selected but not opened, and the first camera opened but not selected)
- Multiple devices can be opened simultaneously (Figure 7). Ensure sufficient network bandwidth. Click a camera ID to switch between cameras

![Device List](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.2/p1.png)

*Figure 1*

| ![Refresh Button](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.2/p2.png) | ![Collapsed List](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.2/p3.png)  | ![IP Hover](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.2/p4.png)  | ![Camera Status](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.2/p5.png)  | ![Selected vs Opened](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.2/p6.png)  |![Multiple Devices](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.2/p7.png) |
| :---: | :---: | :---: | :---: | :---: | :---: |
|*Figure 2* | *Figure 3* | *Figure 4* | *Figure 5*| *Figure 6*| *Figure 7*|

### 3.3 Current Information

- Current information is located in the middle-left area of the interface (Area 1, red in Figure 1)
- It displays the following:
  - All camera information (Area 2, red in Figure 2)
  - Software operations (Area 2, green in Figure 2)
  - Operation results (Area 2, blue in Figure 2)
  - Streaming results (Area 4, magenta in Figure 4)
- When text overflows, drag the scrollbar at the bottom to view the full content (Figure 3 shows an additional scrollbar at the bottom)
- Current information displays a maximum of 25 entries and can be scrolled with the mouse wheel. Older entries are overwritten
- Click the collapse button to collapse this panel (Area 2, yellow in Figure 2)

![Current Information Location](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.3/p1.png)

*Figure 1*

| ![Current Information Details](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.3/p2.png) | ![Scrollbar](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.3/p3.png)  | ![Streaming Results](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.3/p4.png)  |
| :---: | :---: | :---: |
| *Figure 2* | *Figure 3* | *Figure 4* |

### 3.4 Device Information

- Device information is located in the bottom-left corner of the interface (Area 1, red in Figure 1)
- It displays information for the currently selected device in the device list, including:
  - **Serial Number**: Unique device identifier
  - **Camera ID**: Unique device identifier; used to open the device via SDK
  - **IP Address**: Device IP address
  - **MAC Address**: Device MAC address
  - **Firmware Version**: Built-in software version
  - **Algorithm Version**: Built-in algorithm version (visible after opening the camera)
  - **Network Speed**: Displays when the camera is connected; determined by the network card and cable (100M or Gigabit)
  - **Camera Type**: Device model
  - **Mirror Version**: Built-in mirror version (visible after opening the camera)
- Click the collapse button to collapse this panel (Area 2, yellow in Figure 2)
- Right-click a specific item to copy its value (Figure 3 shows copying network speed information)

![Device Information Location](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.4/p1.png)

*Figure 1*

| ![Device Information Details](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.4/p2.png) | ![Copy Item](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.4/p3.png)  |
| :---: | :---: |
| *Figure 2* | *Figure 3* |

### 3.5 Image, Data Preview and Saving

#### 3.5.1 Preview

- Image data and related operations are located in the center of the interface (Area 1, red in Figure 1)
- **[Start/Stop Flow]**, **[Save Single Image]**, **[Depth Image]**, **[Amplitude Image]**, **[RGB]**, and **[Point Cloud]** are image operation controls
- The images in the center are the depth, amplitude, point cloud, and RGB images captured from the device
- The white text at the bottom displays camera and image data (Figure 2)

![Preview Area](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p1.png)

*Figure 1*

![Image Data](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p2.png)

*Figure 2*

#### 3.5.2 Image (Flow) Controls

- **[Start/Stop Flow]**: Starts or stops the device stream
- **[Save Single Image]**: Saves the currently displayed image data
  - For example, in Figure 2, only the depth and point cloud images are saved; the RGB and amplitude images are not enabled and therefore not saved
  - File extensions: `.pgm` for depth and amplitude images, `.png` for RGB images, `.pcd` for point cloud images
  - Saved depth, amplitude, and point cloud images are raw data without pseudo-coloring
  - Multiple saves are supported, but no more than three consecutive saves are allowed
- **[Depth Image]** checkbox: Enables or disables the depth image display
- **[Amplitude Image]** checkbox: Enables or disables the amplitude image display
- **[Point Cloud]** checkbox: Enables or disables the point cloud image display
- **[RGB]** checkbox: Enables or disables the RGB image display
- Unchecking RGB/Amplitude stops the device from capturing those image types. However, the device only stops capturing depth data when both the depth image and point cloud image are disabled
- In Figure 2, the device is streaming and displaying depth, point cloud, RGB, and amplitude images
- In Figure 3, the device has stopped streaming and no images are displayed

![Streaming Enabled](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p3.png)

*Figure 2*

![Streaming Stopped](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p4.png)

*Figure 3*

#### 3.5.3 Image Data

- Displays the camera temperature (Area 4, red in Figure 4)
- When the depth image is selected, shows the depth image frame rate (Area 4, green in Figure 4)
- When the amplitude image is selected, shows the amplitude image frame rate (Area 4, blue in Figure 4)
- When the RGB image is selected, shows the RGB image frame rate (Area 4, magenta in Figure 4)
- When the depth image is selected, shows the pixel values of the selected pixels in the depth image (Area 4, yellow in Figure 4)
- When the amplitude image is selected, shows the pixel values of the selected pixels in the amplitude image (Area 4, orange in Figure 4)
- When the point cloud image is selected, shows the point cloud values corresponding to the selected pixels in the depth image (Area 4, black in Figure 4)
- When no pixels are selected, the default selection is pixel (0, 0)
- Frame rate color indicators: green (> 12 fps), orange (6–12 fps), red (< 6 fps)

![Image Data Display](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p5.png)

*Figure 4*

#### 3.5.4 Image Display

Available display modes:

- **Single Frame**: Displays one image when one of the four types is selected (Figure 5)
- **Dual Frame**: Displays two images when two types are selected (Figure 6)
- **Triple Frame**: Displays three images when three types are selected (Figure 7)
- **Quadruple Frame**: Displays all four images when all types are selected (Figure 8)

Mouse operations:
- Scroll the mouse wheel to zoom in/out, centered on the cursor position
- Hold the left mouse button and drag to pan
- Left-click to select a pixel; use arrow keys to move the selection (Figure 6)

| ![Single Frame](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p6.png) | ![Dual Frame](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p7.png) | ![Triple Frame](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p8.png) | ![Quadruple Frame](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.5/p9.png) |
| :---: | :---: | :---: | :---: |
| *Figure 5* | *Figure 6* | *Figure 7* | *Figure 8* |

#### 3.5.5 Point Cloud Operations

- Hold the **left mouse button** and drag to rotate the camera view
- Hold the **middle mouse button** and drag to move the focal point
- Hold the **right mouse button** and drag up/down, or use the mouse wheel, to zoom in/out

**Point Cloud Keyboard Shortcuts**

| Shortcut | Function |
|:----------:|----------|
| W | Display grid model; press S to switch |
| R | Reset camera view |
| S | Display surface model; press W to switch |
| P | Hide all lines, surfaces, and grids; press W or S to restore |
| U | Call user-defined function (disabled) |
| O | Eliminate fisheye effect; press again to restore |
| F | Zoom to mouse position and set it as rotation center |
| G | Display scale and ratio; press again to hide |
| X | Disable mouse operation on point cloud; press again to restore |
| M | Display point cloud bounding box; press again to show point cloud |
| A | Decrease point cloud thickness (minimum: 1) |
| D | Increase point cloud thickness (maximum: 6) |
| E | Return to initial view |
| [ | Rotate to front view (average center point as focus); not supported in obstacle avoidance mode |
| ] | Rotate to back view (average center point as focus); not supported in obstacle avoidance mode |
| ; | Rotate to left side view (average center point as focus); not supported in obstacle avoidance mode |
| ' | Rotate to right side view (average center point as focus); not supported in obstacle avoidance mode |
| . | Rotate to top view (average center point as focus); not supported in obstacle avoidance mode |
| / | Rotate to bottom view (average center point as focus); not supported in obstacle avoidance mode |

### 3.6 Menu Bar

The menu bar includes **[Basic]**, **[Algorithms]**, **[Others]**, and **[Language]** (Figure 1).

- Clicking **[Basic]** displays the options shown in Figure 2
- Clicking **[Algorithms]** displays the options shown in Figure 3
- Clicking **[Others]** displays the options shown in Figure 4
- Clicking **[Language]** displays the options shown in Figure 5

![Menu Bar](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.6/p1.png)

*Figure 1*

| ![Basic Menu](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.6/p2.png) | ![Algorithms Menu](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.6/p3.png) | ![Others Menu](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.6/p4.png) | ![Language Menu](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.6/p5.png) |
| :---: | :---: | :---: | :---: |
| *Figure 2* | *Figure 3* | *Figure 4* | *Figure 5* |


### 3.7 Function Bar

#### 3.7.1 Display Functions

- Collapse/expand the right sidebar (Area 1, red in Figure 1)
- **Expand All** and **Collapse All** buttons (Area 1, blue in Figure 1; upper icon collapses all, lower icon expands all)
- Click a function name to display the corresponding SDK setting information (highlighted in blue at the bottom of Figure 1)
  - For example, clicking **undistort** in 2D Settings (green box text) displays the API interface and enumeration values in the blue box
- Expanded and collapsed states are shown in Figure 1 (2D Settings is expanded; others are collapsed)

![Display Functions](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.1.png)

*Figure 1*

#### 3.7.2 Basic Tools

Clicking **[Basic]** on the menu bar displays the basic tools settings on the right side of the interface (Figure 1).

Basic tools include **[Image Display Settings]**, **[2D Settings]**, **[3D Settings]**, **[Filtering]**, **[Hardware Interface]**, **[Collection Settings]**, and **[Function Settings]** (Figure 2). Function names and enumeration values are displayed in the blue box at the bottom (Figure 2).

![Basic Tools](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p1.png)

*Figure 1*

![Basic Tools Details](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p2.png)

*Figure 2*

##### 3.7.2.1 Image Display Settings

| Setting                             | Description                                                          |
| ----------------------------------- | -------------------------------------------------------------------- |
| **[Display Camera Model]**         | Toggle camera model overlay on point cloud (Figures 4–5)             |
| **[Display Pseudo Color]**         | Apply depth pseudo-color; switches to RGB when alignment is enabled  |
| **[Point Stability]**              | Stabilize point cloud rendering on systems with compatibility issues |
| **[Sync Up Image]**                | Synchronize image display with camera feed                           |
| **[Preserve Interference Frames]** | Retain interference frames in output stream                          |
| **[Minimum Depth]**                | Set minimum depth threshold for display (output unaffected)          |
| **[Maximum Depth]**                | Set maximum depth threshold for display (output unaffected)          |
| **[Minimum Amp]**                  | Set minimum amplitude threshold for display (output unaffected)      |
| **[Maximum Amp]**                  | Set maximum amplitude threshold for display (output unaffected)      |

![Image Display Settings](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p3.png)

*Figure 3*

| ![Camera Model Shown](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p4.png) | ![Camera Model Hidden](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p5.png) |
| :---: | :---: |
| *Figure 4* | *Figure 5* |

##### 3.7.2.2 2D Settings

| Setting | Description |
|---------|-------------|
| **[RGB Width]** | RGB image pixel width; determined by the device model, read-only |
| **[RGB Height]** | RGB image pixel height; determined by the device model, read-only |
| **[Binning]** | Combines pixel values in a 1×1, 2×2, or N×N matrix, then divides by the matrix size to produce the binned image (image dimensions become original size divided by binning factor) |
| **[Undistort]** | Retrieves distortion parameters from intrinsic parameters to remove distortion from the displayed image |
| **[Undistort Scale]** | Adjusts the distortion coefficient to increase or decrease the degree of distortion correction |
| **[Auto Exposure]** | Enables or disables automatic RGB exposure adjustment by the camera |
| **[Exposure]** | Manual exposure; configurable when auto exposure is disabled |
| **[Gain]** | Manual gain; configurable when auto exposure is disabled |

![2D Settings](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p6.png)

*Figure 6*

##### 3.7.2.3 3D Settings

| Setting | Description |
|---------|-------------|
| **[3D Width]** | Point cloud image width; determined by the device model, read-only |
| **[3D Height]** | Point cloud image height; determined by the device model, read-only |
| **[Binning]** | Combines pixel values in a 1×1, 2×2, or N×N matrix, then divides by the matrix size to produce the binned image (image dimensions become original size divided by binning factor) |
| **[RGB-D Alignment]** | Aligns the depth image, amplitude image, and ToF points with the RGB image. After alignment, depth and amplitude dimensions match the RGB image, and ToF point data increases |
| **[Multi machine Mode]** | When using multiple cameras simultaneously, enables interference elimination from other cameras. Can be modified after stopping the stream |
| **[Multi machine level]** | Built-in device parameter; determined by the device model, read-only |
| **[Undistort]** | Retrieves distortion parameters from intrinsic parameters to remove distortion from the displayed image |
| **[Undistort Scale]** | Adjusts the distortion coefficient to increase or decrease the degree of distortion correction |
| **[Frequency Mode]** | Built-in device parameter; determined by the device model (some cameras have fixed frequency, read-only) |
| **[HDR Enable]** | HDR improves image clarity in scenes with strong lighting changes |
| **[Auto Exposure]** | When enabled, the device automatically adjusts exposure parameters based on ambient light intensity |
| **[Target value]** | Intensity threshold during depth image acquisition (read-only) |
| **[Auto Exposure MAX]** | Sets the upper limit for auto exposure |
| **[Auto Exposure MIN]** | Sets the lower limit for auto exposure (read-only) |
| **[Gain]** | Increases intensity value by multiples (read-only) |
| **[x]** | Sets the x-coordinate of the top-left corner of the ROI area; can be modified after stopping the stream |
| **[y]** | Sets the y-coordinate of the top-left corner of the ROI area; can be modified after stopping the stream |
| **[High Integration]** | High integration time (increasing allows the camera to measure farther distances) |
| **[Low Integration]** | Low integration time (controls distance measurement for nearby objects; typically does not need adjustment) |
| **[x Offset]** | Offset value in the width direction; can be modified after stopping the stream |
| **[y Offset]** | Offset value in the height direction; can be modified after stopping the stream |
| **[Width]** | Sets the width of the ROI area; can be modified after stopping the stream |
| **[Height]** | Sets the height of the ROI area; can be modified after stopping the stream |
| **[3D Frame Rate]** | Sets the point cloud image frame rate within the device frame rate range |

![3D Settings](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p7.png)

*Figure 7*

##### 3.7.2.4 Filtering Settings

| Setting | Description |
|---------|-------------|
| **[Minimum Depth]** | Lower limit for pseudo-color or distance in the depth image. Below this value, pseudo-color is displayed as black and the value is set to zero |
| **[Maximum Depth]** | Upper limit for pseudo-color or distance in the depth image. Above this value, pseudo-color is displayed as black and the value is set to zero |
| **[Minimum Amplitude]** | Lower limit of intensity values. Below this value, the corresponding pixel in the depth image is set to zero (does not affect the amplitude image) |
| **[Maximum Amplitude]** | Upper limit of intensity values. Above this value, the corresponding pixel in the depth image is set to zero (does not affect the amplitude image) |
| **[low signal detect]** | Sets the small signal detection level; higher levels detect smaller signals |
| **[Filtering Mode]** | Sets the filter masking level |
| **[Smooth Level]** | Controls the smoothing strength of spatial filtering, preserving edge details while reducing noise |
| **[Noise Level]** | Sets the noise filtering threshold (higher value = more noise filtered) |
| **[Time Level]** | Controls the strength of temporal filtering, using historical multi-frame data to improve depth data continuity |

> **Note:** Setting Smooth Level, Noise Level, and Time Level all to 0 disables filtering.

![Filtering Settings](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p7.png)

*Figure 8*

##### 3.7.2.5 Hardware Interface

| Setting | Description |
|---------|-------------|
| **[IO Output State]** | Sets the IO interface output format |
| **[IO User Control Mode]** | Built-in device parameter defining the IO interface; read-only |
| **[CAN Baud Rate]** | Default: 250000; must match the baud rate of other devices on the CAN bus |
| **[CAN Protocol Type]** | Defaults to MRDVS official protocol; can be changed to kcrobots |
| **[CAN Address]** | CAN communication start address; default is 18 |
| **[MODBUS Addr]** | MODBUS communication start address; default is 23 |
| **[Get IO Status]** | Retrieves the current IO interface return status |

![Hardware Interface](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p9.png)

*Figure 9*

##### 3.7.2.6 Collection Settings

| Setting | Description |
|---------|-------------|
| **[Sync Frame]** | Controls consistency of environmental conditions when capturing various image types |
| **[Trigger Mode]** | Sets the streaming mode to continuous, passive, or active |
| **[Cloud Save Method]** | **Normal** (Unordered Point Cloud): point count unchanged, width = width × height, height = 1 (Figure 3). **Ordered Point Cloud**: point count unchanged, width = ToF width, height = ToF height (Figure 4) |
| **[Cloud Save Format]** | **BINARY**: saves in binary format. **ASCII**: saves in text-based ASCII format |
| **[Point Cloud numeric Units]** | Sets the measurement unit for point cloud saved data |
| **[Start Snap]** | Continuously saves the currently selected image type |
| **[Open Save Path]** | Opens the folder where **[Save Single Image]** stores files |

> **Note:** For a 640×480 image, ASCII saving takes approximately 6.6 seconds; binary saving takes approximately 32 ms. Binary saving is ~206× faster than ASCII. Binary format is recommended.

*Figure 10*

![Collection Settings](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p10.png)

##### 3.7.2.7 Function Settings

| Setting | Description |
|---------|-------------|
| **[Select param group]** | Select a previously saved camera parameter configuration |
| **[Save param group]** | Manually save the current camera parameter configuration |
| **[Load param group]** | Apply a previously saved camera parameter configuration |
| **[Export to File]** | Export the current camera parameter configuration as a JSON file |
| **[Import from File]** | Import a JSON camera parameter configuration file and apply it |
| **[User custom ID]** | Customize the ID for the current parameter configuration |
| **[Calculate up]** | Migrate the current built-in algorithm to run on the host platform |
| **[Get intrinsic Param]** | View the camera's built-in parameter configuration |
| **[Reset settings]** | Restore the camera to factory defaults. This erases all configured settings, including the camera IP. The camera will automatically shut down and must be manually reopened |
| **[Update Version]** | Upgrade the camera firmware. After clicking, select a `.bin` file in the file dialog |
| **[Camera Restart]** | Restart the camera. The camera will not shut down but will enter a reconnection state. Do not perform other operations during reconnection |

![Function Settings](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.2/p11.png)

*Figure 11*

#### 3.7.3 Apply Algorithms

Clicking **[Algorithms]** on the menu bar displays the algorithm settings on the right side of the interface (Figure 1).

- **[Working Mode]**: Allows the camera to continue outputting algorithm results even when disconnected from the host computer
- **[Algorithm]**: Select the algorithm to enable:
  - **[Close]** (Figure 2)
  - **[Obstacle Avoidance settings 2.0]** (Figure 3)
  - **[Tray Identification settings]** (Figure 4)
- **[Algorithm Ver.]**: Displays the version number of the currently selected algorithm

![Algorithm Settings](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.3/p1.png)

*Figure 1*

| ![Close Algorithm](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.3/p2.png) | ![Obstacle Avoidance](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.3/p3.png) | ![Tray Identification](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.3/p4.png) |
| :---: | :---: | :---: |
| *Figure 2* | *Figure 3* | *Figure 4* |

##### 3.7.3.1 Obstacle Avoidance Settings

When the application algorithm is set to **Obstacle Avoidance**, the following obstacle avoidance parameters are displayed:

| Parameter | Description |
|-----------|-------------|
| **[Show Vehicle]** | Enable or disable display of the vehicle model indicating the body position in the point cloud |
| **[Obstacle Avoidance Index]** | Multiple obstacle avoidance configurations are available; sets the active configuration |
| **[Camera Angle Roll (x-axis)]** | Camera extrinsic parameter, roll angle. Default for upright installation: 180° |
| **[Camera Angle Pitch (y-axis)]** | Camera extrinsic parameter, pitch angle. Default for vertical installation: 90° |
| **[Camera Angle Yaw (z-axis)]** | Camera extrinsic parameter, yaw angle. Default: 90° |
| **[Camera Position (Distance x)]** | Camera extrinsic parameter, distance in x direction from the origin. Unit: meters |
| **[Camera Position (Width y)]** | Camera extrinsic parameter, distance in y direction from the origin. Zero if center-mounted. Unit: meters |
| **[Camera Position (Height z)]** | Camera extrinsic parameter, distance in z direction from the origin (camera height from the ground). Unit: meters |
| **[Early Warning Distance (x-far)]** | Obstacle avoidance range in x direction from the vehicle center. Obstacles beyond this distance are safe. The area between early warning and alarm distances is the early warning deceleration zone (yellow). Unit: meters |
| **[Alarm Distance (x-mid)]** | Obstacle avoidance range in x direction from the vehicle center. Obstacles within this distance trigger an alarm stop (red). The area between early warning and alarm distances is the early warning deceleration zone (yellow). Unit: meters |
| **[Shielding Distance (x-near)]** | Obstacle avoidance range in x direction from the vehicle center. Obstacles within this distance are ignored. Unit: meters |
| **[Obstacle Avoidance Range Left (y-left)]** | Obstacle avoidance range to the left of the vehicle center in y direction. Unit: meters |
| **[Obstacle Avoidance Range Right (y-right)]** | Obstacle avoidance range to the right of the vehicle center in y direction. Unit: meters |
| **[Obstacle Avoidance Height Lower (z-lower)]** | Obstacle avoidance height from the ground. Obstacles below this height are ignored. If set too low, false alarms may occur due to ranging errors; if set too high, larger ground obstacles may be filtered out, causing missed alarms. Unit: meters |
| **[Obstacle Avoidance Height Upper (z-upper)]** | Obstacle avoidance height from the ground. Obstacles above this height are ignored. Unit: meters |
| **[Obstacle Avoidance Output]** | Output obstacle avoidance results |
| **[Reset Camera Extrinsics]** | Restore all parameters that have not been written to the camera |
| **[Set Extrinsics]** | Write all parameters to the camera |

For detailed obstacle avoidance settings, refer to the obstacle avoidance documentation.

##### 3.7.3.3 Tray Identification Settings

When the application algorithm is set to **Pallet Positioning**, the following pallet positioning parameters are displayed:

| Parameter | Description |
|-----------|-------------|
| **[Project Name]** | Assign a name to the current pallet recognition parameter configuration |
| **[Quick Configuration]** | Default configurations provided by the camera |
| **[Camera Angle Roll (x-axis)]** | Tilt angle of the camera relative to the X-axis |
| **[Camera Angle Pitch (y-axis)]** | Tilt angle of the camera relative to the Y-axis |
| **[Camera Angle Yaw (z-axis)]** | Tilt angle of the camera relative to the Z-axis |
| **[Camera Position (Distance x)]** | Relative distance in X direction between the camera optical center and the forklift rotation center |
| **[Camera Position (Width y)]** | Relative distance in Y direction between the camera optical center and the forklift rotation center |
| **[Camera Position (Height z)]** | Relative distance in Z direction between the camera optical center and the forklift rotation center |
| **[Ground Height]** | Distance from the camera optical center to the ground |
| **[Max Pallet Leg Width]** | Maximum width of the pallet leg among compatible pallet types |
| **[Min Pallet Leg Width]** | Minimum width of the pallet leg among compatible pallet types |
| **[Pallet Distance]** | Distance from the camera optical center to the pallet after fork insertion is completed |
| **[Installation Method]** | Set according to the camera mounting orientation; default is upright installation (0) |
| **[Min Z Value]** | Minimum depth distance for pallet recognition |
| **[Max Z Value]** | Maximum depth distance for pallet recognition |
| **[Min X Value]** | Horizontal boundary for pallet recognition (left boundary relative to the camera center) |
| **[Max X Value]** | Horizontal boundary for pallet recognition (right boundary relative to the camera center) |
| **[Pallet Algorithm Output]** | Display the results of the pallet algorithm |
| **[Calibration Instructions]** | Automatic calibration is available when the ground is flat, there are no obstructions in front of the pallet, and the pallet is of standard size |
| **[Restore Default Parameters]** | Restore all parameters to factory defaults |
| **[Read Extrinsics from Camera]** | Read all parameters from the camera; all parameters that have not been deployed will be overwritten |
| **[Deploy Parameters]** | Write all parameters to the camera |

For detailed pallet detection settings, refer to the pallet detection function documentation.

#### 3.7.4 Others

| Setting | Description |
|---------|-------------|
| **[Load Point Cloud]** | Open and view point cloud files in `.pcl` or `.png` format; see tool documentation for details |
| **[Camera IP Configuration Tool]** | Configure the camera IP address, subnet mask, and gateway |
| **[Packet Capture Tool]** | Capture network card packet data; see tool documentation for details |
| **[Local network config]** | Network card and firewall settings; see tool documentation for details |
| **[Communication Tool]** | Supports TCP, UDP, MODBUS, and FTP communication; see tool documentation for details |
| **[GPU Enable]** | Enable GPU-accelerated graphics processing |
| **[Log Level]** | Control the verbosity of log output |

![Others Menu](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.4/p1.png)

*Figure 1*

![Others Details](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/3.7/3.7.4/p2.png)

*Figure 2*

## 4. Accessing the Camera System

- Connect to the camera IP address via SSH on port 22
  - Example using MobaXterm with camera IP `192.168.100.15` (Figures 1 and 2)
- Username: `user`, Password: `user`
- After successful login, the interface shown in Figure 3 will be displayed

**Built-in System Configurations**

| Built-in Configuration | Version |
|----------------------|---------|
| OpenCV | 4.5.0 |
| OpenMPI | 4.0.5 |
| PCL | 1.12.1 |
| VTK | 8.1.2 |

> **Warning:** Do not modify system built-in configurations arbitrarily, as this may cause system instability.

| ![SSH Connection 1](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/4/p1.png) | ![SSH Connection 2](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/4/p2.png) | ![SSH Terminal](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/4/p3.png) |
| :---: | :---: | :---: |
| *Figure 1* | *Figure 2* | *Figure 3* |

## 5. Tool Instructions

### 5.1 IP Configuration Tool

- Open **[Camera IP Configuration Tool]**; the interface is shown in Figure 1
- Double-click the value after the MAC address to copy
- Double-click the values after IP address, subnet mask, and gateway to edit
- Click elsewhere to cancel modifications; press Enter to confirm (Figure 2)

| ![IP Config Tool](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/5.1/p1.png) | ![IP Config Edit](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/5.1/p2.png) |
| :---: | :---: |
| *Figure 1* | *Figure 2* |

### 5.2 Communication Tool

- Open **[Communication Tool]**; the interface is shown in Figure 1
- Supports TCP, UDP, FTP, Modbus TCP, Modbus RTU, and CAN (Linux)
- Supports multiple tabs

![Communication Tool](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/5.4.png)

*Figure 1*

### 5.3 Network Card Configuration Tool

The Network Card Configuration Tool provides the following features:

- Configure firewall settings
- After selecting a network card, open the Windows built-in network adapter properties and configuration interface
- Open the firewall configuration page
- Open Task Manager
- Open Network Connections Center
- Language automatically follows the application language setting

![Network Card Config](https://pub-f1dc758884b94eaba9d8c5d3d7575b62.r2.dev/lxcameraviewer/5.5.png)

*Figure 1*

## 6. FAQ and Solutions

| Problem | Solution |
|---------|----------|
| Camera Not Found | - Check that camera power is connected; the blue LED on the front of the camera should be constantly on after power-on<br>- Check that the IP address and subnet mask are correct<br>- Check that the host computer network is not blocked |
| Camera Opens but Stream Fails | Run the host computer software with administrator privileges |
| Image Stuttering | Check that the network card, cable, and switch support Gigabit Ethernet. After enabling RGB alignment, 100M networks may experience stuttering |
| Camera Reconnects Frequently | After factory reset, firmware upgrade, or first-time connection on a 100M network, the camera performs self-adjustment and restarts for approximately 30 seconds. During this period the host computer will keep trying to connect until the camera stabilizes |
| No Depth Data or Point Cloud Data | - Check that the camera is not obstructed<br>- Check that the subject is within the camera's measurement range<br>- Check that the max/min depth values are correct<br>- Check that the filter parameters are correct |
| Point Cloud Disappears, Only White Frame Visible | Click the point cloud image and press the **M** key to enable point cloud stabilization |
| Point Cloud Display Abnormal | Check if point cloud shortcut keys were pressed; enable point cloud stabilization |
| Point Cloud Not Displayed After Opening Camera on Linux | Click and drag the point cloud image with the mouse to display it |
| VTK loading failed, point cloud will not be displayed; please check graphics card configuration and drivers | - Check that the graphics card meets the minimum requirements<br>- Check that the graphics card driver is installed and functioning<br>- Point cloud display requires a dedicated graphics card with proper driver support |

## 7. Help

### 7.1 About

Click **[About]** in the upper right corner, or view the current information panel when the software starts, to see the software version, SDK version, and related information. Click **[Open Help Document]** to open this document.

### 7.2 User Manual

Right-click the software shortcut → **Open file location** → Go to the parent directory → All related documents can be found in the **Document** folder.

[Others Documents](https://github.com/Lanxin-MRDVS/CameraSDK/tree/master/Document)

---

*Last updated: April 2026*  
*Hangzhou Lanxin Technology Co., Ltd. & MRDVS Co., Ltd.*  
*All Rights Reserved.*
