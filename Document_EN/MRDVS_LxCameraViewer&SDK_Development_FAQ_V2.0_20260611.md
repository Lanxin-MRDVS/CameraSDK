<p align="center">
  <img src="../assets/mrdvs_logo.png" alt="MRDVS Logo" width="300">
</p>

<h1 align="center">LxCameraViewer & SDK Development FAQ</h1>

<p align="center">
  <a href="https://mrdvs.com/">Official Website</a> |
  <a href="https://hub.mrdvs.cn/">Knowledge Base</a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/document-FAQ-2563EB" alt="Document">
  <img src="https://img.shields.io/badge/version-V2.0-0F172A" alt="Version">
  <img src="https://img.shields.io/badge/date-2026--06--11-475569" alt="Date">
  <img src="https://img.shields.io/badge/platform-Windows%20%7C%20Linux-0891B2" alt="Platform">
</p>

---

## Contents

- [Overview](#overview)
- [Quick Check Sequence](#quick-check-sequence)
- [PC Host Software and Vision Imaging Issues](#pc-host-software-and-vision-imaging-issues)
- [SDK Error Codes and Issue Handling](#sdk-error-codes-and-issue-handling)
- [Items to Be Supplemented](#items-to-be-supplemented)

## Overview

This document is compiled from common issues in the company knowledge base. It is mainly intended for LxCameraSDK-based development, LxCameraViewer host-side debugging, and preliminary issue handling in field scenarios.

Items whose causes or solutions are not clearly stated in the original records are marked as "To be supplemented" or "Not specified in the original record". For issues involving hardware replacement, log analysis, or version upgrade, it is recommended to confirm the device SN, firmware version, SDK version, host software version, and device logs together.

## Quick Check Sequence

When the camera fails to open, no stream data is received, no image is displayed, or point cloud data is abnormal, check the following items first:

1. Confirm that the camera and the computer are on the same network segment, and that the host can ping the camera IP address.
2. Disable the firewall, or allow LxCameraViewer, the development application, and related communication ports through the firewall.
3. Check the network cable, switch, network adapter configuration, and packet loss.
4. Confirm whether multiple processes or multiple users are connected to the same camera at the same time.
5. Confirm whether the host software, SDK, firmware, and Dataprocess algorithm library versions are compatible.
6. Save the device logs, host software logs, current parameters, and issue screenshots for further analysis.

## PC Host Software and Vision Imaging Issues

| No. | Symptom | Possible Cause | Recommended Action |
| --- | --- | --- | --- |
| 1 | No data is received after the camera is opened in the host software, and the host software reports that no data stream is received. | The firewall is enabled; the network adapter and camera are not on the same network segment; the switch blocks communication; the customer's computer is abnormal. | 1. Disable the firewall.<br>2. Set the camera and computer IP addresses to the same network segment.<br>3. Check the switch.<br>4. Try another computer for verification.<br>5. Configure multiple network segments if needed. |
| 2 | Standard rectangular blocks are missing in the depth observation data, and black rectangular holes appear in the depth image. | False multi-camera interference detection. | Stop streaming, then disable multi-camera mode in the 3D settings. The algorithm mode must be changed to "disable when disconnected" first. |
| 3 | The S10/S10 pro camera does not display images after being opened in the software, or the depth image appears compressed and very small. | The host software version is not compatible. | Update to `Lanxin-MRDVS-1.3.66.0606` or a later version. |
| 4 | Failed to open the camera, with a network communication error. | The camera and the computer are not on the same network segment. | Change the camera or computer network segment so that they are on the same network segment. |
| 5 | Failed to obtain exclusive application permission. | Multiple users are connected to the camera. | Ensure that only one user connects to the camera at the same time. Version `2.4.50.0720` and later will enter the multi-user permission logic. |
| 6 | The camera device is opened successfully in restricted mode, but the host software or other processes may have no data stream. | Multiple users are connected to the camera. | When multiple processes connect to the same camera device at the same time, the host software or other processes may receive no data stream. The primary permission process, namely the first process that opens the camera device, must start the data stream first. Devices opened in restricted mode can then view the data stream. |
| 7 | The detected object is sparsely imaged or partially missing. | Insufficient exposure or excessive filtering. | 1. Check the high-exposure parameter. It is usually set to `600-1200`.<br>2. Check the low signal threshold. It is usually set to `10-30`. |
| 8 | Streaming starts normally in the host software, no image data is displayed, and the frame rate is normal. | The laser is weak or off. | Use a mobile phone camera to capture the front face of the camera and confirm the laser status. If the laser is off or obviously weak, replace the camera hardware. |
| 9 | No object is present in the front space, but point cloud data exists. | To be supplemented. | To be supplemented. It is recommended to save the on-site point cloud, camera parameters, and logs for further confirmation. |
| 10 | Noise data appears on the side edge of the measured object when the object faces the camera directly. | Glare. | 1. Enable the glare optimization algorithm.<br>2. Increase the low signal threshold. |
| 11 | The center of the RGB image appears reddish, or the RGB image is blurred. | Not specified in the original record. | Replace the hardware. |
| 12 | The RGB image is front-facing, while the depth image appears skewed. | Not specified in the original record. | Replace the hardware. |
| 13 | Point cloud data between adjacent objects is connected or stuck together. | Not specified in the original record. | Increase the denoising level and smoothing level in the filter settings. |
| 14 | No RGB data is available after RGBD alignment is enabled. | Software version issue. | Update the software version. It can be downloaded from [CameraSDK Releases](https://github.com/Lanxin-MRDVS/CameraSDK/releases). |
| 15 | Obvious layering appears in the point cloud, applicable to the S Series. | Not specified in the original record. | Replace the hardware. |
| 16 | The edge accuracy of nearby white objects is poor, applicable to the S Series. | Glare. | Enable the glare optimization algorithm. Note that black object detection may become incomplete after this option is enabled. |
| 17 | Large-area overexposure appears at the reflective column position, applicable to the S Series. | Glare. | Enable the glare suppression algorithm. |
| 18 | Black object imaging is incomplete, applicable to the S Series. | High-exposure mode needs to be enabled. | Enable high-exposure mode and set TOF register `100130B0` to `240`. |

## SDK Error Codes and Issue Handling

| No. | Error Code or Issue | Meaning/Cause | Recommended Action |
| --- | --- | --- | --- |
| 1 | `LX_ERROR=-1` | Unknown error. | The result is abnormal. Collect the device logs and send them to MRDVS support for analysis. |
| 2 | `LX_E_NOT_SUPPORT=-2` | Function not supported. | The current device does not support this function. Confirm the device model, firmware version, and interface capability. |
| 3 | `LX_E_NETWORK_ERROR=-3` | Network communication error. | Check the IP configuration, network cable, switch, and network packet loss. |
| 4 | `LX_E_INPUT_ILLEGAL=-4` | Invalid input parameter. | Check the parameters passed to the API. |
| 5 | `LX_E_RECONNECTING=-5` | Device reconnecting. | 1. If the device is reconnecting, wait for a short delay and retry until reconnection succeeds.<br>2. Check whether the camera is powered on normally and whether the network connection is normal. |
| 6 | `LX_E_DEVICE_ERROR=-6` | Device fault or device response failure. | Restart the application or camera, then retry. |
| 7 | `LX_E_DEVICE_NEED_UPDATE=-7` | The device version is too old and needs to be upgraded. | Upgrade the firmware version. |
| 8 | `LX_E_API_NEED_UPDATE=-8` | The API version is too old. | Update the SDK version. |
| 9 | `LX_E_CTRL_PERMISS_ERROR=-9` | Failed to obtain exclusive control permission. | The camera supports only one primary-permission client connection. If the camera is closed abnormally, wait `3-5s` for the permission to be released before retrying. |
| 10 | `LX_E_GET_DEVICEINFO_ERROR=-10` | Failed to obtain device information. | Confirm the device firmware version, or contact MRDVS support. |
| 11 | `LX_E_IMAGE_SIZE_ERROR=-11` | Image size mismatch. The device needs to be reopened. | The acquired image size does not match the size in the actual stream. Close and reopen the device. |
| 12 | `LX_E_IMAGE_PARTITION_ERROR=-12` | Image parsing failed because the pixformat is incorrect. Try reopening the device. | Reopen the device. |
| 13 | `LX_E_DEVICE_NOT_CONNECTED=-13` | Camera not connected. | This is a type of invalid input parameter error. If camera operations are performed before the camera is opened, this error is returned. Check the calling logic and make sure the camera has been opened successfully. |
| 14 | `LX_E_DEVICE_INIT_FAILED=-14` | Camera initialization failed. | Reopen the camera. |
| 15 | `LX_E_DEVICE_NOT_FOUND=-15` | Camera not found, or no matching camera found. | This error is usually returned when an invalid camera handle is passed to an API. Check the calling logic. |
| 16 | `LX_E_FILE_INVALID=-16` | File error. The file name, type, or format is incorrect, or the file failed to open. | Check whether the input file path and file type are correct, whether the file exists, and whether it can be opened manually. |
| 17 | `LX_E_CRC_CHECK_FAILED=-17` | File CRC or MD5 verification failed. | CRC or MD5 verification failed during file upload or download. Retry the operation. |
| 18 | `LX_E_TIME_OUT=-18` | Timeout. | Retry the operation. This issue may be caused by excessive network packet loss or a low camera frame rate. |
| 19 | `LX_E_FRAME_LOSS=-19` | Frame loss. | This often occurs in callback mode due to packet loss or frame loss. A small number of occurrences usually requires no special handling. If it occurs frequently, check the network bandwidth, buffer, and processing time. |
| 20 | `LX_E_ENABLE_ANYSTREAM_FAILED=-20` | Failed to enable any stream. | Before version `2.4.9`, disabling all streams was not allowed. This error code has been deprecated in later versions. |
| 21 | `LX_E_NOT_RECEIVE_STREAM=-21` | No stream data received. | No stream data is received. Refer to the troubleshooting steps for "No data is received after the camera is opened in the host software". |
| 22 | `LX_E_PARSE_STREAM_FAILED=-22` | Streaming started successfully, but stream parsing failed. | Confirm the firmware version, or contact MRDVS support. |
| 23 | `LX_E_PROCESS_IMAGE_FAILED=-23` | Image processing failed. | Confirm whether the Dataprocess image algorithm library version is compatible. |
| 24 | `LX_E_SETTING_NOT_ALLOWED=-24` | Setting is not allowed in always-on mode. | In always-on mode, ROI, multi-camera, alignment, and similar settings are not allowed. Adjust the calling logic by setting the camera working mode first, and then applying the related settings. |
| 25 | `LX_E_LOAD_DATAPROCESSLIB_ERROR=-25` | Failed to load the image processing algorithm library. | Ensure that the Dataprocess dynamic library or shared object file exists and is included in the library loading environment variables. In Linux, run `source install.sh`. |
| 26 | `LX_E_FUNCTION_CALL_LOGIC_ERROR=-26` | Function call logic error. | Adjust the calling logic based on the SDK logs. Common cases include:<br>1. Parameters that change the image size must be set after streaming is stopped.<br>2. Image size cannot be changed in always-on mode.<br>3. Host-side algorithm processing is mutually exclusive with enabling the built-in application algorithm.<br>4. Auto exposure cannot be enabled when HDR is enabled.<br>5. Auto-exposure parameters depend on the auto-exposure state.<br>6. Trigger-related parameters depend on the trigger mode. |
| 27 | `LX_E_IPAPPDR_UNREACHABLE_ERROR=-27` | IP unreachable or network configuration error. | The network is unreachable. Ensure that the SDK-side host can ping the visual device. |
| 28 | `LX_E_FRAME_ID_NOT_MATCH=-28` | Frame synchronization error within the timeout range. | After RGBD frame synchronization is enabled, this error is returned if TOF and RGB frames in the actual stream are not synchronized. Depending on the application requirement, either discard the frame or ignore the error. |
| 29 | `LX_E_FRAME_MULTI_MACHINE=-29` | Multi-camera interference signal detected in the frame. | Depending on the application requirement, either discard the frame or ignore the error. |
| 30 | `error code:-25` or `LxDataProcess` load failure | Applies only to Linux. The algorithm library is not configured in the environment variables. | Run `source install.sh`. |
| 31 | The image is split into three parts after stream acquisition. | The displayed width-height ratio is abnormal after the image is acquired. | To be supplemented. |
| 32 | ROS2 conflicts with the SDK library. | Not specified in the original record. | Encapsulate the Camera SDK in the `util` folder and link to the Camera SDK library indirectly. The specific project structure needs to be confirmed based on the actual project. |
| 33 | How to determine in software whether the camera is a Lite version or a standard version with RGB. | Not specified in the original record. | Call `DcGetBoolValue(handle,LX_BOOL_ENABLE_2D_STREAM,&test_rgb)`. If the camera is a Lite version, the API returns not supported. |
| 34 | Time synchronization method. | SDK and PTP synchronization. | Synchronization is performed automatically on each API call. No additional trigger is required. |

## Items to Be Supplemented

The following items are not sufficiently described in the original records and should be supplemented later based on field cases:

1. "No object is present in the front space, but point cloud data exists" lacks a clear cause and solution.
2. "The image is split into three parts after stream acquisition" lacks a clear solution.
3. For RGB image abnormality, skewed depth image, and obvious point cloud layering, the current recommendation is hardware replacement. It is recommended to supplement the judgment criteria, log requirements, and repair conditions.
4. For "ROS2 conflicts with the SDK library", it is recommended to supplement the specific project directory, CMake linking method, and example description.
5. For items involving version upgrades, confirm the minimum available version and recommended version before release.

---

<p align="center">
  <sub><em>Last updated: June 2026</em></sub><br>
  <sub><em>Hangzhou Lanxin Technology Co., Ltd. & MRDVS Co., Ltd.</em></sub><br>
  <sub><em>All Rights Reserved.</em></sub>
</p>
