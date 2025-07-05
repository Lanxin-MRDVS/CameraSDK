# LxCamera SDK

A comprehensive software development kit for LxCamera devices, providing APIs and tools for camera control, data processing, and application development across multiple platforms.

## Overview

LxCamera SDK is a complete development solution for integrating LxCamera devices into your applications. It supports various camera series including S10, T2, S3, M4, S2max, and M series cameras, providing both C/C++ and Python APIs for cross-platform development.

## Features

- **Multi-Platform Support**: Windows, Linux (x64, ARM32, ARM64), and OpenHarmony
- **Multiple Programming Languages**: C/C++, Python, C#
- **ROS Integration**: Full support for ROS1 and ROS2
- **Camera Series Support**: 
  - S10 series
  - T2 series  
  - S3 series
  - M4 series
  - S2max series
  - M series
- **Application Types**:
  - Single camera control
  - Multi-camera management
  - Frame callback processing
  - Obstacle detection
  - Pallet localization
  - Application location services
- **Development Tools**: 
  - LxCameraViewer for device configuration and visualization
  - LxApplicationViewer for application testing
  - Network configuration tools

## Supported Platforms

### Linux
- **Architectures**: x64, ARM32, ARM64
- **Distributions**: Ubuntu 18.04+, Ubuntu 22.04+
- **Libraries**: libLxCameraApi.so, libLxDataProcess.so

### Windows
- **Architectures**: x86, x64
- **Libraries**: LxCameraApi.dll, LxDataProcess.dll

### OpenHarmony
- **Architecture**: ARM64
- **Libraries**: libLxCameraApi.so, libLxDataProcess.so

## Quick Start

### Prerequisites

- **Linux**: GCC 7.0+ or Clang 6.0+
- **Windows**: Visual Studio 2017+ or MinGW
- **Python**: Python 3.6+ (for Python API)
- **ROS**: ROS1 Noetic or ROS2 Foxy+ (for ROS integration)

### Installation

#### Linux
```bash
# Navigate to the linux directory
cd linux

# Run the installation script
./install.sh

# Set firewall rules (if needed)
./set_firewall.sh

# Configure socket buffer size (if needed)
./set_socket_buffer_size.sh
```

#### Windows
1. Extract the SDK to your desired location
2. Add the SDK library path to your system PATH
3. Install the required Visual C++ Redistributables

#### Python API
```bash
# Install the Python wheel package
pip install lx_camera_py-1.3.2-py3-none-any.whl
```

### Basic Usage

#### C/C++ Example
```cpp
#include "lx_camera_api.h"

// Initialize camera
LxCameraHandle handle;
int result = LxCameraOpen(&handle, "192.168.1.100");

if (result == LX_SUCCESS) {
    // Start streaming
    LxCameraStartStream(handle);
    
    // Process frames...
    
    // Cleanup
    LxCameraStopStream(handle);
    LxCameraClose(handle);
}
```

#### Python Example
```python
import lx_camera

# Initialize camera
camera = lx_camera.LxCamera()
result = camera.open("192.168.1.100")

if result == lx_camera.LX_SUCCESS:
    # Start streaming
    camera.start_stream()
    
    # Process frames...
    
    # Cleanup
    camera.stop_stream()
    camera.close()
```

## Sample Applications

The SDK includes comprehensive sample applications demonstrating various use cases:

### C/C++ Samples
- **single_camera2**: Basic single camera control
- **multi_cameras**: Multi-camera management
- **frame_callback**: Frame processing with callbacks
- **application_obstacle**: Obstacle detection application
- **application_pallet**: Pallet localization
- **application_location**: Application location services
- **arm_local_camera**: ARM platform specific examples

### Python Samples
- Basic camera control examples
- Data processing demonstrations

### C# Samples
- .NET integration examples
- Windows Forms applications

### ROS Integration
- **ROS1**: Complete ROS1 node implementation
- **ROS2**: Full ROS2 node support with launch files
- **Localization**: SLAM and localization capabilities
- **Mapping**: Environment mapping features

## Building from Source

### Linux
```bash
cd linux/Sample/C
mkdir build && cd build
cmake ..
make
```

### Windows
```bash
cd windows/Sample/C
mkdir build && cd build
cmake ..
# Open generated solution in Visual Studio
```

## Documentation

Comprehensive documentation is available in the `Document/` directory:

- **Camera SDK Development Guide.pdf** - Complete C/C++ API reference
- **LxCameraSDK-Python User Manual_EN.PDF** - Python API documentation
- **Camera 设备二次开发常见问题说明.pdf** - Common development issues (Chinese)
- **Linux 示例程序使用说明.pdf** - Linux sample usage guide
- **LxCameraViewer使用说明书.pdf** - Viewer tool manual

## Tools

### LxCameraViewer
A comprehensive GUI tool for:
- Camera discovery and connection
- Real-time video streaming
- Parameter configuration
- Data visualization
- Device management

### LxApplicationViewer
Application testing and debugging tool for development workflows.

## Firmware

Latest firmware files are available in the `Firmware/` directory:
- H3 series: H3-update-V1.1.11_20250122.bin
- M series: M-kernel-update-V1.1.102_250417_MDS_SP01.bin
- M4Pro series: M4Pro-kernel-update-V1.1.102.250310_mds_sp01.bin
- S2 series: S2-update-V1.1.35_250121.bin

## Version History

### Current Version: 1.3.60.0114
- **SDK V2.4.38**: Support for S10 series, bug fixes and new features
- **LxDataProcess V1.2.27**: Bug fixes and improvements
- **Firmware**: Various bug fixes across all supported series

### Previous Versions
- **1.3.52.0826**: T2 and S3 series support
- **1.3.47.0611**: Enhanced configuration capabilities
- **1.3.45.0425**: M4 and S2max series support

## Support

For technical support and questions:
- Check the documentation in the `Document/` directory
- Review the sample applications for implementation examples
- Consult the common issues guide for troubleshooting

## License

This SDK is provided for development and integration purposes. Please refer to the license terms included with your LxCamera device purchase.

---

**Note**: This SDK is designed for use with LxCamera devices. Ensure your device firmware is up to date for optimal compatibility and performance.
