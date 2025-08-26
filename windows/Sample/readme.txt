      The ***/Sample*** directory contains camera SDK calling examples in three programming languages: C, C#, and Python. Below is an explanation of the files for each language:

1. C Language Examples (/Sample/C)

      ***application_location***: Obtains camera stream data, enables the visual positioning algorithm, and outputs the algorithm results.
      ***application_obstacle_v1***: Obtains camera stream data, enables the obstacle avoidance algorithm 1.0, and outputs the algorithm results.
      ***application_obstacle_v2***: Obtains camera stream data, enables the obstacle avoidance algorithm 2.0, and outputs the algorithm results.
      ***application_pallet***: Obtains camera stream data, enables the pallet algorithm, and outputs the algorithm results.
      ***frame_callback***: Obtains camera stream data using the callback method.
      ***multi_cameras***: Turns on multiple cameras and obtains stream data.
      ***single_camera2***: Obtains frame data through the structure method.
      ***CMakeLists.txt***: A project configuration file for building C projects using CMake.

2. C# Language Examples (/Sample/C#)

      ***demo_camera.cs***: Turns on the camera and obtains frame data via the structure method.
      ***demo_obstacle.cs***: Obtains camera stream data, enables the obstacle avoidance algorithm, and outputs the algorithm results.
      ***demo_pallet.cs***: Obtains camera stream data, enables the pallet positioning algorithm, and outputs the algorithm results.
      ***LxCameraApi.cs***: Calls functions in LxCameraApi.dll through DllImport and encapsulates various operations on relevant camera devices.
      ***LxCameraApplication.cs***: Used to describe various data structures related to camera application algorithms.

3. Python Language Examples (/Sample/python)

      ***lx_camera_py-1.3.3-py3-none-any.whl***: The 1.3.3 version of the lx camera interaction package for Python 3, available cross - platform.
