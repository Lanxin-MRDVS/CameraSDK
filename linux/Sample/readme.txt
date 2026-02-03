The /Sample directory contains C/C++ and ROS2 folders. Here are the descriptions of each folder:

    -/Sample/C: Contains camera SDK calling examples related to C and C++.
    -/Sample/ros2: Contains camera SDK calling examples related to ROS2 drivers.

1. C Language Examples (/Sample/C)

      ***application_location***: Obtains camera stream data, enables the visual positioning algorithm, and outputs the algorithm results.
      ***application_obstacle_v1***: Obtains camera stream data, enables the obstacle avoidance algorithm 1.0, and outputs the algorithm results.
      ***application_obstacle_v2***: Obtains camera stream data, enables the obstacle avoidance algorithm 2.0, and outputs the algorithm results.
      ***application_pallet***: Obtains camera stream data, enables the pallet algorithm, and outputs the algorithm results.
      ***arm_local_camera***: Call the SDK in the camera [ARM] and obtain stream data.
      ***frame_callback***: Obtains camera stream data using the callback method.
      ***multi_cameras***: Turns on multiple cameras and obtains stream data.
      ***single_camera2***: Obtains frame data through the structure method.
      ***CMakeLists.txt***: A project configuration file for building C projects using CMake.

2. ROS2 Examples (/Sample/ROS2)

    ***lx_camera_node_ws***: Contains a standard ROS2 driver.
    You can refer to other detailed documentation on ROS2 examples for reference.
