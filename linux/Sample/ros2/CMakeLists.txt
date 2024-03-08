cmake_minimum_required(VERSION 3.0)

message("***************build ros2 ")
project(lx_camera_ros2)

set(TARGET_NAME lx_camera_node)
set(CMAKE_CXX_STANDARD 14)

#ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
set(dependencies rclcpp image_transport  sensor_msgs cv_bridge)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
ament_package()

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#SDK
include_directories(/opt/LX_MRDVS/include/)
link_directories(/opt/LX_MRDVS/lib/)

add_executable(${TARGET_NAME} src/main.cpp src/LxCameraRos.cpp)
ament_target_dependencies(${TARGET_NAME} ${dependencies})
target_link_libraries(${TARGET_NAME} LxCameraApi )

