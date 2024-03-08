cmake_minimum_required(VERSION 3.5)

message("***************build ros ")
project(lx_camera_ros)

set(TARGET_NAME lx_camera_node)
set(CMAKE_CXX_STANDARD 14)

#catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
		pcl_ros
        image_transport
        sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#SDK
include_directories(/opt/LX_MRDVS/include/)
link_directories(/opt/LX_MRDVS/lib/)

add_executable(${TARGET_NAME} src/main.cpp src/LxCameraRos.cpp)
target_link_libraries(${TARGET_NAME}
        LxCameraApi
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        )
