cmake_minimum_required(VERSION ${CMAKE_VERSION})
message("***************build ros ")
set(CMAKE_CXX_STANDARD 14)
project(lx_camera_ros)

#catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        cv_bridge
        tf
	pcl_ros
        image_transport
	geometry_msgs
        sensor_msgs
        )

add_message_files(
DIRECTORY msg
FILES

ObstacleBox.msg
FrameRate.msg
Obstacle.msg
Pallet.msg
Result.msg
)
add_service_files(
DIRECTORY srv
FILES

LxString.srv
LxFloat.srv
LxBool.srv
LxCmd.srv
LxInt.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp message_runtime)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src)

if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

#SDK
include_directories(/opt/Lanxin-MRDVS/include/)
link_directories(/opt/Lanxin-MRDVS/lib/)
#pcl
find_package(PCL QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_VERSION: " ${PCL_VERSION})
#opencv
find_package(OpenCV QUIET REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIR})
message(STATUS "OPENCV_VERSION " ${OpenCV_VERSION})

set(TARGET_NAME lx_camera_node)
add_executable(${TARGET_NAME}
                ${PROJECT_SOURCE_DIR}/src/lx_camera/lx_camera.cpp
                ${PROJECT_SOURCE_DIR}/src/lx_camera/lx_camera_node.cpp)
add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS} lx_camera_ros_gencpp)
target_link_libraries(${TARGET_NAME} LxCameraApi ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS} ${catkin_LIBRARIES})

set(TARGET_NAME lx_localization_node)
add_executable(${TARGET_NAME}
                ${PROJECT_SOURCE_DIR}/src/lx_localization/lx_localization.cpp
                ${PROJECT_SOURCE_DIR}/src/lx_localization/lx_localization_node.cpp)
target_link_libraries(${TARGET_NAME} LxCameraApi ${OpenCV_LIBS} ${catkin_LIBRARIES})

set(TARGET_NAME sensor_sim_node)
add_executable(${TARGET_NAME}
                ${PROJECT_SOURCE_DIR}/src/lx_localization/sensor_sim.cpp
                ${PROJECT_SOURCE_DIR}/src/lx_localization/sensor_sim_node.cpp)
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS})
endif()
