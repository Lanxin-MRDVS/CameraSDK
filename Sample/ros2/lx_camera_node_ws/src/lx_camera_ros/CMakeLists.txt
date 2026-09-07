cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
project(lx_camera_ros)

add_definitions(-DLX_DYNAMIC)

if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

# 启用性能优化和 CPU 特定指令集
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=native")

# 启用 OpenMP
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

#ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/ObstacleBox.msg"
"msg/FrameRate.msg"
"msg/Obstacle.msg"
"msg/Pallet.msg"
"msg/Result.msg"

"srv/LxString.srv"
"srv/LxFloat.srv"
"srv/LxBool.srv"
"srv/LxCmd.srv"
"srv/LxInt.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs 
  ADD_LINTER_TESTS
)

set(dependencies rclcpp image_transport sensor_msgs
				cv_bridge nav_msgs pcl_conversions tf2 tf2_ros)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
)
#SDK
include_directories(/opt/MRDVS/include)
set(LX_LIB_PATH /opt/MRDVS/lib)
link_directories(${LX_LIB_PATH})
set(LX_LIB ${LX_LIB_PATH}/libLxCameraApi.so)
add_definitions(-DLX_LIB="${LX_LIB}")
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
				${PROJECT_SOURCE_DIR}/src/lx_camera/lx_camera_node.cpp 
				${PROJECT_SOURCE_DIR}/src/utils/dynamic_link.cpp)
# rosidl_target_interfaces(${TARGET_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_target_dependencies(${TARGET_NAME} ${dependencies} rosidl_default_runtime )
target_link_libraries(${TARGET_NAME}
                      ${PCL_LIBRARIES} 
					  "${cpp_typesupport_target}")
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(FILES ${LX_LIB_PATH}/libLxDataProcess.so ${LX_LIB_PATH}/libLxCameraApi.so DESTINATION lib/)

set(TARGET_NAME lx_localization_node)
add_executable(${TARGET_NAME}
				${PROJECT_SOURCE_DIR}/src/lx_localization/lx_localization.cpp 
				${PROJECT_SOURCE_DIR}/src/lx_localization/lx_localization_node.cpp 
				${PROJECT_SOURCE_DIR}/src/utils/dynamic_link.cpp)
ament_target_dependencies(${TARGET_NAME} ${dependencies})
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS} )
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

set(TARGET_NAME sensor_sim_node)
add_executable(${TARGET_NAME}
				${PROJECT_SOURCE_DIR}/src/lx_localization/sensor_sim.cpp 
				${PROJECT_SOURCE_DIR}/src/lx_localization/sensor_sim_node.cpp)
ament_target_dependencies(${TARGET_NAME} ${dependencies})
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
launch rviz
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
