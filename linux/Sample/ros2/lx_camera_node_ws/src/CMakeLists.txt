cmake_minimum_required(VERSION ${CMAKE_VERSION})
message("***************build ros")
set(CMAKE_CXX_STANDARD 14)
project(lx_camera_ros)

execute_process(COMMAND lsb_release -r COMMAND cut -f 2 OUTPUT_VARIABLE LxDynamic)
string(STRIP ${LxDynamic} ${LxDynamic})
if (${LxDynamic} VERSION_LESS_EQUAL "18.04")
	message("Use Normal Link:"${LxDynamic})
	SET(LAUNCH_DIR "${CMAKE_SOURCE_DIR}/src/launch/ubuntu18/")
else()
	add_definitions(-DLxDynamic)
	message("Use Dnamic Link:"${LxDynamic})
	SET(LAUNCH_DIR "${CMAKE_SOURCE_DIR}/src/launch/ubuntu22/")
endif ()
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/pallet.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/mapping.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/obstacle.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/obstacleV2.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/sensor_sim.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/localization.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/lx_camera_ros_launch.py ${CMAKE_SOURCE_DIR}/launch/)

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
if (${LxDynamic} VERSION_GREATER "18.04")
	find_package(PCL REQUIRED COMPONENTS common io)
	message("Ubuntu 20.04 pcl: " ${PCL_VERSION})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
endif()

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

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

set(dependencies rclcpp image_transport sensor_msgs
				cv_bridge nav_msgs pcl_conversions)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
ament_export_dependencies(rosidl_default_runtime)

file(GLOB  HPP src/*.hpp )
file(GLOB  ALL src/*.cpp src/*.h )
file(GLOB  LCCPP src/lx_camera/*.cpp)
file(GLOB  LCH src/lx_camera/*.h)
file(GLOB  SSCPP src/sensor_sim/*.cpp)
file(GLOB  SSH src/sensor_sim/*.h)
file(GLOB  LCPP src/location/*.cpp)
file(GLOB  LH src/location/*.h)

#SDK
include_directories(/opt/Lanxin-MRDVS/include/)
set(LX_LIB_PATH /opt/Lanxin-MRDVS/lib/)
link_directories(${LX_LIB_PATH})
set(LIBS ${LIBS} LxCameraApi)

if (${LxDynamic} VERSION_EQUAL "18.04")
	find_package(PCL REQUIRED)
	message("Ubuntu 18.04 pcl: " ${PCL_VERSION})
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	set(LIBS ${LIBS} ${PCL_LIBRARIES})
endif()

set(LIBS ${LIBS} ${catkin_LIBRARIES})

set(TARGET_NAME lx_camera_node)
add_executable(${TARGET_NAME} ${LCCPP} ${LCH} ${HPP} ${ALL})
rosidl_target_interfaces(lx_camera_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(${TARGET_NAME} ${dependencies} rosidl_default_runtime )
target_link_libraries(${TARGET_NAME} ${LIBS} ${PCL_LIBRARIES})
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(FILES ${LX_LIB_PATH}libLxDataProcess.so ${LX_LIB_PATH}libLxCameraApi.so DESTINATION lib/)

set(TARGET_NAME location_node)
add_executable(${TARGET_NAME}  ${LCPP} ${LH} ${HPP} ${ALL})
ament_target_dependencies(${TARGET_NAME} ${dependencies})
target_link_libraries(${TARGET_NAME} ${LIBS} )
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

set(TARGET_NAME sensor_sim_node)
add_executable(${TARGET_NAME}  ${SSCPP} ${SSH} ${HPP} ${ALL})
ament_target_dependencies(${TARGET_NAME} ${dependencies})
target_link_libraries(${TARGET_NAME} ${LIBS} )
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()

