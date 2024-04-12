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

file(GLOB  HPP src/*.hpp )
file(GLOB  LCCPP src/lx_camera/*.cpp )
file(GLOB  LCH src/lx_camera/*.h )
file(GLOB  SSCPP src/sensor_sim/*.cpp )
file(GLOB  SSH src/sensor_sim/*.h )
file(GLOB  LCPP src/location/*.cpp )
file(GLOB  LH src/location/*.h )

#SDK
include_directories(/opt/Lanxin-MRDVS/include/)
link_directories(/opt/Lanxin-MRDVS/lib/)

set(TARGET_NAME lx_camera_node)
add_executable(${TARGET_NAME} ${LCCPP} ${LCH} ${HPP})
add_dependencies(lx_camera_node ${catkin_EXPORTED_TARGETS} lx_camera_ros_gencpp)
target_link_libraries(${TARGET_NAME} LxCameraApi ${OpenCV_LIBS} ${catkin_LIBRARIES})

set(TARGET_NAME sensor_sim_node)
add_executable(${TARGET_NAME}  ${SSCPP} ${SSH} ${HPP})
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})

set(TARGET_NAME location_node)
add_executable(${TARGET_NAME}  ${LCPP} ${LH} ${HPP})
target_link_libraries(${TARGET_NAME} LxCameraApi ${catkin_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS})
endif()
