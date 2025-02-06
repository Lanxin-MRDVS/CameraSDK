message("***************build demo")

set(LIBS ${LX_LIBRARY_NAME})



if (NOT WIN32 AND NOT ANDROID_ABI)
    list(APPEND LIBS pthread)
endif ()

if(${OPENCV_FOUND})
    add_definitions(-DHAS_OPENCV)
    list(APPEND LIBS ${OpenCV_LIBS})
endif()

macro(add_exe NAME)
add_executable(demo_${NAME} ${NAME}/${NAME}.cpp)
target_link_libraries(demo_${NAME} ${LIBS})
install(FILES ${NAME}/${NAME}.cpp
    DESTINATION ${INSTALL_PLAT_PATH}/Sample/C/${NAME})
if(WIN32)
    install(TARGETS demo_${NAME} 
        DESTINATION ${INSTALL_PLAT_PATH}/Tools)
#else()
#    install(TARGETS demo_${NAME}
#        DESTINATION ${INSTALL_PLAT_PATH}/Tools/${PLATFORM_PREFEX})
endif(WIN32)
SET_PROPERTY(TARGET demo_${NAME} PROPERTY FOLDER "demo")
endmacro()

#add_exe(single_camera)
add_exe(single_camera2)
add_exe(multi_cameras)
add_exe(application_obstacle)
add_exe(application_pallet)
add_exe(frame_callback)
add_exe(application_location)
#add_exe(get_device_log)

if(${PLATFORM_PREFEX} MATCHES "linux_arm64")
add_exe(arm_local_camera)
endif()

add_executable(update_version update_version.cpp)
target_link_libraries(update_version ${LIBS})

if(WIN32)
    install(TARGETS update_version
        DESTINATION ${INSTALL_PLAT_PATH}/Tools)
#else()
#    install(TARGETS update_version
#        DESTINATION ${INSTALL_PLAT_PATH}/Tools/${PLATFORM_PREFEX})
endif(WIN32)

SET_PROPERTY(TARGET update_version PROPERTY FOLDER "tools")
