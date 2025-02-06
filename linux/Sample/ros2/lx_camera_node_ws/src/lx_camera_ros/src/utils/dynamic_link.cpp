#include <math.h>
#include <thread>
#include <dlfcn.h>
#include <iostream>
#include "dynamic_link.h"
#define RETURN_FAILED(A) {std::cout<<"Load "<<A<<" failed!"<<std::endl;return false;}  

bool DynamicLink(DcLib* lib){
	if(!is_dynamic){
		std::cout<<"Is not Dynamic link mode!"<<std::endl;
		return true;
	}

	void* handle = lib->handle;
	const char* lib_path = LX_LIB;
	std::cout<<"Begin to dynamic link libLxCameraApi.so! path:" << lib_path <<std::endl;
	// handle = dlopen("/opt/Lanxin-MRDVS/lib/libLxCameraApi.so", RTLD_LAZY);
	handle = dlopen(lib_path, RTLD_LAZY);
	while(!handle){
		std::cout<<"Load lib failed: "<<dlerror()<<std::endl;
	}
	std::cout<<"Dynamic link libLxCameraApi.so success!"<<std::endl;

	lib->DcGetApiVersion = (const char*(*)())dlsym(handle, "DcGetApiVersion"); 
	if (dlerror())RETURN_FAILED("DcGetApiVersion");

	lib->DcSetInfoOutput = (LX_STATE(*)(int, bool, const char*))dlsym(handle, "DcSetInfoOutput"); 
	if (dlerror())RETURN_FAILED("DcSetInfoOutput");

	lib->DcGetDeviceList = (LX_STATE(*)(LxDeviceInfo**, int*))dlsym(handle, "DcGetDeviceList"); 
	if (dlerror())RETURN_FAILED("DcGetDeviceList");

	lib->DcOpenDevice = (LX_STATE(*)(LX_OPEN_MODE, const char*, DcHandle*, LxDeviceInfo*))dlsym(handle, "DcOpenDevice"); 
	if (dlerror())RETURN_FAILED("DcOpenDevice");

	lib->DcCloseDevice = (LX_STATE(*)(DcHandle))dlsym(handle, "DcCloseDevice"); 
	if (dlerror())RETURN_FAILED("DcCloseDevice");

	lib->DcStartStream = (LX_STATE(*)(DcHandle))dlsym(handle, "DcStartStream"); 
	if (dlerror())RETURN_FAILED("DcStartStream");

	lib->DcStopStream = (LX_STATE(*)(DcHandle))dlsym(handle, "DcStopStream"); 
	if (dlerror())RETURN_FAILED("DcStopStream");

	lib->DcSetIntValue = (LX_STATE(*)(DcHandle, int, int))dlsym(handle, "DcSetIntValue"); 
	if (dlerror())RETURN_FAILED("DcSetIntValue");

	lib->DcGetIntValue = (LX_STATE(*)(DcHandle, int, LxIntValueInfo*))dlsym(handle, "DcGetIntValue"); 
	if (dlerror())RETURN_FAILED("DcGetIntValue");

	lib->DcSetFloatValue = (LX_STATE(*)(DcHandle, int, float))dlsym(handle, "DcSetFloatValue"); 
	if (dlerror())RETURN_FAILED("DcSetFloatValue");

	lib->DcGetFloatValue = (LX_STATE(*)(DcHandle, int, LxFloatValueInfo*))dlsym(handle, "DcGetFloatValue"); 
	if (dlerror())RETURN_FAILED("DcGetFloatValue");

	lib->DcSetBoolValue = (LX_STATE(*)(DcHandle, int, bool))dlsym(handle, "DcSetBoolValue"); 
	if (dlerror())RETURN_FAILED("DcSetBoolValue");

	lib->DcGetBoolValue = (LX_STATE(*)(DcHandle, int, bool*))dlsym(handle, "DcGetBoolValue"); 
	if (dlerror())RETURN_FAILED("DcGetBoolValue");

	lib->DcSetStringValue = (LX_STATE(*)(DcHandle, int, const char*))dlsym(handle, "DcSetStringValue"); 
	if (dlerror())RETURN_FAILED("DcSetStringValue");

	lib->DcGetStringValue = (LX_STATE(*)(DcHandle, int, char**))dlsym(handle, "DcGetStringValue"); 
	if (dlerror())RETURN_FAILED("DcGetStringValue");

	lib->DcGetPtrValue = (LX_STATE(*)(DcHandle, int, void**))dlsym(handle, "DcGetPtrValue"); 
	if (dlerror())RETURN_FAILED("DcGetPtrValue");

	lib->DcSetCmd = (LX_STATE(*)(DcHandle, int))dlsym(handle, "DcSetCmd"); 
	if (dlerror())RETURN_FAILED("DcSetCmd");

	lib->DcGetErrorString = (const char*(*)(LX_STATE))dlsym(handle, "DcGetErrorString"); 
	if (dlerror())RETURN_FAILED("DcGetErrorString");

	lib->DcSpecialControl = (LX_STATE(*)(DcHandle, const char*, void*))dlsym(handle, "DcSpecialControl"); 
	if (dlerror())RETURN_FAILED("DcSpecialControl");

	std::cout<<"Dynamic link success!"<<std::endl;
	return true;
}

void DisDynamicLink(DcLib* lib){
	if(lib->handle) dlclose(lib->handle);
	std::cout<<"Disdynamic link libLxCameraApi.so success!"<<std::endl;
}

Quaternion ToQuaternion(double yaw, double pitch, double roll){
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1) angles.pitch = std::copysign(M_PI / 2, sinp);
    else angles.pitch = std::asin(sinp);

    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
