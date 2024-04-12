#ifndef DL_HPP
#define DL_HPP

#include <sys/time.h>
#include "lx_camera_define.h"
#include "lx_camera_application.h"

struct DcLib{
	void *handle = nullptr;
	const char*(*DcGetApiVersion)();
	LX_STATE (*DcSetInfoOutput)(int, bool, const char*);
	LX_STATE (*DcGetDeviceList)(LxDeviceInfo**, int*);
	LX_STATE (*DcOpenDevice)(LX_OPEN_MODE, const char*, DcHandle*, LxDeviceInfo*);
	LX_STATE (*DcCloseDevice)(DcHandle);
	LX_STATE (*DcStartStream)(DcHandle);
	LX_STATE (*DcStopStream)(DcHandle);
	LX_STATE (*DcSetIntValue)(DcHandle, int, int);
	LX_STATE (*DcGetIntValue)(DcHandle, int, LxIntValueInfo*);
	LX_STATE (*DcSetFloatValue)(DcHandle, int, float);
	LX_STATE (*DcGetFloatValue)(DcHandle, int, LxFloatValueInfo*);
	LX_STATE (*DcSetBoolValue)(DcHandle, int, bool);
	LX_STATE (*DcGetBoolValue)(DcHandle, int, bool*);
	LX_STATE (*DcSetStringValue)(DcHandle, int, const char*);
	LX_STATE (*DcGetStringValue)(DcHandle, int, char**);
	LX_STATE (*DcGetPtrValue)(DcHandle, int, void**);
	LX_STATE (*DcSetCmd)(DcHandle, int);
	const char*(*DcGetErrorString)(LX_STATE);
	LX_STATE (*DcSpecialControl)(DcHandle, const char*, void*);
};

#ifdef LxDynamic
static bool is_dynamic = true;
#define DcGetApiVersion() (*lib->DcGetApiVersion)()
#define DcSetInfoOutput(A,B,C) (*lib->DcSetInfoOutput)(A,B,C)
#define DcGetDeviceList(A,B) (*lib->DcGetDeviceList)(A,B)
#define DcOpenDevice(A,B,C,D) (*lib->DcOpenDevice)(A,B,C,D)
#define DcCloseDevice(A) (*lib->DcCloseDevice)(A)
#define DcStartStream(A) (*lib->DcStartStream)(A)
#define DcStopStream(A) (*lib->DcStopStream)(A)
#define DcSetIntValue(A,B,C) (*lib->DcSetIntValue)(A,B,C)
#define DcGetIntValue(A,B,C) (*lib->DcGetIntValue)(A,B,C)
#define DcSetFloatValue(A,B,C) (*lib->DcSetFloatValue)(A,B,C)
#define DcGetFloatValue(A,B,C) (*lib->DcGetFloatValue)(A,B,C)
#define DcSetBoolValue(A,B,C) (*lib->DcSetBoolValue)(A,B,C)
#define DcGetBoolValue(A,B,C) (*lib->DcGetBoolValue)(A,B,C)
#define DcSetStringValue(A,B,C) (*lib->DcSetStringValue)(A,B,C)
#define DcGetStringValue(A,B,C) (*lib->DcGetStringValue)(A,B,C)
#define DcGetPtrValue(A,B,C) (*lib->DcGetPtrValue)(A,B,C)
#define DcSetCmd(A,B) (*lib->DcSetCmd)(A,B)
#define DcGetErrorString(A) (*lib->DcGetErrorString)(A)
#define DcSpecialControl(A,B,C) (*lib->DcSpecialControl)(A,B,C)
#else
#include "lx_camera_api.h"
static bool is_dynamic = false;
#endif

extern bool DynamicLink(DcLib* lib);
extern void DisDynamicLink(DcLib* lib);

struct Quaternion{double w, x, y, z;};
struct EulerAngles {double roll, pitch, yaw;};
extern EulerAngles ToEulerAngles(Quaternion q);
extern Quaternion ToQuaternion(double yaw, double pitch, double roll);
#endif  //DL_HPP
