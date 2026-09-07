/*************************************************************************************
 * @Description:
 * @Version: 1.0
 * @Autor: Do not edit
 * @Date: 2025-08-06 16:28:45
 * @LastEditors: Do not edit
 * @LastEditTime: 2025-08-06 16:28:46
 *************************************************************************************/
#ifndef DL_HPP
#define DL_HPP

#include "lx_camera_application.h"
#include "lx_camera_define.h"
#include <sys/time.h>

struct DcLib {
  void *handle = nullptr;
  const char *(*DcGetApiVersion)();
  LX_STATE (*DcSetInfoOutput)(int, bool, const char *);
  LX_STATE (*DcGetDeviceList)(LxDeviceInfo **, int *);
  LX_STATE(*DcOpenDevice)
  (LX_OPEN_MODE, const char *, DcHandle *, LxDeviceInfo *);
  LX_STATE (*DcCloseDevice)(DcHandle);
  LX_STATE (*DcStartStream)(DcHandle);
  LX_STATE (*DcStopStream)(DcHandle);
  LX_STATE (*DcSetIntValue)(DcHandle, int, int);
  LX_STATE (*DcGetIntValue)(DcHandle, int, LxIntValueInfo *);
  LX_STATE (*DcSetFloatValue)(DcHandle, int, float);
  LX_STATE (*DcGetFloatValue)(DcHandle, int, LxFloatValueInfo *);
  LX_STATE (*DcSetBoolValue)(DcHandle, int, bool);
  LX_STATE (*DcGetBoolValue)(DcHandle, int, bool *);
  LX_STATE (*DcSetStringValue)(DcHandle, int, const char *);
  LX_STATE (*DcGetStringValue)(DcHandle, int, char **);
  LX_STATE (*DcGetPtrValue)(DcHandle, int, void **);
  LX_STATE (*DcSetGpuEnable)(bool);
  LX_STATE (*DcSetJpegDecodeMethod)(int);
  LX_STATE (*DcSetCmd)(DcHandle, int);
  const char *(*DcGetErrorString)(LX_STATE);
  LX_STATE (*DcSpecialControl)(DcHandle, const char *, void *);
  LX_STATE(*DcRegisterCameraStatusCallback)
  (DcHandle, LX_CAMERA_STATUS_CALLBACK, void *);

  // IMU
  LX_STATE (*DcRegisterImuDataCallback)(DcHandle, LX_IMUDATA_CALLBACK, void *);
  LX_STATE (*DcUnregisterImuDataCallback)(DcHandle);

};

#ifdef LX_DYNAMIC
static bool is_dynamic = true;
#define DcGetApiVersion() (*LX_DYNAMIC_LIB->DcGetApiVersion)()
#define DcSetInfoOutput(A, B, C) (*LX_DYNAMIC_LIB->DcSetInfoOutput)(A, B, C)
#define DcGetDeviceList(A, B) (*LX_DYNAMIC_LIB->DcGetDeviceList)(A, B)
#define DcOpenDevice(A, B, C, D) (*LX_DYNAMIC_LIB->DcOpenDevice)(A, B, C, D)
#define DcCloseDevice(A) (*LX_DYNAMIC_LIB->DcCloseDevice)(A)
#define DcStartStream(A) (*LX_DYNAMIC_LIB->DcStartStream)(A)
#define DcStopStream(A) (*LX_DYNAMIC_LIB->DcStopStream)(A)
#define DcSetIntValue(A, B, C) (*LX_DYNAMIC_LIB->DcSetIntValue)(A, B, C)
#define DcGetIntValue(A, B, C) (*LX_DYNAMIC_LIB->DcGetIntValue)(A, B, C)
#define DcSetFloatValue(A, B, C) (*LX_DYNAMIC_LIB->DcSetFloatValue)(A, B, C)
#define DcGetFloatValue(A, B, C) (*LX_DYNAMIC_LIB->DcGetFloatValue)(A, B, C)
#define DcSetBoolValue(A, B, C) (*LX_DYNAMIC_LIB->DcSetBoolValue)(A, B, C)
#define DcGetBoolValue(A, B, C) (*LX_DYNAMIC_LIB->DcGetBoolValue)(A, B, C)
#define DcSetStringValue(A, B, C) (*LX_DYNAMIC_LIB->DcSetStringValue)(A, B, C)
#define DcGetStringValue(A, B, C) (*LX_DYNAMIC_LIB->DcGetStringValue)(A, B, C)
#define DcGetPtrValue(A, B, C) (*LX_DYNAMIC_LIB->DcGetPtrValue)(A, B, C)
#define DcSetGpuEnable(A) (*LX_DYNAMIC_LIB->DcSetGpuEnable)(A)
#define DcSetJpegDecodeMethod(A) (*LX_DYNAMIC_LIB->DcSetJpegDecodeMethod)(A)
#define DcSetCmd(A, B) (*LX_DYNAMIC_LIB->DcSetCmd)(A, B)
#define DcGetErrorString(A) (*LX_DYNAMIC_LIB->DcGetErrorString)(A)
#define DcSpecialControl(A, B, C) (*LX_DYNAMIC_LIB->DcSpecialControl)(A, B, C)
#define DcRegisterCameraStatusCallback(A, B, C)                                \
  (*LX_DYNAMIC_LIB->DcRegisterCameraStatusCallback)(A, B, C)

#define DcRegisterImuDataCallback(A, B, C)                                     \
  (*LX_DYNAMIC_LIB->DcRegisterImuDataCallback)(A, B, C)
#define DcUnregisterImuDataCallback(A)                                         \
  (*LX_DYNAMIC_LIB->DcUnregisterImuDataCallback)(A)
#define DcStartIMU(A, B, C) (*LX_DYNAMIC_LIB->DcStartIMU)(A, B, C)
#define DcStopImu(A) (*LX_DYNAMIC_LIB->DcStopImu)(A)
#else
#include "lx_camera_api.h"
static bool is_dynamic = false;
#endif

extern bool DynamicLink(DcLib *lib);
extern void DisDynamicLink(DcLib *lib);

struct Quaternion {
  double w, x, y, z;
};
struct EulerAngles {
  double roll, pitch, yaw;
};
extern EulerAngles ToEulerAngles(Quaternion q);
extern Quaternion ToQuaternion(double yaw, double pitch, double roll);
#endif // DL_HPP
