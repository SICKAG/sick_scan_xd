#if defined _MSC_VER && _MSC_VER >= 1300
#   pragma warning( disable : 4996 ) // suppress warning 4996 about unsafe string functions like strcpy, sprintf, etc.
#   ifndef _CRT_SECURE_NO_DEPRECATE
#   define  _CRT_SECURE_NO_DEPRECATE // suppress warning 4996 about unsafe string functions like strcpy, sprintf, etc.
#   endif
#endif
#ifdef WIN32
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

#include  <stdio.h>
#include <stdlib.h>
#include <string.h>
#include  <ctype.h>

#include "sick_scan_api.h"

#ifdef WIN32
#else
typedef void* HINSTANCE;
static HINSTANCE LoadLibrary(const char* szLibFilename)
{
  return dlopen(szLibFilename,RTLD_GLOBAL|RTLD_LAZY);
}
static int FreeLibrary ( HINSTANCE hLib )
{
  return !dlclose(hLib);
}
static void* GetProcAddress(HINSTANCE hLib, const char* szFunctionName)
{
  return dlsym(hLib,szFunctionName);
}
#endif

static HINSTANCE hinstLib = NULL;

typedef SickScanApiHandle(*SickScanApiCreate_PROCTYPE)(int argc, char** argv);
static SickScanApiCreate_PROCTYPE ptSickScanApiCreate = 0;

typedef int32_t(*SickScanApiInitByLaunchfile_PROCTYPE)(SickScanApiHandle apiHandle, const char* launchfile);
static SickScanApiInitByLaunchfile_PROCTYPE ptSickScanApiInitByLaunchfile = 0;

typedef int32_t(*SickScanApiInitByCli_PROCTYPE)(SickScanApiHandle apiHandle, int argc, char** argv);
static SickScanApiInitByCli_PROCTYPE ptSickScanApiInitByCli = 0;

typedef int32_t(*SickScanApiRegisterCartesianPointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
static SickScanApiRegisterCartesianPointCloudMsg_PROCTYPE ptSickScanApiRegisterCartesianPointCloudMsg = 0;

/*
*  Functions to initialize and close the API and a lidar
*/

// Load sick_scan_xd api library (dll or so file)
int32_t SickScanApiLoadLibrary(const char* library_filepath)
{
    int32_t ret = SICK_SCAN_API_SUCCESS;
    if (hinstLib == NULL)
    {
        hinstLib = LoadLibrary(library_filepath);
    }
    if (hinstLib == NULL)
    {
        printf("## ERROR SickScanApiLoadLibrary: LoadLibrary(%s) failed\n", library_filepath);
        ret = SICK_SCAN_API_NOT_LOADED;
    }
    return ret;
}

// Unload sick_scan_xd api library
int32_t SickScanApiUnloadLibrary()
{
    int32_t ret = SICK_SCAN_API_SUCCESS;
    if (hinstLib != 0)
    {
        if (!FreeLibrary(hinstLib))
        {
            printf("## ERROR SickScanApiUnloadLibrary: FreeLibrary() failed\n");
            ret = SICK_SCAN_API_ERROR;
        }
    }
    hinstLib = 0;
    ptSickScanApiCreate = 0;
    return ret;
}

/*
*  Create an instance of sick_scan_xd api.
*  Optional commandline arguments argc, argv identical to sick_generic_caller.
*  Call SickScanApiInitByLaunchfile or SickScanApiInitByCli to process a lidar.
*/
SickScanApiHandle SickScanApiCreate(int argc, char** argv)
{
    if (hinstLib == 0)
    {
        printf("## ERROR SickScanApiCreate: library not loaded\n");
        return 0;
    }
    if (ptSickScanApiCreate == 0)
    {
        ptSickScanApiCreate = GetProcAddress(hinstLib, "SickScanApiCreate");
    }
    if (ptSickScanApiCreate == 0)
    {
        printf("## ERROR SickScanApiCreate: GetProcAddress failed\n");
        return 0;
    }
    SickScanApiHandle apiHandle = ptSickScanApiCreate(argc, argv);
    if (apiHandle == 0)
    {
        printf("## ERROR SickScanApiCreate: library call SickScanApiCreate() returned 0\n");
    }
    return apiHandle;
}

// Release and free all resources of a handle; the handle is invalid after SickScanApiRelease
int32_t SickScanApiRelease(SickScanApiHandle apiHandle)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Initializes a lidar by launchfile and starts message receiving and processing
int32_t SickScanApiInitByLaunchfile(SickScanApiHandle apiHandle, const char* launchfile_args)
{
    if (hinstLib == 0 || apiHandle == 0)
    {
        printf("## ERROR SickScanApiInitByLaunchfile: library not initialized\n");
        return SICK_SCAN_API_NOT_INITIALIZED;
    }
    if (ptSickScanApiInitByLaunchfile == 0)
    {
        ptSickScanApiInitByLaunchfile = GetProcAddress(hinstLib, "SickScanApiInitByLaunchfile");
    }
    if (ptSickScanApiInitByLaunchfile == 0)
    {
        printf("## ERROR SickScanApiInitByLaunchfile: GetProcAddress failed\n");
        return 0;
    }
    int32_t ret = ptSickScanApiInitByLaunchfile(apiHandle, launchfile_args);
    if (ret != SICK_SCAN_API_SUCCESS)
    {
        printf("## ERROR SickScanApiInitByLaunchfile: library call SickScanApiInitByLaunchfile() returned error code %d\n", ret);
    }
    return ret;
}

// Initializes a lidar by commandline arguments and starts message receiving and processing
int32_t SickScanApiInitByCli(SickScanApiHandle apiHandle, int argc, char** argv)
{
    if (hinstLib == 0 || apiHandle == 0)
    {
        printf("## ERROR SickScanApiInitByCli: library not initialized\n");
        return SICK_SCAN_API_NOT_INITIALIZED;
    }
    if (ptSickScanApiInitByCli == 0)
    {
        ptSickScanApiInitByCli = GetProcAddress(hinstLib, "SickScanApiInitByCli");
    }
    if (ptSickScanApiInitByCli == 0)
    {
        printf("## ERROR SickScanApiInitByCli: GetProcAddress failed\n");
        return 0;
    }
    int32_t ret = ptSickScanApiInitByCli(apiHandle, argc, argv);
    if (ret != SICK_SCAN_API_SUCCESS)
    {
        printf("## ERROR SickScanApiInitByCli: library call SickScanApiInitByCli() returned error code %d\n", ret);
    }
    return ret;
}

// Stops message receiving and processing and closes a lidar
int32_t SickScanApiClose(SickScanApiHandle apiHandle)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

/*
*  Registration / deregistration of message callbacks
*/

// Register / deregister a callback for cartesian PointCloud messages, pointcloud in cartesian coordinates with fields x, y, z, intensity
int32_t SickScanApiRegisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    if (hinstLib == 0 || apiHandle == 0)
    {
        printf("## ERROR SickScanApiRegisterCartesianPointCloudMsg: library not initialized\n");
        return SICK_SCAN_API_NOT_INITIALIZED;
    }
    if (ptSickScanApiRegisterCartesianPointCloudMsg == 0)
    {
        ptSickScanApiRegisterCartesianPointCloudMsg= GetProcAddress(hinstLib, "SickScanApiRegisterCartesianPointCloudMsg");
    }
    if (ptSickScanApiRegisterCartesianPointCloudMsg == 0)
    {
        printf("## ERROR SickScanApiRegisterCartesianPointCloudMsg: GetProcAddress failed\n");
        return 0;
    }
    int32_t ret = ptSickScanApiRegisterCartesianPointCloudMsg(apiHandle, callback);
    if (ret != SICK_SCAN_API_SUCCESS)
    {
        printf("## ERROR SickScanApiRegisterCartesianPointCloudMsg: library call SickScanApiRegisterCartesianPointCloudMsg() returned error code %d\n", ret);
    }
    return ret;
}
int32_t SickScanApiDeregisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for polar PointCloud messages, pointcloud in polar coordinates with fields range, azimuth, elevation, intensity
int32_t SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for Imu messages
int32_t SickScanApiRegisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanLFErecMsg messages
int32_t SickScanApiRegisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanLIDoutputstateMsg messages
int32_t SickScanApiRegisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanRadarScan messages
int32_t SickScanApiRegisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Register / deregister a callback for SickScanLdmrsObjectArray messages
int32_t SickScanApiRegisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiDeregisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

/*
*  Polling functions
*/

// Wait for and return the next cartesian resp. polar PointCloud messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiWaitNextPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreePolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next Imu messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next LFErec messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next LIDoutputstate messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next RadarScan messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}

// Wait for and return the next LdmrsObjectArray messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SickScanApiWaitNextLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
int32_t SickScanApiFreeLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg)
{
    return SICK_SCAN_API_NOT_IMPLEMENTED;
}
