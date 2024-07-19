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

#include "sick_scan_xd_api/sick_scan_api.h"

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

#ifdef WIN32
typedef SickScanApiHandle(SICK_SCAN_XD_API_CALLING_CONVENTION * SickScanApiCreate_PROCTYPE)(int argc, char** argv);
static SickScanApiCreate_PROCTYPE ptSickScanApiCreate = 0;
#else
typedef SickScanApiHandle(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiCreate_PROCTYPE)(int argc, char** argv);
static SickScanApiCreate_PROCTYPE ptSickScanApiCreate = 0;
#endif

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRelease_PROCTYPE)(SickScanApiHandle apiHandle);
static SickScanApiRelease_PROCTYPE ptSickScanApiRelease = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiInitByLaunchfile_PROCTYPE)(SickScanApiHandle apiHandle, const char* launchfile);
static SickScanApiInitByLaunchfile_PROCTYPE ptSickScanApiInitByLaunchfile = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiInitByCli_PROCTYPE)(SickScanApiHandle apiHandle, int argc, char** argv);
static SickScanApiInitByCli_PROCTYPE ptSickScanApiInitByCli = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiClose_PROCTYPE)(SickScanApiHandle apiHandle);
static SickScanApiClose_PROCTYPE ptSickScanApiClose = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterCartesianPointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
static SickScanApiRegisterCartesianPointCloudMsg_PROCTYPE ptSickScanApiRegisterCartesianPointCloudMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterCartesianPointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
static SickScanApiDeregisterCartesianPointCloudMsg_PROCTYPE ptSickScanApiDeregisterCartesianPointCloudMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterPolarPointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
static SickScanApiRegisterPolarPointCloudMsg_PROCTYPE ptSickScanApiRegisterPolarPointCloudMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterPolarPointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback);
static SickScanApiDeregisterPolarPointCloudMsg_PROCTYPE ptSickScanApiDeregisterPolarPointCloudMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterImuMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);
static SickScanApiRegisterImuMsg_PROCTYPE ptSickScanApiRegisterImuMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterImuMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback);
static SickScanApiDeregisterImuMsg_PROCTYPE ptSickScanApiDeregisterImuMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterLFErecMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);
static SickScanApiRegisterLFErecMsg_PROCTYPE ptSickScanApiRegisterLFErecMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterLFErecMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback);
static SickScanApiDeregisterLFErecMsg_PROCTYPE ptSickScanApiDeregisterLFErecMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterLIDoutputstateMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);
static SickScanApiRegisterLIDoutputstateMsg_PROCTYPE ptSickScanApiRegisterLIDoutputstateMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterLIDoutputstateMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback);
static SickScanApiDeregisterLIDoutputstateMsg_PROCTYPE ptSickScanApiDeregisterLIDoutputstateMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterRadarScanMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);
static SickScanApiRegisterRadarScanMsg_PROCTYPE ptSickScanApiRegisterRadarScanMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterRadarScanMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback);
static SickScanApiDeregisterRadarScanMsg_PROCTYPE ptSickScanApiDeregisterRadarScanMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterLdmrsObjectArrayMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);
static SickScanApiRegisterLdmrsObjectArrayMsg_PROCTYPE ptSickScanApiRegisterLdmrsObjectArrayMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterLdmrsObjectArrayMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback);
static SickScanApiDeregisterLdmrsObjectArrayMsg_PROCTYPE ptSickScanApiDeregisterLdmrsObjectArrayMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterVisualizationMarkerMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback);
static SickScanApiRegisterVisualizationMarkerMsg_PROCTYPE ptSickScanApiRegisterVisualizationMarkerMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterVisualizationMarkerMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback);
static SickScanApiDeregisterVisualizationMarkerMsg_PROCTYPE ptSickScanApiDeregisterVisualizationMarkerMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterDiagnosticMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback);
static SickScanApiRegisterDiagnosticMsg_PROCTYPE ptSickScanApiRegisterDiagnosticMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterDiagnosticMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback);
static SickScanApiDeregisterDiagnosticMsg_PROCTYPE ptSickScanApiDeregisterDiagnosticMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterLogMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback);
static SickScanApiRegisterLogMsg_PROCTYPE ptSickScanApiRegisterLogMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterLogMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback);
static SickScanApiDeregisterLogMsg_PROCTYPE ptSickScanApiDeregisterLogMsg = 0;

typedef int32_t (SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiGetStatus_PROCTYPE)(SickScanApiHandle apiHandle, int32_t* status_code, char* message_buffer, int32_t message_buffer_size);
static SickScanApiGetStatus_PROCTYPE ptSickScanApiGetStatus = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiSendSOPAS_PROCTYPE)(SickScanApiHandle apiHandle, const char* sopas_command, char* sopas_response_buffer, int32_t response_buffer_size);
static SickScanApiSendSOPAS_PROCTYPE ptSickScanApiSendSOPAS = 0;

typedef int32_t (SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiSetVerboseLevel_PROCTYPE)(SickScanApiHandle apiHandle, int32_t verbose_level);
static SickScanApiSetVerboseLevel_PROCTYPE ptSickScanApiSetVerboseLevel = 0;

typedef int32_t (SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiGetVerboseLevel_PROCTYPE)(SickScanApiHandle apiHandle);
static SickScanApiGetVerboseLevel_PROCTYPE ptSickScanApiGetVerboseLevel = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextCartesianPointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
static SickScanApiWaitNextCartesianPointCloudMsg_PROCTYPE ptSickScanApiWaitNextCartesianPointCloudMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextPolarPointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec);
static SickScanApiWaitNextPolarPointCloudMsg_PROCTYPE ptSickScanApiWaitNextPolarPointCloudMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreePointCloudMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg);
static SickScanApiFreePointCloudMsg_PROCTYPE ptSickScanApiFreePointCloudMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextImuMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec);
static SickScanApiWaitNextImuMsg_PROCTYPE ptSickScanApiWaitNextImuMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreeImuMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanImuMsg* msg);
static SickScanApiFreeImuMsg_PROCTYPE ptSickScanApiFreeImuMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextLFErecMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec);
static SickScanApiWaitNextLFErecMsg_PROCTYPE ptSickScanApiWaitNextLFErecMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreeLFErecMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg);
static SickScanApiFreeLFErecMsg_PROCTYPE ptSickScanApiFreeLFErecMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextLIDoutputstateMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec);
static SickScanApiWaitNextLIDoutputstateMsg_PROCTYPE ptSickScanApiWaitNextLIDoutputstateMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreeLIDoutputstateMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg);
static SickScanApiFreeLIDoutputstateMsg_PROCTYPE ptSickScanApiFreeLIDoutputstateMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextRadarScanMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec);
static SickScanApiWaitNextRadarScanMsg_PROCTYPE ptSickScanApiWaitNextRadarScanMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreeRadarScanMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanRadarScan* msg);
static SickScanApiFreeRadarScanMsg_PROCTYPE ptSickScanApiFreeRadarScanMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextLdmrsObjectArrayMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec);
static SickScanApiWaitNextLdmrsObjectArrayMsg_PROCTYPE ptSickScanApiWaitNextLdmrsObjectArrayMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreeLdmrsObjectArrayMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg);
static SickScanApiFreeLdmrsObjectArrayMsg_PROCTYPE ptSickScanApiFreeLdmrsObjectArrayMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextVisualizationMarkerMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg, double timeout_sec);
static SickScanApiWaitNextVisualizationMarkerMsg_PROCTYPE ptSickScanApiWaitNextVisualizationMarkerMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreeVisualizationMarkerMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg);
static SickScanApiFreeVisualizationMarkerMsg_PROCTYPE ptSickScanApiFreeVisualizationMarkersg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiRegisterNavPoseLandmarkMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback);
static SickScanApiRegisterNavPoseLandmarkMsg_PROCTYPE ptSickScanApiRegisterNavPoseLandmarkMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiDeregisterNavPoseLandmarkMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback);
static SickScanApiDeregisterNavPoseLandmarkMsg_PROCTYPE ptSickScanApiDeregisterNavPoseLandmarkMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiWaitNextNavPoseLandmarkMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg, double timeout_sec);
static SickScanApiWaitNextNavPoseLandmarkMsg_PROCTYPE ptSickScanApiWaitNextNavPoseLandmarkMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiFreeNavPoseLandmarkMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg);
static SickScanApiFreeNavPoseLandmarkMsg_PROCTYPE ptSickScanApiFreeNavPoseLandmarkMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiNavOdomVelocityMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanNavOdomVelocityMsg* msg);
static SickScanApiNavOdomVelocityMsg_PROCTYPE ptSickScanApiNavOdomVelocityMsg = 0;

typedef int32_t(SICK_SCAN_XD_API_CALLING_CONVENTION *SickScanApiOdomVelocityMsg_PROCTYPE)(SickScanApiHandle apiHandle, SickScanOdomVelocityMsg* msg);
static SickScanApiOdomVelocityMsg_PROCTYPE ptSickScanApiOdomVelocityMsg = 0;

/*
*  Functions and macros to initialize and close the API and a lidar
*/

// load a function by its name using GetProcAddress if not done before (i.e. if ptFunction is 0)
#define CACHE_FUNCTION_PTR(apiHandle, ptFunction, szFunctionName, procType)                              \
do                                                                                                       \
{                                                                                                        \
    if (hinstLib == 0 || apiHandle == 0)                                                                 \
    {                                                                                                    \
        printf("## ERROR SickScanApi, cacheFunctionPtr(%s): library not initialized\n", szFunctionName); \
        ptFunction = 0;                                                                                  \
    }                                                                                                    \
    else if (ptFunction == 0)                                                                            \
    {                                                                                                    \
        ptFunction = (procType)GetProcAddress(hinstLib, szFunctionName);                                 \
    }                                                                                                    \
    if (ptFunction == 0)                                                                                 \
    {                                                                                                    \
        printf("## ERROR SickScanApi, cacheFunctionPtr(%s): GetProcAddress failed\n", szFunctionName);   \
    }                                                                                                    \
}   while(0)

// Load sick_scan_xd api library (dll or so file)
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiLoadLibrary(const char* library_filepath)
{
    int32_t ret = SICK_SCAN_API_SUCCESS;
    if (hinstLib == NULL)
    {
        hinstLib = LoadLibrary(library_filepath);
    }
    if (hinstLib == NULL)
    {
        printf("## WARNING SickScanApiLoadLibrary: LoadLibrary(\"%s\") failed\n", library_filepath);
        ret = SICK_SCAN_API_NOT_LOADED;
    }
    return ret;
}

// Unload sick_scan_xd api library
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiUnloadLibrary()
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
    ptSickScanApiRelease = 0;
    ptSickScanApiInitByLaunchfile = 0;
    ptSickScanApiInitByCli = 0;
    ptSickScanApiClose = 0;
    ptSickScanApiRegisterCartesianPointCloudMsg = 0;
    ptSickScanApiDeregisterCartesianPointCloudMsg = 0;
    ptSickScanApiRegisterPolarPointCloudMsg = 0;
    ptSickScanApiDeregisterPolarPointCloudMsg = 0;
    ptSickScanApiRegisterImuMsg = 0;
    ptSickScanApiDeregisterImuMsg = 0;
    ptSickScanApiRegisterLFErecMsg = 0;
    ptSickScanApiDeregisterLFErecMsg = 0;
    ptSickScanApiRegisterLIDoutputstateMsg = 0;
    ptSickScanApiDeregisterLIDoutputstateMsg = 0;
    ptSickScanApiRegisterRadarScanMsg = 0;
    ptSickScanApiDeregisterRadarScanMsg = 0;
    ptSickScanApiRegisterLdmrsObjectArrayMsg = 0;
    ptSickScanApiDeregisterLdmrsObjectArrayMsg = 0;
    ptSickScanApiRegisterVisualizationMarkerMsg = 0;
    ptSickScanApiDeregisterVisualizationMarkerMsg = 0;
    ptSickScanApiRegisterDiagnosticMsg = 0;
    ptSickScanApiDeregisterDiagnosticMsg = 0;
    ptSickScanApiRegisterLogMsg = 0;
    ptSickScanApiDeregisterLogMsg = 0;
    ptSickScanApiGetStatus = 0;
    ptSickScanApiSendSOPAS = 0;
    ptSickScanApiSetVerboseLevel = 0;
    ptSickScanApiGetVerboseLevel = 0;
    ptSickScanApiWaitNextCartesianPointCloudMsg = 0;
    ptSickScanApiWaitNextPolarPointCloudMsg = 0;
    ptSickScanApiFreePointCloudMsg = 0;
    ptSickScanApiWaitNextImuMsg = 0;
    ptSickScanApiFreeImuMsg = 0;
    ptSickScanApiWaitNextLFErecMsg = 0;
    ptSickScanApiFreeLFErecMsg = 0;
    ptSickScanApiWaitNextLIDoutputstateMsg = 0;
    ptSickScanApiFreeLIDoutputstateMsg = 0;
    ptSickScanApiWaitNextRadarScanMsg = 0;
    ptSickScanApiFreeRadarScanMsg = 0;
    ptSickScanApiWaitNextLdmrsObjectArrayMsg = 0;
    ptSickScanApiFreeLdmrsObjectArrayMsg = 0;
    ptSickScanApiWaitNextVisualizationMarkerMsg = 0;
    ptSickScanApiFreeVisualizationMarkersg = 0;
    ptSickScanApiRegisterNavPoseLandmarkMsg = 0;
    ptSickScanApiDeregisterNavPoseLandmarkMsg = 0;
    ptSickScanApiWaitNextNavPoseLandmarkMsg = 0;
    ptSickScanApiFreeNavPoseLandmarkMsg = 0;
    ptSickScanApiNavOdomVelocityMsg = 0;
    ptSickScanApiOdomVelocityMsg = 0;
    return ret;
}

/*
*  Create an instance of sick_scan_xd api.
*  Optional commandline arguments argc, argv identical to sick_generic_caller.
*  Call SickScanApiInitByLaunchfile or SickScanApiInitByCli to process a lidar.
*/
SickScanApiHandle SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiCreate(int argc, char** argv)
{
    if (hinstLib == 0)
    {
        printf("## ERROR SickScanApiCreate: library not loaded\n");
        return 0;
    }
    if (ptSickScanApiCreate == 0)
    {
        ptSickScanApiCreate = (SickScanApiCreate_PROCTYPE)GetProcAddress(hinstLib, "SickScanApiCreate");
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
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRelease(SickScanApiHandle apiHandle)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRelease, "SickScanApiRelease", SickScanApiRelease_PROCTYPE);
    int32_t ret = (ptSickScanApiRelease ? (ptSickScanApiRelease(apiHandle)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRelease: library call SickScanApiRelease() failed, error code %d\n", ret);
    return ret;
}

// Initializes a lidar by launchfile and starts message receiving and processing
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiInitByLaunchfile(SickScanApiHandle apiHandle, const char* launchfile_args)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiInitByLaunchfile, "SickScanApiInitByLaunchfile", SickScanApiInitByLaunchfile_PROCTYPE);
    int32_t ret = (ptSickScanApiInitByLaunchfile ? (ptSickScanApiInitByLaunchfile(apiHandle, launchfile_args)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiInitByLaunchfile: library call SickScanApiInitByLaunchfile() failed, error code %d\n", ret);
    return ret;
}

// Initializes a lidar by commandline arguments and starts message receiving and processing
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiInitByCli(SickScanApiHandle apiHandle, int argc, char** argv)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiInitByCli, "SickScanApiInitByCli", SickScanApiInitByCli_PROCTYPE);
    int32_t ret = (ptSickScanApiInitByCli ? (ptSickScanApiInitByCli(apiHandle, argc, argv)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiInitByCli: library call SickScanApiInitByCli() failed, error code %d\n", ret);
    return ret;
}

// Stops message receiving and processing and closes a lidar
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiClose(SickScanApiHandle apiHandle)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiClose, "SickScanApiClose", SickScanApiClose_PROCTYPE);
    int32_t ret = (ptSickScanApiClose ? (ptSickScanApiClose(apiHandle)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiClose: library call SickScanApiClose() failed, error code %d\n", ret);
    return ret;
}

/*
*  Registration / deregistration of message callbacks
*/

// Register / deregister a callback for cartesian PointCloud messages, pointcloud in cartesian coordinates with fields x, y, z, intensity
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterCartesianPointCloudMsg, "SickScanApiRegisterCartesianPointCloudMsg", SickScanApiRegisterCartesianPointCloudMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterCartesianPointCloudMsg ? (ptSickScanApiRegisterCartesianPointCloudMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterCartesianPointCloudMsg: library call SickScanApiRegisterCartesianPointCloudMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterCartesianPointCloudMsg, "SickScanApiDeregisterCartesianPointCloudMsg", SickScanApiDeregisterCartesianPointCloudMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterCartesianPointCloudMsg ? (ptSickScanApiDeregisterCartesianPointCloudMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterCartesianPointCloudMsg: library call SickScanApiDeregisterCartesianPointCloudMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for polar PointCloud messages, pointcloud in polar coordinates with fields range, azimuth, elevation, intensity
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterPolarPointCloudMsg, "SickScanApiRegisterPolarPointCloudMsg", SickScanApiRegisterPolarPointCloudMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterPolarPointCloudMsg ? (ptSickScanApiRegisterPolarPointCloudMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterPolarPointCloudMsg: library call SickScanApiRegisterPolarPointCloudMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterPolarPointCloudMsg, "SickScanApiDeregisterPolarPointCloudMsg", SickScanApiDeregisterPolarPointCloudMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterPolarPointCloudMsg ? (ptSickScanApiDeregisterPolarPointCloudMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterPolarPointCloudMsg: library call SickScanApiDeregisterPolarPointCloudMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for Imu messages
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterImuMsg, "SickScanApiRegisterImuMsg", SickScanApiRegisterImuMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterImuMsg ? (ptSickScanApiRegisterImuMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterImuMsg: library call SickScanApiRegisterImuMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterImuMsg(SickScanApiHandle apiHandle, SickScanImuMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterImuMsg, "SickScanApiDeregisterImuMsg", SickScanApiDeregisterImuMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterImuMsg ? (ptSickScanApiDeregisterImuMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterImuMsg: library call SickScanApiDeregisterImuMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for SickScanLFErecMsg messages
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterLFErecMsg, "SickScanApiRegisterLFErecMsg", SickScanApiRegisterLFErecMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterLFErecMsg ? (ptSickScanApiRegisterLFErecMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterLFErecMsg: library call SickScanApiRegisterLFErecMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterLFErecMsg, "SickScanApiDeregisterLFErecMsg", SickScanApiDeregisterLFErecMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterLFErecMsg ? (ptSickScanApiDeregisterLFErecMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterLFErecMsg: library call SickScanApiDeregisterLFErecMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for SickScanLIDoutputstateMsg messages
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterLIDoutputstateMsg, "SickScanApiRegisterLIDoutputstateMsg", SickScanApiRegisterLIDoutputstateMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterLIDoutputstateMsg ? (ptSickScanApiRegisterLIDoutputstateMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterLIDoutputstateMsg: library call SickScanApiRegisterLIDoutputstateMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterLIDoutputstateMsg, "SickScanApiDeregisterLIDoutputstateMsg", SickScanApiDeregisterLIDoutputstateMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterLIDoutputstateMsg ? (ptSickScanApiDeregisterLIDoutputstateMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterLIDoutputstateMsg: library call SickScanApiDeregisterLIDoutputstateMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for SickScanRadarScan messages
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterRadarScanMsg, "SickScanApiRegisterRadarScanMsg", SickScanApiRegisterRadarScanMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterRadarScanMsg ? (ptSickScanApiRegisterRadarScanMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterRadarScanMsg: library call SickScanApiRegisterRadarScanMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScanCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterRadarScanMsg, "SickScanApiDeregisterRadarScanMsg", SickScanApiDeregisterRadarScanMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterRadarScanMsg ? (ptSickScanApiDeregisterRadarScanMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterRadarScanMsg: library call SickScanApiDeregisterRadarScanMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for SickScanLdmrsObjectArray messages
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterLdmrsObjectArrayMsg, "SickScanApiRegisterLdmrsObjectArrayMsg", SickScanApiRegisterLdmrsObjectArrayMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterLdmrsObjectArrayMsg ? (ptSickScanApiRegisterLdmrsObjectArrayMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterLdmrsObjectArrayMsg: library call SickScanApiRegisterLdmrsObjectArrayMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArrayCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterLdmrsObjectArrayMsg, "SickScanApiDeregisterLdmrsObjectArrayMsg", SickScanApiDeregisterLdmrsObjectArrayMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterLdmrsObjectArrayMsg ? (ptSickScanApiDeregisterLdmrsObjectArrayMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterLdmrsObjectArrayMsg: library call SickScanApiDeregisterLdmrsObjectArrayMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for VisualizationMarker messages
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterVisualizationMarkerMsg, "SickScanApiRegisterVisualizationMarkerMsg", SickScanApiRegisterVisualizationMarkerMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterVisualizationMarkerMsg ? (ptSickScanApiRegisterVisualizationMarkerMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterVisualizationMarkerMsg: library call SickScanApiRegisterVisualizationMarkerMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterVisualizationMarkerMsg, "SickScanApiDeregisterVisualizationMarkerMsg", SickScanApiDeregisterVisualizationMarkerMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterVisualizationMarkerMsg ? (ptSickScanApiDeregisterVisualizationMarkerMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterVisualizationMarkerMsg: library call SickScanApiDeregisterVisualizationMarkerMsg() failed, error code %d\n", ret);
    return ret;
}

/*
*  Functions for diagnostic and logging
*/

// Register / deregister a callback for diagnostic messages (notification in case of changed status, e.g. after errors)
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterDiagnosticMsg(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterDiagnosticMsg, "SickScanApiRegisterDiagnosticMsg", SickScanApiRegisterDiagnosticMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterDiagnosticMsg ? (ptSickScanApiRegisterDiagnosticMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterDiagnosticMsg: library call SickScanApiRegisterDiagnosticMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterDiagnosticMsg(SickScanApiHandle apiHandle, SickScanDiagnosticMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterDiagnosticMsg, "SickScanApiDeregisterDiagnosticMsg", SickScanApiDeregisterDiagnosticMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterDiagnosticMsg ? (ptSickScanApiDeregisterDiagnosticMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterDiagnosticMsg: library call SickScanApiDeregisterDiagnosticMsg() failed, error code %d\n", ret);
    return ret;
}

// Register / deregister a callback for log messages (all informational and error messages)
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterLogMsg(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterLogMsg, "SickScanApiRegisterLogMsg", SickScanApiRegisterLogMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterLogMsg ? (ptSickScanApiRegisterLogMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterLogMsg: library call SickScanApiRegisterLogMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterLogMsg(SickScanApiHandle apiHandle, SickScanLogMsgCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterLogMsg, "SickScanApiDeregisterLogMsg", SickScanApiDeregisterLogMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterLogMsg ? (ptSickScanApiDeregisterLogMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterLogMsg: library call SickScanApiDeregisterLogMsg() failed, error code %d\n", ret);
    return ret;
}

// Query current status and status message
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiGetStatus(SickScanApiHandle apiHandle, int32_t* status_code, char* message_buffer, int32_t message_buffer_size)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiGetStatus, "SickScanApiGetStatus", SickScanApiGetStatus_PROCTYPE);
    int32_t ret = (ptSickScanApiGetStatus ? (ptSickScanApiGetStatus(apiHandle, status_code, message_buffer, message_buffer_size)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiGetStatus: library call SickScanApiGetStatus() failed, error code %d\n", ret);
    return ret;
}

// Sends a SOPAS command like "sRN SCdevicestate" or "sRN ContaminationResult" and returns the lidar response
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiSendSOPAS(SickScanApiHandle apiHandle, const char* sopas_command, char* sopas_response_buffer, int32_t response_buffer_size)
{
  CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiSendSOPAS, "SickScanApiSendSOPAS", SickScanApiSendSOPAS_PROCTYPE);
  int32_t ret = (ptSickScanApiSendSOPAS ? (ptSickScanApiSendSOPAS(apiHandle, sopas_command, sopas_response_buffer, response_buffer_size)) : SICK_SCAN_API_NOT_INITIALIZED);
  if (ret != SICK_SCAN_API_SUCCESS)
    printf("## ERROR SickScanApiSendSOPAS: library call SickScanApiSendSOPAS() failed, error code %d\n", ret);
  return ret;
}

// Set verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET (equivalent to ros::console::levels),
// i.e. print messages on console above the given verbose level.
// Default verbose level is 1 (INFO), i.e. print informational, warnings and error messages.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiSetVerboseLevel(SickScanApiHandle apiHandle, int32_t verbose_level)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiSetVerboseLevel, "SickScanApiSetVerboseLevel", SickScanApiSetVerboseLevel_PROCTYPE);
    int32_t ret = (ptSickScanApiSetVerboseLevel ? (ptSickScanApiSetVerboseLevel(apiHandle, verbose_level)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiSetVerboseLevel: library call SickScanApiSetVerboseLevel() failed, error code %d\n", ret);
    return ret;
}

// Returns the current verbose level 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL or 5=QUIET. Default verbose level is 1 (INFO)
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiGetVerboseLevel(SickScanApiHandle apiHandle)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiGetVerboseLevel, "SickScanApiGetVerboseLevel", SickScanApiGetVerboseLevel_PROCTYPE);
    int32_t verbose_level = (ptSickScanApiGetVerboseLevel ? (ptSickScanApiGetVerboseLevel(apiHandle)) : SICK_SCAN_API_NOT_INITIALIZED);
    return verbose_level;
}

/*
*  Polling functions
*/

// Wait for and return the next cartesian resp. polar PointCloud messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextCartesianPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextCartesianPointCloudMsg, "SickScanApiWaitNextCartesianPointCloudMsg", SickScanApiWaitNextCartesianPointCloudMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextCartesianPointCloudMsg ? (ptSickScanApiWaitNextCartesianPointCloudMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextCartesianPointCloudMsg: library call SickScanApiWaitNextCartesianPointCloudMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextPolarPointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextPolarPointCloudMsg, "SickScanApiWaitNextPolarPointCloudMsg", SickScanApiWaitNextPolarPointCloudMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextPolarPointCloudMsg ? (ptSickScanApiWaitNextPolarPointCloudMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextPolarPointCloudMsg: library call SickScanApiWaitNextPolarPointCloudMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreePointCloudMsg(SickScanApiHandle apiHandle, SickScanPointCloudMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreePointCloudMsg, "SickScanApiFreePointCloudMsg", SickScanApiFreePointCloudMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreePointCloudMsg ? (ptSickScanApiFreePointCloudMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreePointCloudMsg: library call SickScanApiFreePointCloudMsg() failed, error code %d\n", ret);
    return ret;
}

// Wait for and return the next Imu messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextImuMsg, "SickScanApiWaitNextImuMsg", SickScanApiWaitNextImuMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextImuMsg ? (ptSickScanApiWaitNextImuMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextImuMsg: library call SickScanApiWaitNextImuMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeImuMsg(SickScanApiHandle apiHandle, SickScanImuMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreeImuMsg, "SickScanApiFreeImuMsg", SickScanApiFreeImuMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreeImuMsg ? (ptSickScanApiFreeImuMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreeImuMsg: library call SickScanApiFreeImuMsg() failed, error code %d\n", ret);
    return ret;
}

// Wait for and return the next LFErec messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextLFErecMsg, "SickScanApiWaitNextLFErecMsg", SickScanApiWaitNextLFErecMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextLFErecMsg ? (ptSickScanApiWaitNextLFErecMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextLFErecMsg: library call SickScanApiWaitNextLFErecMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeLFErecMsg(SickScanApiHandle apiHandle, SickScanLFErecMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreeLFErecMsg, "SickScanApiFreeLFErecMsg", SickScanApiFreeLFErecMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreeLFErecMsg ? (ptSickScanApiFreeLFErecMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreeLFErecMsg: library call SickScanApiFreeLFErecMsg() failed, error code %d\n", ret);
    return ret;
}

// Wait for and return the next LIDoutputstate messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextLIDoutputstateMsg, "SickScanApiWaitNextLIDoutputstateMsg", SickScanApiWaitNextLIDoutputstateMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextLIDoutputstateMsg ? (ptSickScanApiWaitNextLIDoutputstateMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextLIDoutputstateMsg: library call SickScanApiWaitNextLIDoutputstateMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeLIDoutputstateMsg(SickScanApiHandle apiHandle, SickScanLIDoutputstateMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreeLIDoutputstateMsg, "SickScanApiFreeLIDoutputstateMsg", SickScanApiFreeLIDoutputstateMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreeLIDoutputstateMsg ? (ptSickScanApiFreeLIDoutputstateMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreeLIDoutputstateMsg: library call SickScanApiFreeLIDoutputstateMsg() failed, error code %d\n", ret);
    return ret;
}

// Wait for and return the next RadarScan messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextRadarScanMsg, "SickScanApiWaitNextRadarScanMsg", SickScanApiWaitNextRadarScanMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextRadarScanMsg ? (ptSickScanApiWaitNextRadarScanMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextRadarScanMsg: library call SickScanApiWaitNextRadarScanMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeRadarScanMsg(SickScanApiHandle apiHandle, SickScanRadarScan* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreeRadarScanMsg, "SickScanApiFreeRadarScanMsg", SickScanApiFreeRadarScanMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreeRadarScanMsg ? (ptSickScanApiFreeRadarScanMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreeRadarScanMsg: library call SickScanApiFreeRadarScanMsg() failed, error code %d\n", ret);
    return ret;
}

// Wait for and return the next LdmrsObjectArray messages. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextLdmrsObjectArrayMsg, "SickScanApiWaitNextLdmrsObjectArrayMsg", SickScanApiWaitNextLdmrsObjectArrayMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextLdmrsObjectArrayMsg ? (ptSickScanApiWaitNextLdmrsObjectArrayMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextLdmrsObjectArrayMsg: library call SickScanApiWaitNextLdmrsObjectArrayMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeLdmrsObjectArrayMsg(SickScanApiHandle apiHandle, SickScanLdmrsObjectArray* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreeLdmrsObjectArrayMsg, "SickScanApiFreeLdmrsObjectArrayMsg", SickScanApiFreeLdmrsObjectArrayMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreeLdmrsObjectArrayMsg ? (ptSickScanApiFreeLdmrsObjectArrayMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreeLdmrsObjectArrayMsg: library call SickScanApiFreeLdmrsObjectArrayMsg() failed, error code %d\n", ret);
    return ret;
}

// Wait for and return the next VisualizationMarker message. Note: SickScanApiWait...Msg() allocates a message. Use function SickScanApiFree...Msg() to deallocate it after use.
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextVisualizationMarkerMsg, "SickScanApiWaitNextVisualizationMarkerMsg", SickScanApiWaitNextVisualizationMarkerMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextVisualizationMarkerMsg ? (ptSickScanApiWaitNextVisualizationMarkerMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextVisualizationMarkerMsg: library call SickScanApiWaitNextVisualizationMarkerMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeVisualizationMarkerMsg(SickScanApiHandle apiHandle, SickScanVisualizationMarkerMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreeVisualizationMarkersg, "SickScanApiFreeVisualizationMarkerMsg", SickScanApiFreeVisualizationMarkerMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreeVisualizationMarkersg ? (ptSickScanApiFreeVisualizationMarkersg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreeVisualizationMarkerMsg: library call SickScanApiFreeVisualizationMarkerMsg() failed, error code %d\n", ret);
    return ret;
}

/*
*  NAV350 support
*/

int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiRegisterNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiRegisterNavPoseLandmarkMsg, "SickScanApiRegisterNavPoseLandmarkMsg", SickScanApiRegisterNavPoseLandmarkMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiRegisterNavPoseLandmarkMsg ? (ptSickScanApiRegisterNavPoseLandmarkMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiRegisterNavPoseLandmarkMsg: library call SickScanApiRegisterNavPoseLandmarkMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiDeregisterNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkCallback callback)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiDeregisterNavPoseLandmarkMsg, "SickScanApiDeregisterNavPoseLandmarkMsg", SickScanApiDeregisterNavPoseLandmarkMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiDeregisterNavPoseLandmarkMsg ? (ptSickScanApiDeregisterNavPoseLandmarkMsg(apiHandle, callback)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiDeregisterNavPoseLandmarkMsg: library call SickScanApiDeregisterNavPoseLandmarkMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiWaitNextNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg, double timeout_sec)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiWaitNextNavPoseLandmarkMsg, "SickScanApiWaitNextNavPoseLandmarkMsg", SickScanApiWaitNextNavPoseLandmarkMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiWaitNextNavPoseLandmarkMsg ? (ptSickScanApiWaitNextNavPoseLandmarkMsg(apiHandle, msg, timeout_sec)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS && ret != SICK_SCAN_API_TIMEOUT)
        printf("## ERROR SickScanApiWaitNextNavPoseLandmarkMsg: library call SickScanApiWaitNextNavPoseLandmarkMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiFreeNavPoseLandmarkMsg(SickScanApiHandle apiHandle, SickScanNavPoseLandmarkMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiFreeNavPoseLandmarkMsg, "SickScanApiFreeNavPoseLandmarkMsg", SickScanApiFreeNavPoseLandmarkMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiFreeNavPoseLandmarkMsg ? (ptSickScanApiFreeNavPoseLandmarkMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiFreeNavPoseLandmarkMsg: library call SickScanApiFreeNavPoseLandmarkMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiNavOdomVelocityMsg(SickScanApiHandle apiHandle, SickScanNavOdomVelocityMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiNavOdomVelocityMsg, "SickScanApiNavOdomVelocityMsg", SickScanApiNavOdomVelocityMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiNavOdomVelocityMsg ? (ptSickScanApiNavOdomVelocityMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiNavOdomVelocityMsg: library call SickScanApiNavOdomVelocityMsg() failed, error code %d\n", ret);
    return ret;
}
int32_t SICK_SCAN_XD_API_CALLING_CONVENTION SickScanApiOdomVelocityMsg(SickScanApiHandle apiHandle, SickScanOdomVelocityMsg* msg)
{
    CACHE_FUNCTION_PTR(apiHandle, ptSickScanApiOdomVelocityMsg, "SickScanApiOdomVelocityMsg", SickScanApiOdomVelocityMsg_PROCTYPE);
    int32_t ret = (ptSickScanApiOdomVelocityMsg ? (ptSickScanApiOdomVelocityMsg(apiHandle, msg)) : SICK_SCAN_API_NOT_INITIALIZED);
    if (ret != SICK_SCAN_API_SUCCESS)
        printf("## ERROR SickScanApiOdomVelocityMsg: library call SickScanApiOdomVelocityMsg() failed, error code %d\n", ret);
    return ret;
}
