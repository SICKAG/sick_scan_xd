#include <stdio.h>
#ifdef _MSC_VER
#include <Windows.h> // requrired for Sleep()
#define sleep(x) Sleep(x*1000)
#else
#include <unistd.h> // requrired for sleep()
#endif

#include "sick_scan_xd_api/sick_scan_api.h"

// Implement a callback to process pointcloud messages
void customizedPointCloudMsgCb(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{
  printf("C PointCloudMsgCb: %d x %d pointcloud message received\n", msg->width, msg->height); // data processing to be done
}

int main(int argc, char** argv)
{
  // Create a sick_scan instance and initialize lidar with commandline arguments
#ifdef _MSC_VER
	const char* sick_scan_api_lib = "sick_scan_xd_shared_lib.dll";
#else
	const char* sick_scan_api_lib = "libsick_scan_xd_shared_lib.so";
#endif
  SickScanApiLoadLibrary(sick_scan_api_lib);
  SickScanApiHandle apiHandle = SickScanApiCreate(argc, argv);
  SickScanApiInitByCli(apiHandle, argc, argv);

  // Register for pointcloud messages
  SickScanApiRegisterCartesianPointCloudMsg(apiHandle, &customizedPointCloudMsgCb);

  // Run application or main loop
  // getchar();
  sleep(15);

  // Close lidar and release sick_scan api
  SickScanApiDeregisterCartesianPointCloudMsg(apiHandle, &customizedPointCloudMsgCb);
  SickScanApiClose(apiHandle);
  SickScanApiRelease(apiHandle);
  SickScanApiUnloadLibrary();
}
