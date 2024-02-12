#include <chrono>
#include <iostream>
#include <thread>

#include "sick_scan_xd_api/sick_scan_api.h"

// Implement a callback to process pointcloud messages
void customizedPointCloudMsgCb(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
{
  std::cout << "C++ PointCloudMsgCb: " << msg->width << " x " << msg->height << " pointcloud message received" << std::endl; // data processing to be done
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
  std::this_thread::sleep_for(std::chrono::seconds(15));
  // std::this_thread::sleep_for(std::chrono::hours(24));

  // Close lidar and release sick_scan api
  SickScanApiDeregisterCartesianPointCloudMsg(apiHandle, &customizedPointCloudMsgCb);
  SickScanApiClose(apiHandle);
  SickScanApiRelease(apiHandle);
  SickScanApiUnloadLibrary();
}
