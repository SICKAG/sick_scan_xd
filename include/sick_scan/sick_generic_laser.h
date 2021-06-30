#ifndef SICK_GENERIC_LASER_H
#define SICK_GENERIC_LASER_H
/*
 *
 */
#ifndef _MSC_VER
#endif

#include <sick_scan/sick_scan_common_tcp.h>


int mainGenericLaser(int argc, char **argv, std::string scannerName, rosNodePtr nh);

void rosSignalHandler(int signalRecv);

void setVersionInfo(std::string _versionInfo);

std::string getVersionInfo();

#endif

