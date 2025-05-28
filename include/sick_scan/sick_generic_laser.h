#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef SICK_GENERIC_LASER_H
#define SICK_GENERIC_LASER_H

#include <sick_scan/sick_scan_common_tcp.h>

enum NodeRunState
{
  scanner_init, scanner_run, scanner_finalize
};


bool startGenericLaser(int argc, char **argv, std::string nodeName, rosNodePtr nhPriv, int* exit_code);

void joinGenericLaser(void);

int mainGenericLaser(int argc, char **argv, std::string scannerName, rosNodePtr nh);

void rosSignalHandler(int signalRecv);

void setVersionInfo(std::string _versionInfo);

std::string getVersionInfo();

bool parseLaunchfileSetParameter(rosNodePtr nhPriv, int argc, char **argv);

bool stopScannerAndExit(bool force_immediate_shutdown = false);

bool convertSendSOPASCommand(const std::string& sopas_ascii_request, std::string& sopas_response, bool wait_for_reply = true);

#endif

