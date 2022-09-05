#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
//
// Created by michael on 10/11/18.
//

#ifndef SICK_SCAN_CONFIG_H
#define SICK_SCAN_CONFIG_H

#include <fstream>
#include <map>
#include <string>

class SickScanConfigInternal
{
public:
  static SickScanConfigInternal &get()
  {
    static SickScanConfigInternal instance;
    return instance;
  }

  void setValue(std::string key, std::string val);

  void setValue(std::string key, bool val);

  std::string getValue(std::string key);

  bool getBoolValue(std::string key);

private:
  SickScanConfigInternal()
  {};

  SickScanConfigInternal(const SickScanConfigInternal &);

  SickScanConfigInternal &operator=(const SickScanConfigInternal &);

  std::map<std::string, std::string> storedConfig;
};

#endif //SICK_SCAN_SICK_SCAN_CONFIG_H
