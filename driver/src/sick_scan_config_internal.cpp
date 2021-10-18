//
// Created by michael on 10/11/18.
//

#include "sick_scan/sick_scan_config_internal.h"
#include <string>
#include <map>

void SickScanConfigInternal::setValue(std::string key, std::string val)
{
  storedConfig[key] = val;

}


void SickScanConfigInternal::setValue(std::string key, bool val)
{
  std::string valStr;
  if (val == true)
  {
    valStr = "True";
  }
  else
  {
    valStr = "False";
  }
  storedConfig[key] = valStr;

}

std::string SickScanConfigInternal::getValue(std::string key)
{
  std::string tmp;
  tmp = storedConfig[key];
  return tmp;
}

bool SickScanConfigInternal::getBoolValue(std::string key)
{
  bool retVal = false;
  std::string tmp;
  tmp = storedConfig[key];
  if (tmp.size() > 0)
  {
    if ((tmp[0] == 't') || (tmp[0] == 'T'))
    {
      retVal = false;
    }

  }
  return (retVal);
}