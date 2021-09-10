//
// Created by rosuser on 11.07.19.
//

#ifndef SICK_SCAN_BASE_LAUNCHPARSER_H
#define SICK_SCAN_BASE_LAUNCHPARSER_H

#include <string>
#include <vector>

class LaunchParser
{
public:
  LaunchParser()
  {};
  bool parseFile(std::string launchFileFullName, std::vector<std::string>& nameVec,
                               std::vector<std::string>& typeVec, std::vector<std::string>& valVec);
};


#endif //SICK_SCAN_BASE_LAUNCHPARSER_H
