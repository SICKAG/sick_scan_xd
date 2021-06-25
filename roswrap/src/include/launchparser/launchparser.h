//
// Created by rosuser on 11.07.19.
//

#ifndef SICK_SCAN_BASE_LAUNCHPARSER_H
#define SICK_SCAN_BASE_LAUNCHPARSER_H

#include "boost/filesystem.hpp"
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm> // for std::min


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#ifndef _MSC_VER
#include <sys/wait.h>
#endif
#include <unistd.h>
#include <cstdio>
#include <cstdlib>

#include <string>
#include <map>

#include "tinystr.h"
#include "tinyxml.h"


class LaunchParser
{
public:
  LaunchParser()
  {};
  bool parseFile(std::string launchFileFullName, std::vector<std::string>& nameVec,
                               std::vector<std::string>& typeVec, std::vector<std::string>& valVec);
};


#endif //SICK_SCAN_BASE_LAUNCHPARSER_H
