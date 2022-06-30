#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef BINPRINTF_HPP
#define BINPRINTF_HPP

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <vector>
#include <string>

int binSprintf(char *out, const char *format, ...);

int binSprintfVec(std::vector<unsigned char> *outvec, const char *fmt, ...);

std::string binDumpVecToString(const std::vector<unsigned char> *outvec, bool appendReadableText = false);

#endif
