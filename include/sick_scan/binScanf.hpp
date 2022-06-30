#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef BINSCANF_HPP
#define BINSCANF_HPP

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <vector>

int
binSscanf(const char *fmt, ...);

int
binScanfVec(const std::vector<unsigned char> *vec, const char *fmt, ...);

int
binSscanf(const char *buf, const char *fmt, ...);

int binScanfGuessDataLenFromMask(const char *scanfMask);

#endif
