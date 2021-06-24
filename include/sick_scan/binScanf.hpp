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
