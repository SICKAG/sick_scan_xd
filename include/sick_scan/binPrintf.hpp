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

std::string binDumpVecToString(std::vector<unsigned char> *outvec, bool appendReadableText = false);

#endif
