/*
  Copyright 2001 Georges Menie
  https://www.menie.org/georges/embedded/small_printf_source_code.html

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "sick_scan/binPrintf.hpp"
#include <stdio.h>

#define BIN_SPRINTF_MAX_LEN (10240)

static void binPrintchar(char **str, int c)
{
  extern int putchar(int c);
  if (str)
  {
    **str = c;
    ++(*str);
  }
  else
  { (void) putchar(c); }
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int binPrints(char **out, const char *string, int width, int pad)
{
  int pc = 0, padchar = ' ';

  if (width > 0)
  {
    int len = 0;
    const char *ptr;
    for (ptr = string; *ptr; ++ptr) ++len;
    if (len >= width)
    { width = 0; }
    else
    { width -= len; }
    if (pad & PAD_ZERO)
    { padchar = '0'; }
  }
  if (!(pad & PAD_RIGHT))
  {
    for (; width > 0; --width)
    {
      binPrintchar(out, padchar);
      ++pc;
    }
  }
  for (; *string; ++string)
  {
    binPrintchar(out, *string);
    ++pc;
  }
  for (; width > 0; --width)
  {
    binPrintchar(out, padchar);
    ++pc;
  }

  return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int binPrinti(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
  char print_buf[PRINT_BUF_LEN];
  char *s;
  int t, neg = 0, pc = 0;
  unsigned int u = i;

  if (b == 1)
  {
    int cnt = 0;
    while (width > 0)
    {
      ++pc;
      ++cnt;
      --width;
      char ch = (char) (0xFF & (i >> (width * 8)));
      binPrintchar(out, ch);
    }
    return (cnt);
  }

  if (i == 0)
  {
    print_buf[0] = '0';
    print_buf[1] = '\0';
    return binPrints(out, print_buf, width, pad);
  }

  if (sg && b == 10 && i < 0)
  {
    neg = 1;
    u = -i;
  }

  s = print_buf + PRINT_BUF_LEN - 1;
  *s = '\0';

  while (u)
  {
    t = u % b;
    if (t >= 10)
    {
      t += letbase - '0' - 10;
    }
    *--s = t + '0';
    u /= b;
  }

  if (neg)
  {
    if (width && (pad & PAD_ZERO))
    {
      binPrintchar(out, '-');
      ++pc;
      --width;
    }
    else
    {
      *--s = '-';
    }
  }

  return pc + binPrints(out, s, width, pad);
}

static int binPrint(char **out, long long *varg)
{
  int width, pad;
  int pc = 0;
  char *format = (char *) (*varg++);
  char scr[2];

  for (; *format != 0; ++format)
  {
    if (*format == '%')
    {
      ++format;
      width = pad = 0;
      if (*format == '\0')
      { break; }
      if (*format == '%')
      { goto out; }
      if (*format == '-')
      {
        ++format;
        pad = PAD_RIGHT;
      }
      while (*format == '0')
      {
        ++format;
        pad |= PAD_ZERO;
      }
      for (; *format >= '0' && *format <= '9'; ++format)
      {
        width *= 10;
        width += *format - '0';
      }
      if (*format == 's')
      {
        char *s = *((char **) varg++);
        pc += binPrints(out, s ? s : "(null)", width, pad);
        continue;
      }
      if (*format == 'd')
      {
        pc += binPrinti(out, (int)(*varg++), 10, 1, width, pad, 'a');
        continue;
      }
      if (*format == 'x')
      {
        pc += binPrinti(out, (int)(*varg++), 16, 0, width, pad, 'a');
        continue;
      }
      if (*format == 'X')
      {
        pc += binPrinti(out, (int)(*varg++), 16, 0, width, pad, 'A');
        continue;
      }
      if (*format == 'y')
      {
        pc += binPrinti(out, (int)(*varg++), 1, 0, width, pad, 'A');
        continue;
      }
      if (*format == 'u')
      {
        pc += binPrinti(out, (int)(*varg++), 10, 0, width, pad, 'a');
        continue;
      }
      if (*format == 'c')
      {
        /* char are converted to int then pushed on the stack */
        scr[0] = (char)(*varg++);
        scr[1] = '\0';
        pc += binPrints(out, scr, width, pad);
        continue;
      }
    }
    else
    {
      out:
      binPrintchar(out, *format);
      ++pc;
    }
  }
  if (out)
  { **out = '\0'; }
  return pc;
}

/* assuming sizeof(void *) == sizeof(int) */

int binPrintf(const char *format, ...)
{
  long long *varg = (long long *) (&format);
  return binPrint(0, varg);
}

int binSprintf(char *out, const char *format, ...)
{
  long long *varg = (long long *) (&format);
  return binPrint(&out, varg);
}

int binSprintfVec(std::vector<unsigned char> *outvec, const char *fmt, ...)
{
  outvec->clear();
  char buffer[BIN_SPRINTF_MAX_LEN];
  char *bufferPtr = &(buffer[0]);
  void *tttt = &fmt;
  long long *varg = (long long *) (&fmt);
  int retCode = binPrint(&bufferPtr, varg);
  if (retCode > 0)
  {
    for (int i = 0; i < retCode; i++)
    {
      outvec->push_back(buffer[i]);
    }
  }

  return retCode;
}


std::string binDumpVecToString(const std::vector<unsigned char> *outvec, bool appendReadableText /*= false*/)
{
  std::string s;
  for (int i = 0; i < outvec->size(); i++)
  {
    char szDummy[255] = {0};
    sprintf(szDummy, "%02x ", (int) (0xFF & (*outvec)[i]));
    s += szDummy;
  }
  if (appendReadableText)
  {
    for (int i = 0; i < outvec->size(); i++)
    {
      char szDummy[255] = {0};
      sprintf(szDummy, "%c", (*outvec)[i] < 0x20 ? '.' : (*outvec)[i]);
      s += szDummy;
    }
  }
  return (s);
}



