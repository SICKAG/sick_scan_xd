/* $Id: scanf.c,v 1.2 2002/08/09 20:56:57 pefo Exp $ */

/*
 * Copyright (c) 2000-2002 Opsycon AB  (www.opsycon.se)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Opsycon AB.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

//#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <vector>
#include "sick_scan/binScanf.hpp"

#define MAXLN (10 * 1024)

#define ISSPACE " \t\n\r\f\v"

int binIsspace(int val)
{
  if (val < 0) // characters >= 80
  {
    val = (int) (val & 0xFF);
  }
  int ret = isspace(val);
  return (ret);
}

static char *
_binGetbase(char *p, int *basep)
{
  if (p[0] == '0')
  {
    switch (p[1])
    {
      case 'x':
        *basep = 16;
        break;
      case 't':
      case 'n':
        *basep = 10;
        break;
      case 'o':
        *basep = 8;
        break;
      default:
        *basep = 10;
        return (p);
    }
    return (p + 2);
  }
  *basep = 10;
  return (p);
}

static int
_binAtob(unsigned long *vp, char *p, int base)
{
  unsigned long value, v1, v2;
  char *q, tmp[20];
  int digit;

  if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
  {
    base = 16;
    p += 2;
  }

  if (base == 16 && (q = strchr(p, '.')) != 0)
  {
    if (q - p > sizeof(tmp) - 1)
    {
      return (0);
    }

    strncpy(tmp, p, q - p);
    tmp[q - p] = '\0';
    if (!_binAtob(&v1, tmp, 16))
    {
      return (0);
    }

    q++;
    if (strchr(q, '.'))
    {
      return (0);
    }

    if (!_binAtob(&v2, q, 16))
    {
      return (0);
    }
    *vp = (v1 << 16) + v2;
    return (1);
  }

  value = *vp = 0;
  for (; *p; p++)
  {
    if (*p >= '0' && *p <= '9')
    {
      digit = *p - '0';
    }
    else if (*p >= 'a' && *p <= 'f')
    {
      digit = *p - 'a' + 10;
    }
    else if (*p >= 'A' && *p <= 'F')
    {
      digit = *p - 'A' + 10;
    }
    else
    {
      return (0);
    }

    if (digit >= base)
    {
      return (0);
    }
    value *= base;
    value += digit;
  }
  *vp = value;
  return (1);
}

/*
*  atob(vp,p,base)
*      converts p to binary result in vp, rtn 1 on success
*/
int binAtob(unsigned long *vp, char *p, int base)
{

  unsigned long v;

  if (base == 0)
  {
    p = _binGetbase(p, &base);
  }
  if (_binAtob(&v, p, base))
  {
    *vp = v;
    return (1);
  }
  return (0);
}


/*
 *  scanf(fmt,va_alist)
 */
int
binSscanf(const char *fmt, ...)
{
  int count;
  va_list ap;

  va_start(ap, fmt);
  count = vfscanf(stdin, fmt, ap);
  va_end(ap);
  return (count);
}

/*
 *  fscanf(fp,fmt,va_alist)
 */
int
binFscanf(FILE *fp, const char *fmt, ...)
{
  int count;
  va_list ap;

  va_start(ap, fmt);
  count = vfscanf(fp, fmt, ap);
  va_end(ap);
  return (count);
}

/*
 *  sscanf(buf,fmt,va_alist)
 */
int
binSscanf(const char *buf, const char *fmt, ...)
{
  int count;
  va_list ap;

  va_start(ap, fmt);
  count = vsscanf(buf, fmt, ap);
  va_end(ap);
  return (count);
}

/*
 *  vfscanf(fp,fmt,ap)
 */
static int
binVfscanf(FILE *fp, const char *fmt, va_list ap)
{
  int count;
  char buf[MAXLN + 1];

  if (fgets(buf, MAXLN, fp) == 0)
  {
    return (-1);
  }
  count = vsscanf(buf, fmt, ap);
  return (count);
}


/*!
\brief sscanf-like function
       In addition to standard sscanf a format identifier "y" is introduced to parse big endian
\param bufOrg: Ptr. to original data
\param s: format identifier
\param ap: variable numer of arguments
\param bufLen: length of buffer in bytes
\return number of parsed arguments
*/

static int
binVsscanf(const char *bufOrg, const char *s, va_list ap, int bufLen)
{
  int count, noassign, base, lflag;
  unsigned long width;
  const char *tc;
  char *t, tmp[MAXLN];
  const char *buf;
  const char *bufEnd;
  buf = bufOrg;
  bufEnd = buf + bufLen;

  count = noassign = width = lflag = base = 0;
  // Parse Inputbuffer buf
  while (*s && (buf < bufEnd))
  {
    while (binIsspace(*s)) // skipping blanks
      s++;
    if (*s == '%')
    { // maybe a format identifier
      s++;
      for (; *s; s++)
      {
        if (strchr("dibouxycsefg%", *s))
        { // check allowed format identifier
          break;
        }
        if (*s == '*')
        {  // length identifier
          noassign = 1;
        }
        else if (*s == 'l' || *s == 'L')
        { // long flag
          lflag = 1;
        }
        else if (*s >= '1' && *s <= '9')
        {  // length of argument
          for (tc = s; isdigit(*s); s++);
          strncpy(tmp, tc, s - tc);
          tmp[s - tc] = '\0';
          binAtob((unsigned long *) &width, tmp, 10);
          s--;
        }
      }
      if (*s == 's')
      {
        while (binIsspace((int) (0xFF & *buf)))   // must be done to handle data >= 0x80
        {
          buf++;
        }
        if (!width)
        {
          width = (int) strcspn(buf, ISSPACE);
        }
        if (!noassign)
        {
          strncpy(t = va_arg(ap, char *), buf, width);
          t[width] = '\0';
        }
        buf += width;
      }
      else if (*s == 'c')
      {
        if (!width)
        {
          width = 1;
        }
        if (!noassign)
        {
          strncpy(t = va_arg(ap, char *), buf, width);
          t[width] = '\0';
        }
        buf += width;
      }
      else if (strchr("dobxyu", *s))
      {
        while (binIsspace((int) (0xFF & *buf)))   // must be done to handle data >= 0x80
        {
          buf++;
        }
        if (*s == 'd' || *s == 'u')
        {
          base = 10;
        }
        else if (*s == 'x')
        {
          base = 16;
        }
        else if (*s == 'o')
        {
          base = 8;
        }
        else if (*s == 'b')
        {
          base = 2;
        }
        else if (*s == 'y')
        {
          base = 1;
        } // use as marker for binary scan
        if (!width)
        {
          if (binIsspace(*(s + 1)) || *(s + 1) == 0)
          {
            width = (int) strcspn(buf, ISSPACE);
          }
          else
          {
            width = (int) (strchr(buf, *(s + 1)) - buf);
          }
        }
        if (base == 1)  // binary data - plain copy without string function
        {
          memcpy(tmp, buf, width);
          unsigned char *destAdr = va_arg(ap, unsigned char *);
          unsigned long destVal = 0;
          for (int i = 0; i < (int)width; i++)  // Big Endian - MSB first - convention for SOPAS-Binary
          {
            destVal <<= 8;
            destVal |= (unsigned char) (0xFF & tmp[i]);
          }
          for (int i = 0; i < (int)width; i++)  // Big Endian - MSB first - convention for SOPAS-Binary
          {
            unsigned char val = (unsigned char) (0xFF & (destVal >> (i * 8)));
            destAdr[i] = val;
          }
        }
        else
        {
          strncpy(tmp, buf, width);
          tmp[width] = '\0';
          if (!noassign)
          {
            binAtob(va_arg(ap, unsigned long *), tmp, base);
          }
        }
        buf += width;
      }
      if (!noassign)
      {
        count++;
      }
      width = noassign = lflag = 0;
      s++;
    }
    else
    {
      while (binIsspace(*buf))
        buf++;
      if (*s != *buf)
      {
        break;
      }
      else
      {
        s++, buf++;
      }
    }
  }
  return (count);
}

/*
*  scanf(fmt,va_alist)
*/
int
binScanfVec(const std::vector<unsigned char> *vec, const char *fmt, ...)
{
  if (vec->size() == 0)
  {
    return (0);
  }
  const char *buf = (const char *) (&(*vec)[0]);
  int count;
  int bufLen = (int) vec->size();
  if (0 == bufLen)
  {
    return (0);
  }
  va_list ap;

  va_start(ap, fmt);
  count = binVsscanf(buf, fmt, ap, bufLen);
  va_end(ap);
  return (count);
}

int binScanfGuessDataLenFromMask(const char *scanfMask)
{
  int retLen = 0;
  int noassign = 0;
  int lflag = 0;
  char tmp[20];
  int width;
  if (scanfMask == NULL)
  {
  }
  else
  {
    const char *s = scanfMask;
    const char *tc = NULL;
    while (*s)
    {
      while (binIsspace(*s))
      {
        s++;
        retLen++;
      }
      if (*s == '%')
      {
        s++;
        for (; *s; s++)
        {
          if (strchr("dibouxycsefg%", *s))
          {
            break;
          }
          if (*s == '*')
          {
            noassign = 1;
          }
          else if (*s == 'l' || *s == 'L')
          {
            lflag = 1;
          }
          else if (*s >= '1' && *s <= '9')
          {
            for (tc = s; isdigit(*s); s++);
            strncpy(tmp, tc, s - tc);
            tmp[s - tc] = '\0';
            sscanf(tmp, "%d", &width);
            retLen += width;
          }
        }
      }
      else
      {
        s++;
        retLen++;
      }
    }

  }
  return (retLen);
}