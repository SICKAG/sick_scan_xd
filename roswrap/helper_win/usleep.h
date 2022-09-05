#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef  USLEEP_H
#define USLEEP_H
#include <windows.h>
  extern "C" { 
void usleep(__int64 usec) ;
  }
#endif
