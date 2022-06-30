#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__
#define __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__

#include <mutex>

namespace dynamic_reconfigure
{
  extern std::mutex __init_mutex__;
}

#endif
