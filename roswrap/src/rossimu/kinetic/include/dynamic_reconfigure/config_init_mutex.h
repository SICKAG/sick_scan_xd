#ifndef __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__
#define __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__

#include <mutex>

namespace dynamic_reconfigure
{
  extern std::mutex __init_mutex__;
}

#endif
