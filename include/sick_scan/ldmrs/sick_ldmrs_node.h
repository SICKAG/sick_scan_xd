#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
** @brief ros node to integrate LDMRS driver into sick_scan
*/
#ifndef __SICK_SCAN_LDMRS_NODE_INCLUDED
#define __SICK_SCAN_LDMRS_NODE_INCLUDED

#if defined LDMRS_SUPPORT && LDMRS_SUPPORT > 0

#include <sick_scan/sick_ros_wrapper.h>

#include <sick_scan/ldmrs/sick_ldmrs_driver.hpp>
#include <sick_ldmrs/devices/LD_MRS.hpp>
#include "sick_scan/sick_scan_common.h"

namespace sick_scan_xd
{
  class SickLdmrsNode
  {
  public:
    SickLdmrsNode();
    ~SickLdmrsNode();

    virtual int init(rosNodePtr node, const std::string & hostName = "192.168.0.1", const std::string & frameId = "cloud");

    virtual int run();

  protected:

    rosNodePtr m_nh;
    std::shared_ptr<diagnostic_updater::Updater> m_diagnostics;
    Manager* m_manager;
    sick_ldmrs_driver::SickLDMRS* m_app;
    devices::LDMRS* m_ldmrs;
  };

} // namespace sick_scan_xd

#endif // LDMRS_SUPPORT && LDMRS_SUPPORT > 0
#endif // __SICK_SCAN_LDMRS_NODE_INCLUDED
