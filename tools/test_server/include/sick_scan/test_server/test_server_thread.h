#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/*
 * @brief sick_scan2 test_server_thread implements a simple tcp server thread,
 * simulating a lidar device for unittests. It runs a thread to listen
 * and accept tcp connections from clients and generates telegrams to test
 * the sick_scan2 ros driver.
 *
 * Note: sick_scan2 test_server does not implement the functions of lidar sensor,
 * it just implements a simple tcp server, accepting tcp connections from clients
 * and generating telegrams to test the sick_scan2 ros drivers.
 *
 * Copyright (C) 2020 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SICK_SCAN_TEST_SERVER_THREAD_H_INCLUDED
#define __SICK_SCAN_TEST_SERVER_THREAD_H_INCLUDED

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
//#include <boost/make_shared.hpp>
#include "sick_scan/sick_ros_wrapper.h"
#include "sick_scan/sick_generic_laser.h"
#include "sick_scan/sick_scan_common.h"

namespace sick_scan_xd
{
  namespace test
  {
    /*!
    * class TestServerThread runs a thread to listen and accept tcp connections from clients
    * and generates telegrams to test sick_scan ros driver.
    */
    class TestServerThread
    {
    public:

      /*!
      * Constructor. The server thread does not start automatically, call start() and stop() to start and stop the server.
      * @param[in] nh ros node handle
      * @param[in] scanner_name scanner type, f.e. "sick_ldmrs"
      * @param[in] ip_port ip port for tcp connections, default: 2112
      */
      TestServerThread(rosNodePtr nh = 0, const std::string & scanner_name = "undefined", int ip_port = 2112);

      /*!
      * Destructor. Stops the server thread and closes all tcp connections.
      */
      virtual ~TestServerThread();

      /*!
      * Starts the server thread, starts to listen and accept tcp connections from clients.
      * @return true on success, false on failure.
      */
      virtual bool start(void);

      /*!
      * Stops the server thread and closes all tcp connections.
      * @return true on success, false on failure.
      */
      virtual bool stop(void);

    protected:

      /*
        * @brief Thread callback, runs the tcp communication with clients
        */
      bool run(void);

      rosNodePtr m_nh;                  // ros node handle
      std::string m_scanner_name;       // scanner type, f.e. "sick_ldmrs"
      int m_ip_port;                    // ip port for tcp connections
      std::thread* m_server_thread;     // background thread running tcp communication with clients
      bool m_run_server_thread;         // flag to start and stop m_server_thread

    }; // class TestServerThread

  } // namespace test
} // namespace sick_scan_xd
#endif // __SICK_SCAN_TEST_SERVER_THREAD_H_INCLUDED
