#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/**
 * @file scansegment_threads.h
 * @brief scansegement_threads runs all threads to receive, convert and publish scan data for the sick 3D lidar multiScan136.
 *
 * Copyright (C) 2022-2026, SICK AG, Waldkirch
 * Copyright (C) 2022-2026, Ing.-Buero Dr. Michael Lehning, Hildesheim
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @date 2022 (original), September 2026 (updated)
 * @author Michael Lehning <michael.lehning@lehning.de>
 */
#ifndef __SICK_SCANSEGEMENT_THREADS_H
#define __SICK_SCANSEGEMENT_THREADS_H

namespace sick_scan_xd
{
  class SickScanServices;
}

#include "sick_scansegment_xd/config.h"
#include <atomic>

namespace sick_scansegment_xd
{
    /*
	  * @brief Initializes and runs all threads to receive, convert and publish scan data for the sick 3D lidar multiScan136.
	  */
    int run(rosNodePtr node, const std::string& scannerName);

    /*
     * @brief Stops msgpack threads from external code (e.g., lifecycle node cleanup).
     * This function can be called to stop the msgpack threads without waiting for them to finish naturally.
     */
    void stopMsgPackThreads();

    /*
	  * @brief class MsgPackThreads runs all threads to receive, convert and publish scan data for the sick 3D lidar multiScan136.
	  */
    class MsgPackThreads
    {
    public:

        /*
	     * @brief MsgPackThreads constructor
	     */
        MsgPackThreads();

        /*
	     * @brief MsgPackThreads destructor
	     */
        ~MsgPackThreads();

        /*
	     * @brief Initializes msgpack receiver, converter and publisher and starts msgpack handling and publishing in a background thread.
	     */
        bool start(const sick_scansegment_xd::Config& config);

        /*
	     * @brief Stops running threads and closes msgpack receiver, converter and publisher.
	     */
        bool stop(bool do_join);

        /*
	     * @brief Joins running threads and returns after they finished.
	     */
        void join(void);

        /*
         * @brief Returns whether the scansegment thread is running.
         */
	      bool getRunScansegmentThread() const
        {
            return m_run_scansegment_thread.load();
        }

        /*
         * @brief Sets the flag controlling the execution of the scansegment thread.
         */
        void setRunScansegmentThread(bool run)
        {
            m_run_scansegment_thread.store(run);
        }
    protected:

        /*
        * @brief Thread callback, initializes and runs msgpack receiver, converter and publisher.
        */
        bool runThreadCb(void);

       sick_scansegment_xd::Config m_config;                      // sick_scansegment_xd configuration
       std::thread* m_scansegment_thread;                         // background thread to convert msgpack to ScanSegmentParserOutput data
       /**
        * @brief Flag controlling execution of the scan-segment thread.
        *
        * The flag is accessed through getRunScansegmentThread() and
        * setRunScansegmentThread() for lifecycle control.
        */
	   std::atomic<bool> m_run_scansegment_thread{false};
    
    };

    sick_scan_xd::SickScanServices* sopasService();

}   // namespace sick_scansegment_xd
#endif // __SICK_SCANSEGEMENT_THREADS_H
