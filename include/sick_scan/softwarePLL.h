#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef SOFTWARE_PLL_H
#define SOFTWARE_PLL_H

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

#include <map>
#include <string>
#include <iostream>
#include <fstream>      // std::ifstream
#include <cstring>
#include <sstream>      // std::stringstream
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <ctime>
#include <cstdint>

class SoftwarePLL
{
public:
  static SoftwarePLL &instance()
  {
    static SoftwarePLL _instance;
    return _instance;
  }

  ~SoftwarePLL()
  {}

  bool pushIntoFifo(double curTimeStamp, uint64_t curtickTransmit, uint64_t curtickScan); // update tick fifo and update clock (timestamp) fifo;
  double extraPolateRelativeTimeStamp(uint64_t cur_tick, uint64_t first_tick);

  bool getCorrectedTimeStamp(uint32_t &sec, uint32_t &nanoSec, uint32_t tick);
  bool getCorrectedTimeStamp(uint32_t &sec, uint32_t &nanoSec, uint64_t tick);

  bool convSystemtimeToLidarTimestamp(uint32_t systemtime_sec, uint32_t systemtime_nanosec, uint32_t& tick);
  bool convSystemtimeToLidarTimestamp(uint32_t systemtime_sec, uint32_t systemtime_nanosec, uint64_t& tick);

  bool getDemoFileData(std::string fileName, std::vector<uint32_t> &tickVec, std::vector<uint32_t> &secVec,
                       std::vector<uint32_t> &nanoSecVec);

  static void testbed();

  bool IsInitialized() const
  { 
    if (ticksToTimestampMode == TICKS_TO_MICROSEC_OFFSET_TIMESTAMP)
    {
      return (offsetTimestampFirstLidarTick > 0);
    }
    else if (ticksToTimestampMode == TICKS_TO_LIDAR_TIMESTAMP)
    {
      return true; // no initialization required
    }
    else
    {
      return isInitialized; 
    }
  }

  void IsInitialized(bool val)
  { isInitialized = val; }

  uint64_t FirstTickTransmit() const
  { return firstTickTransmit; }

  void FirstTickTransmit(uint64_t val)
  { firstTickTransmit = val; }

  uint64_t FirstTickScan() const
  { return firstTickScan; }

  void FirstTickScan(uint64_t val)
  { firstTickScan = val; }

  double FirstTimeStamp() const
  { return firstTimeStamp; }

  void FirstTimeStamp(double val)
  { firstTimeStamp = val; }

  double InterpolationSlope() const
  { return interpolationSlope; }

  void InterpolationSlope(double val)
  { interpolationSlope = val; }

  double AllowedTimeDeviation() const
  { return allowedTimeDeviation; }

  void AllowedTimeDeviation(double val)
  { allowedTimeDeviation = val; }

  uint32_t ExtrapolationDivergenceCounter() const
  { return extrapolationDivergenceCounter; }

  void ExtrapolationDivergenceCounter(uint32_t val)
  { extrapolationDivergenceCounter = val; }

  bool updatePLL(uint32_t sec, uint32_t nanoSec, uint32_t curtickTransmit, uint32_t curtickScan);
  bool updatePLL(uint32_t sec, uint32_t nanoSec, uint64_t curtickTransmit, uint64_t curtickScan);

  int findDiffInFifo(double diff, double tol);

  static const int fifoSize = 7;
  size_t packets_dropped = 0;    // just for printing statusmessages when dropping packets
  size_t packets_received = 0;   // just for printing statusmessages when dropping packets
  double max_abs_delta_time = 0; // just for printing statusmessages when dropping packets

  void setTicksToTimestampMode(int val)
  {
    ticksToTimestampMode = (TICKS_TO_TIMESTAMP_MODE)val;
  }

private:
  int numberValInFifo;
  static const double MaxAllowedTimeDeviation;
  static const uint32_t MaxExtrapolationCounter;
  uint64_t tickFifoTransmit[fifoSize]; //  = { 0 }; // fifo of transmit timestamps in ticks
  uint64_t tickFifoScan[fifoSize]; //  = { 0 }; // fifo of scan timestamps in ticks
  double clockFifo[fifoSize];
  double lastValidTimeStamp = 0;
  // uint64_t lastValidTick; // = 0;
  bool isInitialized = false;
  double dTAvgFeedback = 0;
  double dClockDiffFeedBack = 0;
  double firstTimeStamp = 0;
  double allowedTimeDeviation = 0;
  uint64_t firstTickTransmit = 0;
  uint64_t firstTickScan = 0;
  uint64_t lastcurtick = 0;
  uint32_t mostRecentSec;
  uint32_t mostRecentNanoSec;
  double mostRecentTimeStamp;
  double interpolationSlope;

  enum TICKS_TO_TIMESTAMP_MODE
  {
    TICKS_TO_SYSTEM_TIMESTAMP = 0, // default: convert lidar ticks in microseconds to system timestamp by software-pll
    TICKS_TO_MICROSEC_OFFSET_TIMESTAMP = 1, // optional tick-mode: convert lidar ticks in microseconds to timestamp by 1.0e-6*(curtick-firstTick)+firstSystemTimestamp
    TICKS_TO_LIDAR_TIMESTAMP = 2 // optional tick-mode: convert lidar ticks in microseconds directly into a lidar timestamp by sec = tick/1000000, nsec = 1000*(tick%1000000)
    // Note: Using tick_to_timestamp_mode = 2, the timestamps in ROS message headers will be in lidar time, not in system time. Lidar and system time can be very different.
    // Using tick_to_timestamp_mode = 2 might cause unexpected results or error messages. We recommend using tick_to_timestamp_mode = 2 for special test cases only.
  };
  TICKS_TO_TIMESTAMP_MODE ticksToTimestampMode = TICKS_TO_SYSTEM_TIMESTAMP;
  uint32_t offsetTimestampFirstSystemSec = 0;
  uint32_t offsetTimestampFirstSystemMicroSec = 0;
  uint64_t offsetTimestampFirstLidarTick = 0;

  bool nearSameTimeStamp(double relTimeStamp1, double relTimeStamp2, double& delta_time_abs);

  bool updateInterpolationSlope();

  uint32_t extrapolationDivergenceCounter;

  SoftwarePLL()
  {
    AllowedTimeDeviation(SoftwarePLL::MaxAllowedTimeDeviation); // 1 ms
    numberValInFifo = 0;
    isInitialized = false;
  }

  // verhindert, dass ein Objekt von au�erhalb von N erzeugt wird.
  // protected, wenn man von der Klasse noch erben m�chte
  SoftwarePLL(const SoftwarePLL &); /* verhindert, dass eine weitere Instanz via
								   Kopier-Konstruktor erstellt werden kann */
  SoftwarePLL &operator=(const SoftwarePLL &); //Verhindert weitere Instanz durch Kopie
};


#endif