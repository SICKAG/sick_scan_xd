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

  bool pushIntoFifo(double curTimeStamp, uint32_t curtick);// update tick fifo and update clock (timestamp) fifo;
  double extraPolateRelativeTimeStamp(uint32_t tick);

  bool getCorrectedTimeStamp(uint32_t &sec, uint32_t &nanoSec, uint32_t tick);

  bool convSystemtimeToLidarTimestamp(uint32_t systemtime_sec, uint32_t systemtime_nanosec, uint32_t& tick);

  bool getDemoFileData(std::string fileName, std::vector<uint32_t> &tickVec, std::vector<uint32_t> &secVec,
                       std::vector<uint32_t> &nanoSecVec);

  static void testbed();

  bool IsInitialized() const
  { 
    if (ticksToTimestampMode == TICKS_TO_MICROSEC_OFFSET_TIMESTAMP)
    {
      return (offsetTimestampFirstLidarTick > 0);
    }
    return isInitialized; 
  }

  void IsInitialized(bool val)
  { isInitialized = val; }

  uint64_t FirstTick() const
  { return firstTick; }

  void FirstTick(uint64_t val)
  { firstTick = val; }

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

  bool updatePLL(uint32_t sec, uint32_t nanoSec, uint32_t curtick);

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
  uint32_t tickFifo[fifoSize]; //  = { 0 };
  double clockFifo[fifoSize];
  double lastValidTimeStamp;
  uint32_t lastValidTick; // = 0;
  bool isInitialized; // = false;
  double dTAvgFeedback; // = 0.0;
  double dClockDiffFeedBack; //  = 0.0;
  double firstTimeStamp;
  double allowedTimeDeviation;
  uint64_t firstTick;
  uint32_t lastcurtick = 0;
  uint32_t mostRecentSec;
  uint32_t mostRecentNanoSec;
  double mostRecentTimeStamp;
  double interpolationSlope;

  enum TICKS_TO_TIMESTAMP_MODE
  {
    TICKS_TO_SYSTEM_TIMESTAMP = 0, // default: convert lidar ticks in microseconds to system timestamp by software-pll
    TICKS_TO_MICROSEC_OFFSET_TIMESTAMP = 1 // optional tick-mode: convert lidar ticks in microseconds to timestamp by 1.0e-6*(curtick-firstTick)+firstSystemTimestamp
  };
  TICKS_TO_TIMESTAMP_MODE ticksToTimestampMode = TICKS_TO_SYSTEM_TIMESTAMP;
  uint32_t offsetTimestampFirstSystemSec = 0;
  uint32_t offsetTimestampFirstSystemMicroSec = 0;
  uint32_t offsetTimestampFirstLidarTick = 0;

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