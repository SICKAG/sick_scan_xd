#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef SOFTWARE_PLL_H
#define SOFTWARE_PLL_H

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <ctime>
#include <cstdint>

class SoftwarePLL
{
public:
  static SoftwarePLL& instance()
  {
    static SoftwarePLL _instance;
    return _instance;
  }

  ~SoftwarePLL() {}

  // runtime fifo size setter
  void setFifoSize(std::size_t newSize);

  // read size of fio
  std::size_t getFifoSize() const { return fifoSize; }

  // Update tick fifo and clock fifo
  bool pushIntoFifo(double curTimeStamp, uint64_t curtickTransmit, uint64_t curtickScan);

  // Extrapolate relative timestamp from ticks using current slope
  double extraPolateRelativeTimeStamp(uint64_t cur_tick, uint64_t first_tick);

  // Map lidar ticks to timestamps
  bool getCorrectedTimeStamp(uint32_t& sec, uint32_t& nanoSec, uint32_t tick);
  bool getCorrectedTimeStamp(uint32_t& sec, uint32_t& nanoSec, uint64_t tick);

  // Inverse mapping: timestamps -> lidar ticks
  bool convSystemtimeToLidarTimestamp(uint32_t systemtime_sec, uint32_t systemtime_nanosec, uint32_t& tick);
  bool convSystemtimeToLidarTimestamp(uint32_t systemtime_sec, uint32_t systemtime_nanosec, uint64_t& tick);

  bool getDemoFileData(std::string fileName, std::vector<uint32_t>& tickVec,
    std::vector<uint32_t>& secVec, std::vector<uint32_t>& nanoSecVec);


  void dumpInternalStatus(const std::string& filename) const; // dump internal status for debugging to a csv file

  static void testbed();
  static void testbed_32bit_overflow();

  static constexpr int FIFO_SIZE_SEGMENT_MODE = 101;
  static constexpr int FIFO_SIZE_STANDARD_MODE = 7;

  bool IsInitialized() const
  {
    if (ticksToTimestampMode == TICKS_TO_MICROSEC_OFFSET_TIMESTAMP)
      return (offsetTimestampFirstLidarTick > 0);
    else if (ticksToTimestampMode == TICKS_TO_LIDAR_TIMESTAMP)
      return true; // no initialization required
    else
      return isInitialized;
  }

  void IsInitialized(bool val) { isInitialized = val; }

  uint64_t FirstTickTransmit() const { return firstTickTransmit; }
  void FirstTickTransmit(uint64_t val) { firstTickTransmit = val; }

  uint64_t FirstTickScan() const { return firstTickScan; }
  void FirstTickScan(uint64_t val) { firstTickScan = val; }

  double FirstTimeStamp() const { return firstTimeStamp; }
  void FirstTimeStamp(double val) { firstTimeStamp = val; }

  double InterpolationSlope() const { return interpolationSlope; }
  void InterpolationSlope(double val) { interpolationSlope = val; }

  double AllowedTimeDeviation() const { return allowedTimeDeviation; }
  void AllowedTimeDeviation(double val) { allowedTimeDeviation = val; }

  uint32_t ExtrapolationDivergenceCounter() const { return extrapolationDivergenceCounter; }
  void ExtrapolationDivergenceCounter(uint32_t val) { extrapolationDivergenceCounter = val; }

  bool updatePLL(uint32_t sec, uint32_t nanoSec, uint32_t curtickTransmit, uint32_t curtickScan);
  bool updatePLL(uint32_t sec, uint32_t nanoSec, uint64_t curtickTransmit, uint64_t curtickScan);

  int findDiffInFifo(double diff, double tol);

  size_t packets_dropped = 0;
  size_t packets_received = 0;
  double max_abs_delta_time = 0;

  void setTicksToTimestampMode(int val)
  {
    ticksToTimestampMode = (TICKS_TO_TIMESTAMP_MODE)val;
  }

private:
  // Identify which hardware tick stream we unwrap
  enum TickSource
  {
    TICK_TRANSMIT,
    TICK_SCAN
  };

  std::size_t fifoSize = 0;

  std::vector<uint64_t> tickFifoTransmit;
  std::vector<uint64_t> tickFifoScan;
  std::vector<double>   clockFifo;

  bool isInitialized = false;
  int  numberValInFifo = 0;

  // Per-source 32-bit unwrap state
  bool     m_unwrap_init_transmit = false;
  bool     m_unwrap_init_scan = false;
  uint32_t m_last_tick32_transmit = 0;
  uint32_t m_last_tick32_scan = 0;
  uint64_t m_tick_offset_transmit = 0;
  uint64_t m_tick_offset_scan = 0;

  // Convert possibly-32-bit tick into internal monotonic 64-bit tick
  uint64_t unwrapTick32(uint64_t raw_tick, TickSource source);

  static const double MaxAllowedTimeDeviation;
  static const uint32_t MaxExtrapolationCounter;

  double lastValidTimeStamp = 0.0;
  double dTAvgFeedback = 0.0;
  double dClockDiffFeedBack = 0.0;
  double firstTimeStamp = 0.0;
  double allowedTimeDeviation = 0.0;
  uint64_t firstTickTransmit = 0;
  uint64_t firstTickScan = 0;
  uint64_t lastcurtick = 0;
  uint32_t mostRecentSec = 0;
  uint32_t mostRecentNanoSec = 0;
  double   mostRecentTimeStamp = 0.0;
  double   interpolationSlope = 0.0;

  enum TICKS_TO_TIMESTAMP_MODE
  {
    TICKS_TO_SYSTEM_TIMESTAMP = 0,        // default: PLL to system time
    TICKS_TO_MICROSEC_OFFSET_TIMESTAMP = 1, // 1e-6*(tick-first) + firstSystemTs
    TICKS_TO_LIDAR_TIMESTAMP = 2          // ticks -> lidar time directly
  };

  TICKS_TO_TIMESTAMP_MODE ticksToTimestampMode = TICKS_TO_SYSTEM_TIMESTAMP;

  uint32_t offsetTimestampFirstSystemSec = 0;
  uint32_t offsetTimestampFirstSystemMicroSec = 0;
  uint64_t offsetTimestampFirstLidarTick = 0;

  bool nearSameTimeStamp(double relTimeStamp1, double relTimeStamp2, double& delta_time_abs);
  bool updateInterpolationSlope();

  uint32_t extrapolationDivergenceCounter = 0;

  SoftwarePLL()
  {
    setFifoSize(FIFO_SIZE_STANDARD_MODE);
    AllowedTimeDeviation(SoftwarePLL::MaxAllowedTimeDeviation);
    numberValInFifo = 0;
    isInitialized = false;
  }

  SoftwarePLL(const SoftwarePLL&);
  SoftwarePLL& operator=(const SoftwarePLL&);
};

#endif
