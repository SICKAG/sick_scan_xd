/*
====================================================================================================
File: softwarePLL.cpp
====================================================================================================
*/
#include "softwarePLL.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <iterator>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstdint>

#if defined __ROS_VERSION && __ROS_VERSION > 0
#include <sick_scan/sick_ros_wrapper.h>
#endif

const double   SoftwarePLL::MaxAllowedTimeDeviation = 0.1;
const uint32_t SoftwarePLL::MaxExtrapolationCounter = 20;

// CSV helper for optional test/demo
class CSVRow
{
public:
  std::string const& operator[](std::size_t index) const { return m_data[index]; }
  std::size_t size() const { return m_data.size(); }

  void readNextRow(std::istream& str)
  {
    std::string line;
    std::getline(str, line);
    std::stringstream lineStream(line);
    std::string cell;
    m_data.clear();
    while (std::getline(lineStream, cell, ';'))
      m_data.push_back(cell);
    if (!lineStream && cell.empty())
      m_data.push_back("");
  }

private:
  std::vector<std::string> m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
  data.readNextRow(str);
  return str;
}

/**
 * Convert a possibly-32-bit hardware tick into a monotonic 64-bit tick.
 * Separate unwrap state for transmit vs scan.
 */
#include <cstdint>
#include <cstdio>

uint64_t SoftwarePLL::unwrapTick32(uint64_t raw_tick, TickSource source)
{
  // If it's already beyond 32-bit range, assume it's a real 64-bit tick
  if (raw_tick > 0xFFFFFFFFULL)
    return raw_tick;

  const uint32_t t32 = static_cast<uint32_t>(raw_tick);

  bool* init_flag = nullptr;
  uint32_t* last_tick = nullptr;
  uint64_t* wrap_offset = nullptr;

  if (source == TICK_TRANSMIT)
  {
    init_flag = &m_unwrap_init_transmit;
    last_tick = &m_last_tick32_transmit;
    wrap_offset = &m_tick_offset_transmit;
  }
  else
  {
    init_flag = &m_unwrap_init_scan;
    last_tick = &m_last_tick32_scan;
    wrap_offset = &m_tick_offset_scan;
  }

  if (!(*init_flag))
  {
    *init_flag = true;
    *last_tick = t32;
    *wrap_offset = 0;
    return static_cast<uint64_t>(t32);
  }

  const uint32_t prev = *last_tick;

  // Treat as wrap ONLY if the backward jump is "large" (more than half the 32-bit range).
  // This tolerates small out-of-order arrivals without falsely adding 2^32.
  if (t32 < prev)
  {
    const uint32_t back = prev - t32;

    // Real wrap: prev near max and t32 near zero -> huge backward jump
    if (back > 0x80000000U)
    {
      *wrap_offset += 0x100000000ULL; // add 2^32
    }
    else
    {
      // Out-of-order / jitter / reset-like small backward step: do NOT wrap.
      // Optional debug:
      // printf("unwrapTick32: non-wrap backward step source=%d prev=%u t32=%u back=%u\n",
      //        (int)source, prev, t32, back);
    }
  }

  *last_tick = t32;
  return *wrap_offset + static_cast<uint64_t>(t32);
}

/**
 * Update tick fifo and clock fifo
 */
bool SoftwarePLL::pushIntoFifo(double curTimeStamp, uint64_t curtickTransmit, uint64_t curtickScan)
{
  for (int i = 0; i < fifoSize - 1; i++)
  {
    tickFifoTransmit[i] = tickFifoTransmit[i + 1];
    tickFifoScan[i] = tickFifoScan[i + 1];
    clockFifo[i] = clockFifo[i + 1];
  }
  tickFifoTransmit[fifoSize - 1] = curtickTransmit;
  tickFifoScan[fifoSize - 1] = curtickScan;
  clockFifo[fifoSize - 1] = curTimeStamp;

  if (numberValInFifo < fifoSize)
  {
    numberValInFifo++;
  }
  FirstTickTransmit(tickFifoTransmit[0]);
  FirstTickScan(tickFifoScan[0]);
  FirstTimeStamp(clockFifo[0]);

  return true;
}

/**
 * With internal unwrapped ticks, this is just a linear mapping.
 */
double SoftwarePLL::extraPolateRelativeTimeStamp(uint64_t cur_tick, uint64_t first_tick)
{
  int64_t tempTick = static_cast<int64_t>(cur_tick - first_tick);
  double timeDiff = tempTick * this->InterpolationSlope();
  return timeDiff;
}

int SoftwarePLL::findDiffInFifo(double diff, double tol)
{
  int numFnd = 0;
  double minAllowedDiff = (1.0 - tol) * diff;
  double maxAllowedDiff = (1.0 + tol) * diff;

  for (int i = 0; i < numberValInFifo - 1; i++)
  {
    double diffTime = this->clockFifo[i + 1] - clockFifo[i];
    if (diffTime >= minAllowedDiff && diffTime <= maxAllowedDiff)
      numFnd++;
  }
  return numFnd;
}

/*!
\brief Updates PLL internal state. Call with network send timestamps.
*/
bool SoftwarePLL::updatePLL(uint32_t sec, uint32_t nanoSec,
  uint64_t curtickTransmit, uint64_t curtickScan)
{
  // unwrap to internal monotonic 64-bit ticks
  uint64_t curtickTransmit64 = unwrapTick32(curtickTransmit, TICK_TRANSMIT);
  uint64_t curtickScan64 = unwrapTick32(curtickScan, TICK_SCAN);

  if (offsetTimestampFirstLidarTick == 0)
  {
    offsetTimestampFirstSystemSec = sec;
    offsetTimestampFirstSystemMicroSec = nanoSec / 1000;
    offsetTimestampFirstLidarTick = curtickTransmit64;
  }

  if (curtickTransmit64 != this->lastcurtick)
  {
    this->lastcurtick = curtickTransmit64;
    double start = sec + nanoSec * 1e-9;

    if (!IsInitialized())
    {
      pushIntoFifo(start, curtickTransmit64, curtickScan64);
      bool bCheck = this->updateInterpolationSlope();
      if (bCheck)
      {
        IsInitialized(true);
      }
    }

    if (!IsInitialized())
      return false;

    double relTimeStamp = extraPolateRelativeTimeStamp(curtickTransmit64, FirstTickTransmit());
    double cmpTimeStamp = start - this->FirstTimeStamp();

    bool   timeStampVerified = false;
    double delta_time_abs = 0.0;

    if (nearSameTimeStamp(relTimeStamp, cmpTimeStamp, delta_time_abs))
    {
      timeStampVerified = true;
      pushIntoFifo(start, curtickTransmit64, curtickScan64);
      updateInterpolationSlope();
      ExtrapolationDivergenceCounter(0);
    }

    if (!timeStampVerified)
    {
      uint32_t tmp = ExtrapolationDivergenceCounter();
      tmp++;
      ExtrapolationDivergenceCounter(tmp);
      if (ExtrapolationDivergenceCounter() >= SoftwarePLL::MaxExtrapolationCounter)
      {
        IsInitialized(false);
      }
    }
    return true;
  }

  return false; // update already done
}

bool SoftwarePLL::updatePLL(uint32_t sec, uint32_t nanoSec,
  uint32_t curtickTransmit, uint32_t curtickScan)
{
  return updatePLL(sec, nanoSec,
    static_cast<uint64_t>(curtickTransmit),
    static_cast<uint64_t>(curtickScan));
}

// Map lidar ticks -> timestamp
bool SoftwarePLL::getCorrectedTimeStamp(uint32_t& u32Sec,
  uint32_t& u32NanoSec,
  uint64_t u64Curtick)
{
  if (ticksToTimestampMode == TICKS_TO_LIDAR_TIMESTAMP)
  {
    u32Sec = static_cast<uint32_t>(u64Curtick / 1000000ULL);
    u32NanoSec = static_cast<uint32_t>(1000ULL * (u64Curtick % 1000000ULL));
    return true;
  }

  if (!IsInitialized())
    return false;

  double corrTime = 0.0;
  double relTimeStamp = 0.0;

  if (ticksToTimestampMode == TICKS_TO_MICROSEC_OFFSET_TIMESTAMP)
  {
    uint64_t curtick_unwrapped = unwrapTick32(u64Curtick, TICK_TRANSMIT);
    corrTime = 1.0e-6 * (static_cast<double>(curtick_unwrapped - offsetTimestampFirstLidarTick))
      + (offsetTimestampFirstSystemSec
        + 1.0e-6 * offsetTimestampFirstSystemMicroSec);
  }
  else // TICKS_TO_SYSTEM_TIMESTAMP, default PLL mode
  {
    uint64_t curtick_unwrapped = unwrapTick32(u64Curtick, TICK_SCAN);
    relTimeStamp = extraPolateRelativeTimeStamp(curtick_unwrapped, FirstTickScan());
    corrTime = relTimeStamp + this->FirstTimeStamp();
  }

  corrTime = std::max(0.0, corrTime);
  u32Sec = static_cast<uint32_t>(corrTime);
  double frac = corrTime - static_cast<double>(u32Sec);
  u32NanoSec = static_cast<uint32_t>(1e9 * frac);

#if defined __ROS_VERSION && __ROS_VERSION > 0
  bool   timestamp_ok = true;
  rosTime timestamp_now = rosTimeNow();

  uint64_t curtick_unwrapped_chk = unwrapTick32(u64Curtick, TICK_SCAN);
  int64_t  i64_curtick_minus_first =
    static_cast<int64_t>(curtick_unwrapped_chk - this->FirstTickScan());

  if (i64_curtick_minus_first < 0)
  {
    timestamp_ok = false;
    double slope = this->InterpolationSlope();
    double rel2 = static_cast<double>(i64_curtick_minus_first) * slope;
    ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp(u32Sec=" << u32Sec
      << ", u32NanoSec=" << u32NanoSec << ", u64Tick=" << u64Curtick
      << "): FirstTickScan=" << this->FirstTickScan()
      << ", Curtick-FirstTickScan=" << i64_curtick_minus_first
      << ", InterpolationSlope=" << slope
      << ", relTimeStamp1=" << relTimeStamp
      << ", relTimeStamp2=" << rel2
      << ", invalid lidar ticks, extrapolated timestamp not ok.");
  }

  if (timestamp_ok)
  {
    try
    {
      rosTime     timestamp_ros = rosTime(u32Sec, u32NanoSec);
      rosDuration time_delta = timestamp_ros - timestamp_now;
#if __ROS_VERSION == 1
      double sec_delta = time_delta.toSec();
#else
      double sec_delta = time_delta.seconds();
#endif
      if (std::abs(sec_delta) > 2.0)
      {
        timestamp_ok = false;
        ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp(u64Tick=" << u64Curtick
          << "): extrapolated timestamp = " << std::fixed << std::setprecision(9)
          << (u32Sec + 1.0e-9 * u32NanoSec)
          << " [sec], ros timestamp now = " << std::fixed << std::setprecision(9)
          << (sec(timestamp_now) + 1.0e-9 * nsec(timestamp_now))
          << " [sec], delta time = " << sec_delta
          << " [sec], extrapolated timestamp not ok.");
      }
    }
    catch (const std::exception& exc)
    {
      timestamp_ok = false;
      ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp(u32Sec=" << u32Sec
        << ", u32NanoSec=" << u32NanoSec << ", u64Tick=" << u64Curtick
        << "): exception \"" << exc.what()
        << "\", extrapolated timestamp not ok.");
    }
  }

  if (!timestamp_ok)
  {
    u32Sec = sec(timestamp_now);
    u32NanoSec = nsec(timestamp_now);
    ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp() failed, returning current system time "
      << std::fixed << std::setprecision(9)
      << (u32Sec + 1.0e-9 * u32NanoSec) << " [sec]");
    ROS_WARN_STREAM("## u64Curtick=" << u64Curtick
      << ", FirstTickScan()=" << this->FirstTickScan()
      << ", FirstTimeStamp()=" << this->FirstTimeStamp()
      << ", InterpolationSlope()=" << this->InterpolationSlope()
      << ", corrTime=" << corrTime);
  }
#endif

  return true;
}

bool SoftwarePLL::getCorrectedTimeStamp(uint32_t& sec,
  uint32_t& nanoSec,
  uint32_t curtick)
{
  return getCorrectedTimeStamp(sec, nanoSec,
    static_cast<uint64_t>(curtick));
}

// Inverse mapping: timestamp -> lidar ticks
bool SoftwarePLL::convSystemtimeToLidarTimestamp(uint32_t systemtime_sec,
  uint32_t systemtime_nanosec,
  uint64_t& tick)
{
  if (ticksToTimestampMode == TICKS_TO_LIDAR_TIMESTAMP)
  {
    tick = 1000000ULL * static_cast<uint64_t>(systemtime_sec)
      + static_cast<uint64_t>(systemtime_nanosec) / 1000ULL;
    return true;
  }

  if (!IsInitialized())
    return false;

  if (ticksToTimestampMode == TICKS_TO_MICROSEC_OFFSET_TIMESTAMP)
  {
    double relSystemTimestamp =
      (systemtime_sec + 1.0e-9 * systemtime_nanosec)
      - (offsetTimestampFirstSystemSec
        + 1.0e-6 * offsetTimestampFirstSystemMicroSec);
    double relTicks = 1.0e6 * relSystemTimestamp;
    tick = static_cast<uint64_t>(std::round(relTicks + offsetTimestampFirstLidarTick));
  }
  else // TICKS_TO_SYSTEM_TIMESTAMP (PLL mode)
  {
    double systemTimestamp =
      static_cast<double>(systemtime_sec)
      + 1.0e-9 * static_cast<double>(systemtime_nanosec);
    double relSystemTimestamp = systemTimestamp - this->FirstTimeStamp();
    double relTicks = relSystemTimestamp / this->InterpolationSlope();

    // use the same reference as FirstTickScan (unwrapped)
    uint64_t tick_offset = this->FirstTickScan();
    tick = static_cast<uint64_t>(std::round(relTicks + tick_offset));
  }
  return true;
}

bool SoftwarePLL::convSystemtimeToLidarTimestamp(uint32_t systemtime_sec,
  uint32_t systemtime_nanosec,
  uint32_t& tick)
{
  uint64_t lidar_ticks = 0;
  bool success = convSystemtimeToLidarTimestamp(systemtime_sec,
    systemtime_nanosec,
    lidar_ticks);
  tick = static_cast<uint32_t>(lidar_ticks);
  return success;
}

bool SoftwarePLL::nearSameTimeStamp(double relTimeStamp1,
  double relTimeStamp2,
  double& delta_time_abs)
{
  delta_time_abs = fabs(relTimeStamp1 - relTimeStamp2);
  return (delta_time_abs < AllowedTimeDeviation());
}

/**
 * Update interpolationSlope based on fifo (ticks already unwrapped).
 */
bool SoftwarePLL::updateInterpolationSlope()
{
  if (numberValInFifo < fifoSize)
    return false;

  std::vector<uint64_t> tickFifoUnwrap(fifoSize);
  std::vector<double>   clockFifoUnwrap(fifoSize);

  FirstTimeStamp(this->clockFifo[0]);
  FirstTickTransmit(this->tickFifoTransmit[0]);
  FirstTickScan(this->tickFifoScan[0]);

  clockFifoUnwrap[0] = 0.0;
  tickFifoUnwrap[0] = 0;

  for (int i = 1; i < fifoSize; i++)
  {
    tickFifoUnwrap[i] = tickFifoTransmit[i] - FirstTickTransmit();
    clockFifoUnwrap[i] = clockFifo[i] - FirstTimeStamp();
  }

  double sum_xy = 0.0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;

  for (int i = 0; i < fifoSize; i++)
  {
    sum_xy += static_cast<double>(tickFifoUnwrap[i]) * clockFifoUnwrap[i];
    sum_x += static_cast<double>(tickFifoUnwrap[i]);
    sum_y += clockFifoUnwrap[i];
    sum_xx += static_cast<double>(tickFifoUnwrap[i]) * static_cast<double>(tickFifoUnwrap[i]);
  }

  double denom = (fifoSize * sum_xx - sum_x * sum_x);
  if (denom == 0.0)
    return false;

  double m = (fifoSize * sum_xy - sum_x * sum_y) / denom;

  int matchCnt = 0;
  max_abs_delta_time = 0.0;

  for (int i = 0; i < fifoSize; i++)
  {
    double yesti = m * static_cast<double>(tickFifoUnwrap[i]);
    double abs_delta_time = 0.0;
    if (this->nearSameTimeStamp(yesti, clockFifoUnwrap[i], abs_delta_time))
      matchCnt++;
    max_abs_delta_time = std::max(max_abs_delta_time, abs_delta_time);
  }

  if (matchCnt == fifoSize)
  {
    InterpolationSlope(m);
    return true;
  }

  return false;
}

#if 0
bool SoftwarePLL::getDemoFileData(std::string fileName,
  std::vector<uint32_t>& tickVec,
  std::vector<uint32_t>& secVec,
  std::vector<uint32_t>& nanoSecVec)
{
  std::ifstream file(fileName);
  CSVRow row;
  tickVec.clear();
  secVec.clear();
  nanoSecVec.clear();
  int lineCnt = 0;
  while (file >> row)
  {
    if (lineCnt > 0)
    {
      uint32_t tickVal = static_cast<uint32_t>(std::stoi(row[0]));
      uint32_t secVal = static_cast<uint32_t>(std::stoi(row[1]));
      uint32_t nanoSecVal = static_cast<uint32_t>(std::stoi(row[2]));
      tickVec.push_back(tickVal);
      secVec.push_back(secVal);
      nanoSecVec.push_back(nanoSecVal);
    }
    lineCnt++;
  }
  return (lineCnt > 1);
}
#endif

void SoftwarePLL::testbed()
{
  std::cout << "Running testbed for SoftwarePLL" << std::endl;
  uint32_t curtick = 0;
  int cnt = 0;

  SoftwarePLL testPll;
  uint32_t sec = 9999;
  uint32_t nanoSec = 0;
  double tickPerSec = 1E6;
  uint32_t tickInc = 1000;
  int maxLoop = 20;

  std::vector<uint32_t> tickVec;
  std::vector<uint32_t> secVec;
  std::vector<uint32_t> nanoSecVec;

  bool testWithDataFile = false;
  if (testWithDataFile)
  {
    maxLoop = static_cast<int>(tickVec.size());
  }

  for (int i = 0; i < maxLoop; i++)
  {
    if (testWithDataFile && i < static_cast<int>(tickVec.size()))
    {
      curtick = tickVec[i];
      sec = secVec[i];
      nanoSec = nanoSecVec[i];
    }
    else
    {
      cnt++;
      curtick += tickInc;
      double deltaT = tickInc / tickPerSec;
      nanoSec += static_cast<uint32_t>(deltaT * 1E9);
      if (nanoSec >= 1000000000u)
      {
        nanoSec = 0;
        sec++;
      }
      if (cnt >= 8)
      {
        sec++;
      }
    }

    printf("Before correction: %3u.%09u\n", sec, nanoSec);
    uint32_t org_sec = sec;
    uint32_t org_nanoSec = nanoSec;

    bool bRet = testPll.updatePLL(sec, nanoSec, curtick, curtick);
    (void)bRet; // ignore here

    bool ts_ok = testPll.getCorrectedTimeStamp(sec, nanoSec, curtick);

    bool corrected = ((nanoSec != org_nanoSec) || (sec != org_sec));
    printf("After correction : %3u.%09u %s %s\n",
      sec, nanoSec,
      ts_ok ? "OK     " : "DISMISS",
      corrected ? "MODI." : "OK   ");
  }
}

void SoftwarePLL::testbed_32bit_overflow()
{
  std::cout << "Running testbed_32bit_overflow for SoftwarePLL" << std::endl;

  SoftwarePLL testPll;

  uint32_t base_sec = 1761140000u;
  uint32_t base_nsec = 0u;

  // Initialize with values close to 32-bit max to force wrap
  for (int i = 0; i < 10; i++)
  {
    uint32_t tick32 = 4294000000u + i * 10000u; // near 2^32
    uint32_t sec = base_sec;
    uint32_t nsec = base_nsec + i * 10000000u;
    testPll.updatePLL(sec, nsec, tick32, tick32);
  }

  if (!testPll.IsInitialized())
  {
    std::cerr << "PLL not initialized in overflow test" << std::endl;
    return;
  }

  // Simulate wrapped tick
  uint32_t sec_after = base_sec + 1;
  uint32_t nsec_after = 100000000u;
  uint32_t tick_after = 50000u; // after wrap

  uint32_t corr_sec = sec_after;
  uint32_t corr_nsec = nsec_after;
  bool ok = testPll.getCorrectedTimeStamp(corr_sec, corr_nsec, tick_after);

  std::cout << "Wrapped tick corrected: ok=" << ok
    << " ts=" << corr_sec << "." << std::setw(9) << std::setfill('0') << corr_nsec
    << std::endl;
}

void SoftwarePLL::setFifoSize(std::size_t newSize)
{
  if (newSize == 0)
    return; // defensive: ignore invalid size

  fifoSize = newSize;

  // Clear and resize FIFOs
  tickFifoTransmit.assign(fifoSize, 0);
  tickFifoScan.assign(fifoSize, 0);
  clockFifo.assign(fifoSize, 0.0);

  // Reset internal state
  numberValInFifo = 0;
  isInitialized = false;
}

void SoftwarePLL::dumpInternalStatus(const std::string& filename) const
{
  // Nothing to dump
  if (numberValInFifo <= 0)
    return;

  const int n = (numberValInFifo < fifoSize) ? numberValInFifo : static_cast<int>(fifoSize);

  // Offsets (first valid FIFO entry)
  const uint64_t sc0 = tickFifoScan[0];     // used for BOTH scan and transmit offsets
  const double   t00 = clockFifo[0];

  std::ofstream out(filename, std::ios::out | std::ios::trunc);
  if (!out.is_open())
    return;

  // CSV header (semicolon separated)
  out << "idx;"
    << "d_tick_scan;d_tick_transmit;d_clock;"
    << "tick_scan;tick_transmit;clock"
    << "\n";

  // For nice double formatting
  out << std::fixed << std::setprecision(9);

  for (int i = 0; i < n; ++i)
  {
    const uint64_t tx = tickFifoTransmit[i];
    const uint64_t sc = tickFifoScan[i];
    const double   tt = clockFifo[i];

    // Diffs to first SCAN tick
    const uint64_t d_sc = sc - sc0;
    const uint64_t d_tx = tx - sc0;
    const double   d_tt = tt - t00;

    out << i << ";"
      << d_sc << ";" << d_tx << ";" << d_tt << ";"
      << sc << ";" << tx << ";" << tt
      << "\n";
  }

  out.flush();
}


#ifdef softwarePLL_MAINTEST
int main(int argc, char** argv)
{
  std::cout << "Test for softwarePLL-Class" << std::endl;
  SoftwarePLL::testbed();
  SoftwarePLL::testbed_32bit_overflow();
  return 0;
}
#endif
