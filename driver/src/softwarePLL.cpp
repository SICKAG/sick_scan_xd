/*
====================================================================================================
File: softwarePLL.cpp
====================================================================================================
*/
#include "softwarePLL.h"
#include <algorithm>
#include <iostream>
// #include <chrono>
// #include <thread>
#include <math.h>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


#if defined __ROS_VERSION && __ROS_VERSION > 0 // Check extrapolated timestamps against ros time
#include <sick_scan/sick_ros_wrapper.h>
#endif

const double SoftwarePLL::MaxAllowedTimeDeviation = 0.1;
const uint32_t SoftwarePLL::MaxExtrapolationCounter = 20;

// Helper class for reading csv file with test data

class CSVRow
{
public:
  std::string const &operator[](std::size_t index) const
  {
    return m_data[index];
  }

  std::size_t size() const
  {
    return m_data.size();
  }

  void readNextRow(std::istream &str)
  {
    std::string line;
    std::getline(str, line);

    std::stringstream lineStream(line);
    std::string cell;

    m_data.clear();
    while (std::getline(lineStream, cell, ';'))
    {
      m_data.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
      // If there was a trailing comma then add an empty element.
      m_data.push_back("");
    }
  }

private:
  std::vector<std::string> m_data;
};

std::istream &operator>>(std::istream &str, CSVRow &data)
{
  data.readNextRow(str);
  return str;
}


bool SoftwarePLL::pushIntoFifo(double curTimeStamp, uint64_t curtickTransmit, uint64_t curtickScan)
// update tick fifo and update clock (timestamp) fifo
{
  for (int i = 0; i < fifoSize - 1; i++)
  {
    tickFifoTransmit[i] = tickFifoTransmit[i + 1];
    tickFifoScan[i] = tickFifoScan[i + 1];
    clockFifo[i] = clockFifo[i + 1];
  }
  tickFifoTransmit[fifoSize - 1] = curtickTransmit; // push most recent tick and timestamp into fifo
  tickFifoScan[fifoSize - 1] = curtickScan;
  clockFifo[fifoSize - 1] = curTimeStamp;

  if (numberValInFifo < fifoSize)
  {
    numberValInFifo++; // remember the number of valid number in fifo
  }
  FirstTickTransmit(tickFifoTransmit[0]);
  FirstTickScan(tickFifoScan[0]);
  FirstTimeStamp(clockFifo[0]);

  return (true);
}

double SoftwarePLL::extraPolateRelativeTimeStamp(uint64_t cur_tick, uint64_t first_tick)
{
  int64_t tempTick = ((cur_tick >= first_tick) ? (int64_t)(cur_tick - first_tick) : -((int64_t)(first_tick - cur_tick)));
  double timeDiff = tempTick * this->InterpolationSlope();
  return (timeDiff);

}

int SoftwarePLL::findDiffInFifo(double diff, double tol)
{
  int numFnd = 0;
  double minAllowedDiff = (1.0 - tol) * diff;
  double maxAllowedDiff = (1.0 + tol) * diff;

  for (int i = 0; i < numberValInFifo - 1; i++)
  {
    double diffTime = this->clockFifo[i + 1] - clockFifo[i];
    if ((diffTime >= minAllowedDiff) && (diffTime <= maxAllowedDiff))
    {
      numFnd++;
    }
  }

  return (numFnd);
}

/*!
\brief Updates PLL internale State should be called only with network send timestamps

\param sec: System Timetamp from received network packed
\param nsec: System Timestamp from received network packed
\param curtickTransmit transmit timestamp in microseconds since scanner start from SOPAS Datagram
\param curtickScan scan timestamp (generation timestamp by default, transmit timestamp if generation timestamp not supported) in microseconds since scanner start from SOPAS Datagram
\return PLL is in valid state (true)
*/
bool SoftwarePLL::updatePLL(uint32_t sec, uint32_t nanoSec, uint64_t curtickTransmit, uint64_t curtickScan)
{
  if (offsetTimestampFirstLidarTick == 0)
  {
    // Store first timestamp and ticks for optional TICKS_TO_MICROSEC_OFFSET_TIMESTAMP
    offsetTimestampFirstSystemSec = sec;
    offsetTimestampFirstSystemMicroSec = nanoSec / 1000;
    offsetTimestampFirstLidarTick = curtickTransmit;
  }

  if (curtickTransmit != this->lastcurtick)
  {
    this->lastcurtick = curtickTransmit;
    double start = sec + nanoSec * 1E-9;
    bool bRet = true;

    if (false == IsInitialized())
    {
      pushIntoFifo(start, curtickTransmit, curtickScan);
      bool bCheck = this->updateInterpolationSlope();
      if (bCheck)
      {
        IsInitialized(true);
      }
    }

    if (IsInitialized() == false)
    {
      return (false);
    }

    double relTimeStamp = extraPolateRelativeTimeStamp(curtickTransmit, FirstTickTransmit());
    double cmpTimeStamp = start - this->FirstTimeStamp();

    bool timeStampVerified = false;
    double delta_time_abs = 0;
    if (nearSameTimeStamp(relTimeStamp, cmpTimeStamp, delta_time_abs) == true)// if timestamp matches prediction update FIFO
    {
      timeStampVerified = true;
      pushIntoFifo(start, curtickTransmit, curtickScan);
      updateInterpolationSlope();
      ExtrapolationDivergenceCounter(0);
    }

    if (timeStampVerified == false)
    {
      // BEGIN HANDLING Extrapolation divergence
      uint32_t tmp = ExtrapolationDivergenceCounter();
      tmp++;
      ExtrapolationDivergenceCounter(tmp);
      if (ExtrapolationDivergenceCounter() >= SoftwarePLL::MaxExtrapolationCounter)
      {
        IsInitialized(false); // reset FIFO - maybe happened due to abrupt change of time base
      }
      // END HANDLING Extrapolation divergence
    }
    return (true);
  }
  else
  {
    return (false); // update already done
  }
}

bool SoftwarePLL::updatePLL(uint32_t sec, uint32_t nanoSec, uint32_t curtickTransmit, uint32_t curtickScan)
{
  return updatePLL(sec, nanoSec, (uint64_t)curtickTransmit, (uint64_t)curtickScan);
}

//TODO Kommentare
bool SoftwarePLL::getCorrectedTimeStamp(uint32_t &u32Sec, uint32_t &u32NanoSec, uint64_t u64Curtick)
{
  if (ticksToTimestampMode == TICKS_TO_LIDAR_TIMESTAMP) // optional tick-mode: convert lidar ticks in microseconds directly into a lidar timestamp by sec = tick/1000000, nsec = 1000 * (tick % 1000000)
  {
    u32Sec = (uint32_t)(u64Curtick / 1000000);
    u32NanoSec = (uint32_t)(1000 * (u64Curtick % 1000000));
    return true;
  }
  if (IsInitialized() == false)
  {
    return (false);
  }
  double corrTime = 0, relTimeStamp = 0;
  if (ticksToTimestampMode == TICKS_TO_MICROSEC_OFFSET_TIMESTAMP) // optional tick-mode: convert lidar ticks in microseconds to timestamp by 1.0e-6*(u64Curtick-firstTick)+firstSystemTimestamp
  {
    corrTime = 1.0e-6 * (u64Curtick - offsetTimestampFirstLidarTick) + (offsetTimestampFirstSystemSec + 1.0e-6 * offsetTimestampFirstSystemMicroSec);
  }
  else // default: convert lidar ticks in microseconds to system timestamp by software-pll
  {
    relTimeStamp = extraPolateRelativeTimeStamp(u64Curtick, FirstTickScan());
    corrTime = relTimeStamp + this->FirstTimeStamp();
  }
  corrTime = std::max<double>(0, corrTime);
  u32Sec = (uint32_t) corrTime;
  double frac = corrTime - u32Sec;
  u32NanoSec = (uint32_t) (1E9 * frac);
  // Check extrapolated timestamps against ros time
  // std::cout << "SoftwarePLL::getCorrectedTimeStamp(): timestamp_mode=" << (int)ticksToTimestampMode << ", curticks = " << u64Curtick << " [microsec], system time = " << std::fixed << std::setprecision(9) << (u32Sec + 1.0e-9 * u32NanoSec) << " [sec]" << std::endl;
#if defined __ROS_VERSION && __ROS_VERSION > 0
  bool timestamp_ok = true;
  rosTime timestamp_now = rosTimeNow();
  int64_t i64_curtick_minus_first = ((u64Curtick >= this->FirstTickScan()) ? ((int64_t)(u64Curtick - this->FirstTickScan())) : (-1 * (int64_t)(this->FirstTickScan() - u64Curtick)));
  if (i64_curtick_minus_first < 0)
  {
    timestamp_ok = false;
    ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp(u32Sec=" << u32Sec << ", u32NanoSec=" << u32NanoSec << ", u64Tick=" << u64Curtick << "): FirstTickScan=" << this->FirstTickScan() 
      << ", Curtick-FirstTickScan=" << i64_curtick_minus_first << ", InterpolationSlope=" << this->InterpolationSlope() 
      << ", (Curtick-FirstTickScan)*InterpolationSlope=" << ((u64Curtick-this->FirstTickScan())*this->InterpolationSlope()) << ", relTimeStamp1=" << relTimeStamp << ", relTimeStamp2=" << (i64_curtick_minus_first * this->InterpolationSlope())
      << ", invalid lidar ticks, extrapolated timestamp not ok.");
  }
  if (timestamp_ok)
  {
    try
    {
      rosTime timestamp_ros = rosTime(u32Sec, u32NanoSec);
      rosDuration time_delta = timestamp_ros - timestamp_now;
  #if __ROS_VERSION == 1
      double sec_delta = time_delta.toSec();
  #else
      double sec_delta = time_delta.seconds();
  #endif
      if (std::abs(sec_delta) > 2)
      {
        timestamp_ok = false;
        ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp(u64Tick=" << u64Curtick << "): extrapolated timestamp = " << std::fixed << std::setprecision(9) << (u32Sec + 1.0e-9 * u32NanoSec) 
          << " [sec], ros timestamp now = " << std::fixed << std::setprecision(9) << (sec(timestamp_now) + 1.0e-9 * nsec(timestamp_now)) << " [sec], delta time = " << sec_delta << " [sec], extrapolated timestamp not ok.");
      }
    }
    catch(const std::exception& exc)
    {
      timestamp_ok = false;
      ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp(u32Sec=" << u32Sec << ", u32NanoSec=" << u32NanoSec << ", u64Tick=" << u64Curtick << "): exception \"" << exc.what() << "\", extrapolated timestamp not ok.");
    }
  }
  if (!timestamp_ok)
  {
    u32Sec = sec(timestamp_now);
    u32NanoSec = nsec(timestamp_now);
    ROS_WARN_STREAM("## WARNING SoftwarePLL::getCorrectedTimeStamp() failed, returning current system time " << std::fixed << std::setprecision(9) << (u32Sec + 1.0e-9 * u32NanoSec) << " [sec]");
    ROS_WARN_STREAM("## u64Curtick=" << u64Curtick << ", FirstTickScan()=" << this->FirstTickScan() << ", Curtick-FirstTickScan=" << i64_curtick_minus_first << ", FirstTimeStamp()=" << this->FirstTimeStamp() << ", InterpolationSlope()=" << this->InterpolationSlope() << ", corrTime=" << corrTime);
  }
#endif  
  return (true);
}

bool SoftwarePLL::getCorrectedTimeStamp(uint32_t &sec, uint32_t &nanoSec, uint32_t curtick)
{
  return getCorrectedTimeStamp(sec, nanoSec, (uint64_t)curtick);
}

// converts a system timestamp to lidar ticks, computes the inverse to getCorrectedTimeStamp().
bool SoftwarePLL::convSystemtimeToLidarTimestamp(uint32_t systemtime_sec, uint32_t systemtime_nanosec, uint64_t& tick)
{
  if (ticksToTimestampMode == TICKS_TO_LIDAR_TIMESTAMP) // optional tick-mode: convert lidar ticks in microseconds directly into a lidar timestamp
  {
    tick = 1000000 * (uint64_t)systemtime_sec + (uint64_t)systemtime_nanosec / 1000; // tick in micro seconds
    return true;
  }
  if (IsInitialized() == false)
  {
    return (false);
  }
  if (ticksToTimestampMode == TICKS_TO_MICROSEC_OFFSET_TIMESTAMP) // optional tick-mode: convert lidar ticks in microseconds to timestamp by 1.0e-6*(curtick-firstTick)+firstSystemTimestamp
  {
    double relSystemTimestamp = (systemtime_sec + 1.0e-9 * systemtime_nanosec) - (offsetTimestampFirstSystemSec + 1.0e-6 * offsetTimestampFirstSystemMicroSec);
    double relTicks = 1.0e6 * relSystemTimestamp;
    tick = (uint64_t)std::round(relTicks + offsetTimestampFirstLidarTick);
  }
  else // default: convert lidar ticks in microseconds to system timestamp by software-pll
  {
    double systemTimestamp = (double)systemtime_sec + 1.0e-9 * (double)systemtime_nanosec; // systemTimestamp := corrTime in getCorrectedTimeStamp
    // getCorrectedTimeStamp(): corrTime = relTimeStamp + this->FirstTimeStamp()
    // => inverse: relSystemTimestamp = systemTimestamp - this->FirstTimeStamp()
    double relSystemTimestamp = systemTimestamp - this->FirstTimeStamp();
    // getCorrectedTimeStamp(): relSystemTimestamp = (tick - (uint32_t) (0xFFFFFFFF & FirstTickScan())) * this->InterpolationSlope() 
    //=> inverse: tick = (relSystemTimestamp / this->InterpolationSlope()) + (uint32_t) (0xFFFFFFFF & FirstTickScan())
    double relTicks = relSystemTimestamp / this->InterpolationSlope();
    uint32_t tick_offset = (uint32_t)(0xFFFFFFFF & FirstTickScan());
    tick = (uint64_t)std::round(relTicks + tick_offset);
  }
  return (true);
}

bool SoftwarePLL::convSystemtimeToLidarTimestamp(uint32_t systemtime_sec, uint32_t systemtime_nanosec, uint32_t& tick)
{
  uint64_t lidar_ticks = 0;
  bool success = convSystemtimeToLidarTimestamp(systemtime_sec, systemtime_nanosec, lidar_ticks);
  tick = (uint32_t)lidar_ticks;
  return success;
}

bool SoftwarePLL::nearSameTimeStamp(double relTimeStamp1, double relTimeStamp2, double& delta_time_abs)
{
  delta_time_abs = fabs(relTimeStamp1 - relTimeStamp2);
  if (delta_time_abs < AllowedTimeDeviation())
  {
    return (true);
  }
  else
  {
    return (false);
  }
}

bool SoftwarePLL::updateInterpolationSlope() // fifo already updated
{

  if (numberValInFifo < fifoSize)
  {
    return (false);
  }
  std::vector<uint64_t> tickFifoUnwrap;
  std::vector<double> clockFifoUnwrap;
  clockFifoUnwrap.resize(fifoSize);
  tickFifoUnwrap.resize(fifoSize);
  uint64_t tickOffset = 0;
  clockFifoUnwrap[0] = 0.00;
  tickFifoUnwrap[0] = 0;
  FirstTimeStamp(this->clockFifo[0]);
  FirstTickTransmit(this->tickFifoTransmit[0]);
  FirstTickScan(this->tickFifoScan[0]);

  uint64_t tickDivisor = 0x100000000;


  for (int i = 1;
       i < fifoSize; i++)  // typical 643 for 20ms -> round about 32150 --> near to 32768 standard clock in many watches
  {
    if (tickFifoTransmit[i] < tickFifoTransmit[i - 1]) // Overflow
    {
      tickOffset += tickDivisor;
    }
    tickFifoUnwrap[i] = tickOffset + tickFifoTransmit[i] - FirstTickTransmit();
    clockFifoUnwrap[i] = (this->clockFifo[i] - FirstTimeStamp());
  }

  double sum_xy = 0.0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  for (int i = 0; i < fifoSize; i++)
  {
    sum_xy += tickFifoUnwrap[i] * clockFifoUnwrap[i];
    sum_x += tickFifoUnwrap[i];
    sum_y += clockFifoUnwrap[i];
    sum_xx += tickFifoUnwrap[i] * tickFifoUnwrap[i];
  }

  // calculate slope of regression line, interception is 0 by construction
  double m = (fifoSize * sum_xy - sum_x * sum_y) / (fifoSize * sum_xx - sum_x * sum_x);

  int matchCnt = 0;
  max_abs_delta_time = 0;
  for (int i = 0; i < fifoSize; i++)
  {
    double yesti = m * tickFifoUnwrap[i];
    double abs_delta_time = 0;
    if (this->nearSameTimeStamp(yesti, clockFifoUnwrap[i], abs_delta_time))
    {
      matchCnt++;
    }
    max_abs_delta_time = std::max(max_abs_delta_time, abs_delta_time);
    // std::cout << "SoftwarePLL::updateInterpolationSlope(): matchCnt=" << matchCnt << "/" << fifoSize << ", yesti=" << yesti << ", clockFifoUnwrap=" << clockFifoUnwrap[i] << ", tickFifoUnwrap=" << tickFifoUnwrap[i] << std::endl;
  }

  bool retVal = false;
  if (matchCnt == fifoSize)
  {
    InterpolationSlope(m);
    retVal = true;
  }
  // else
  // {
  //   std::cerr << "SoftwarePLL::updateInterpolationSlope(): matchCnt=" << matchCnt << "/" << fifoSize << ", max_abs_delta_time=" << max_abs_delta_time << " sec." << std::endl;
  // }

  return (retVal);
}

#if 0
bool SoftwarePLL::getDemoFileData(std::string fileName, std::vector<uint32_t>& tickVec,std::vector<uint32_t>& secVec, std::vector<uint32_t>& nanoSecVec )
{
    std::ifstream file(fileName);

    CSVRow row;
    tickVec.clear();
    secVec.clear();
    nanoSecVec.clear();
    int lineCnt = 0;
    while (file >> row) {
      if (lineCnt > 0)
    {
        uint32_t tickVal = (uint32_t)std::stoi(row[0]);
      uint32_t secVal = (uint32_t)std::stoi(row[1]);
      uint32_t nanoSecVal = (uint32_t)std::stoi(row[2]);
      tickVec.push_back(tickVal);
      secVec.push_back(secVal);
      nanoSecVec.push_back(nanoSecVal);
    }
      lineCnt++;
    }
    if (lineCnt <= 1)
        return false;
    else
        return true;
}
#endif

//TODO update testbed
void SoftwarePLL::testbed()
{
  std::cout << "Running testbed for SofwarePLL" << std::endl;
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
  //bool bRet = false;

  bool testWithDataFile = true;
  if (testWithDataFile)
  {
    // commented for trusty bRet = testPll.getDemoFileData("/home/rosuser/dumpimu3.csv", tickVec, secVec, nanoSecVec);
    maxLoop = (int)tickVec.size();
  }

  for (int i = 0; i < maxLoop; i++)
  {
    if (testWithDataFile)
    {
      curtick = tickVec[i];
      sec = secVec[i];
      nanoSec = nanoSecVec[i];
    }
    else
    {
      cnt++;
      curtick += tickInc; // increment tick counter
      double deltaT = tickInc / tickPerSec;
      nanoSec += (int) (deltaT * 1E9);
      if (nanoSec >= 1E9)
      {
        nanoSec = 0;
        sec++;
      }
      if (cnt >= 8)
      {
        sec++;
      }
    }
    printf("Before correction: %3d.%09d\n", sec, nanoSec);
    uint32_t org_sec = sec;
    uint32_t org_nanoSec = nanoSec;

    bool bRet = testPll.getCorrectedTimeStamp(sec, nanoSec, curtick);

    bool corrected = false;
    if ((nanoSec != org_nanoSec) || (sec != org_sec))
    {
      corrected = true;
    }
    printf("After correction : %3d.%09d %s %s\n", sec, nanoSec, bRet ? "OK     " : "DISMISS",
           corrected ? "MODI." : "OK   ");
  }

  return;
}


#ifdef softwarePLL_MAINTEST
int main(int argc, char **argv)
{
  printf("Test for softwarePLL-Class\n");
  printf("\n");
  SoftwarePLL::testbed();
}
#endif



/* 
Example CMakeLists.txt to generate test-binary-file for testing this class
--- CUT ---
#
#
# softwarePLL
#
#
cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)
project( softwarePLL )
#
#
add_definitions(-D${PROJECT_NAME}_MAINTEST)

MESSAGE( STATUS "CMKAKE for " ${PROJECT_NAME} )

include_directories( inc)
file( GLOB LIB_SOURCES src/ *.cpp )

if(WIN32)
else()
set(CMAKE_CXX_STANDARD 11)
endif()

add_executable( ${PROJECT_NAME} ${LIB_SOURCES} inc/${PROJECT_NAME}.h)
target_link_libraries( ${PROJECT_NAME})
--- CUT ---

*/