/*
====================================================================================================
File: softwarePLL.cpp
====================================================================================================
*/
#include "softwarePLL.h"
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


bool SoftwarePLL::pushIntoFifo(double curTimeStamp, uint32_t curtick)
// update tick fifo and update clock (timestamp) fifo
{
  for (int i = 0; i < fifoSize - 1; i++)
  {
    tickFifo[i] = tickFifo[i + 1];
    clockFifo[i] = clockFifo[i + 1];
  }
  tickFifo[fifoSize - 1] = curtick; // push most recent tick and timestamp into fifo
  clockFifo[fifoSize - 1] = curTimeStamp;

  if (numberValInFifo < fifoSize)
  {
    numberValInFifo++; // remember the number of valid number in fifo
  }
  FirstTick(tickFifo[0]);
  FirstTimeStamp(clockFifo[0]);

  return (true);
}

double SoftwarePLL::extraPolateRelativeTimeStamp(uint32_t tick)
{
  int32_t tempTick = 0;
  tempTick = tick - (uint32_t) (0xFFFFFFFF & FirstTick());
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
\param curtick micro Seconds since scanner start from SOPAS Datagram
\return PLL is in valid state (true)
*/
bool SoftwarePLL::updatePLL(uint32_t sec, uint32_t nanoSec, uint32_t curtick)
{
  if (curtick != this->lastcurtick)
  {
    this->lastcurtick = curtick;
    double start = sec + nanoSec * 1E-9;
    bool bRet = true;

    if (false == IsInitialized())
    {
      pushIntoFifo(start, curtick);
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

    double relTimeStamp = extraPolateRelativeTimeStamp(curtick); // evtl. hier wg. Ueberlauf noch einmal pruefen
    double cmpTimeStamp = start - this->FirstTimeStamp();

    bool timeStampVerified = false;
    if (nearSameTimeStamp(relTimeStamp, cmpTimeStamp) == true)// if timestamp matches prediction update FIFO
    {
      timeStampVerified = true;
      pushIntoFifo(start, curtick);
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
    return (false);
    //this curtick has been updated allready
  }

}

//TODO Kommentare
bool SoftwarePLL::getCorrectedTimeStamp(uint32_t &sec, uint32_t &nanoSec, uint32_t curtick)
{
  if (IsInitialized() == false)
  {
    return (false);
  }

  double relTimeStamp = extraPolateRelativeTimeStamp(curtick); // evtl. hier wg. Ueberlauf noch einmal pruefen
  double corrTime = relTimeStamp + this->FirstTimeStamp();
  sec = (uint32_t) corrTime;
  double frac = corrTime - sec;
  nanoSec = (uint32_t) (1E9 * frac);
  return (true);
}

bool SoftwarePLL::nearSameTimeStamp(double relTimeStamp1, double relTimeStamp2)
{
  double dTAbs = fabs(relTimeStamp1 - relTimeStamp2);
  if (dTAbs < AllowedTimeDeviation())
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
  FirstTick(this->tickFifo[0]);

  uint64_t tickDivisor = 0x100000000;


  for (int i = 1;
       i < fifoSize; i++)  // typical 643 for 20ms -> round about 32150 --> near to 32768 standard clock in many watches
  {
    if (tickFifo[i] < tickFifo[i - 1]) // Overflow
    {
      tickOffset += tickDivisor;
    }
    tickFifoUnwrap[i] = tickOffset + tickFifo[i] - FirstTick();
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
  for (int i = 0; i < fifoSize; i++)
  {
    double yesti = m * tickFifoUnwrap[i];
    if (this->nearSameTimeStamp(yesti, clockFifoUnwrap[i]))
    {
      matchCnt++;
    }
  }

  bool retVal = false;
  if (matchCnt == fifoSize)
  {
    InterpolationSlope(m);
    retVal = true;
  }

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
  bool bRet = false;

  bool testWithDataFile = true;
  if (testWithDataFile)
  {
    // commented for trusty bRet = testPll.getDemoFileData("/home/rosuser/dumpimu3.csv", tickVec, secVec, nanoSecVec);
    maxLoop = tickVec.size();
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