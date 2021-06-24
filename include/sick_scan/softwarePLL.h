#ifndef SOFTWARE_PLL_H
#define SOFTWARE_PLL_H

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

#include <string>
#include <iostream>
#include <fstream>      // std::ifstream
#include <cstring>
#include <sstream>      // std::stringstream
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <ctime>
#include <boost/cstdint.hpp>
// #include <chrono>

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

  bool getDemoFileData(std::string fileName, std::vector<uint32_t> &tickVec, std::vector<uint32_t> &secVec,
                       std::vector<uint32_t> &nanoSecVec);

  static void testbed();

  bool IsInitialized() const
  { return isInitialized; }

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
  int packeds_droped = 0;//just for printing statusmessages when dropping packets

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

  bool nearSameTimeStamp(double relTimeStamp1, double relTimeStamp2);

  bool updateInterpolationSlope();

  uint32_t extrapolationDivergenceCounter;

  SoftwarePLL()
  {
    AllowedTimeDeviation(SoftwarePLL::MaxAllowedTimeDeviation); // 1 ms
    numberValInFifo = 0;
  }

  // verhindert, dass ein Objekt von au�erhalb von N erzeugt wird.
  // protected, wenn man von der Klasse noch erben m�chte
  SoftwarePLL(const SoftwarePLL &); /* verhindert, dass eine weitere Instanz via
								   Kopier-Konstruktor erstellt werden kann */
  SoftwarePLL &operator=(const SoftwarePLL &); //Verhindert weitere Instanz durch Kopie
};


#endif