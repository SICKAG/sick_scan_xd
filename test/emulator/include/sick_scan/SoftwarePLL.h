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
#include <stdint.h>

/**
@brief class SoftwarePLL implements synchronisation between ticks and timestamp.
       See https://github.com/michael1309/SoftwarePLL/blob/master/README.md for details.
*/
class SoftwarePLL
{
public:

  /**
  @brief Creates an instance of SoftwarePLL or returns an existing one, given its id.
  *
  @param id: identifier of the instance, f.e. "Sensor1", "TIM571".
  @param fifo_length: length of fifo (buffer size, i.e. number of ticks in fifo). Applied only for new instances of SoftwarePLL, existing instance will keep their buffer size.
  @return instance of a SoftwarePLL
  */
	static SoftwarePLL& Instance(const std::string & id = "", int fifo_length = 7);
	
	/**
	@brief Destructor of class SoftwarePLL
	*/
	~SoftwarePLL();

	/**
	@brief Computes the timestamp of a measurement from sensor ticks
	*
	@param[out] sec: Estimated system Timetamp (seconds)
	@param[out] nanoSec: Estimated system Timetamp (nanoseconds)
	@param[in] tick sensor ticks
	@return true if timestamp is computed, false otherwise (SoftwarePLL not initialized)
	*/
  bool GetCorrectedTimeStamp(uint32_t& sec, uint32_t& nanoSec, uint32_t tick);

	/**
	@brief Updates PLL internale State should be called only with network send timestamps
	*
	@param[in] sec: System Timetamp from received network packed
	@param[in] nanoSec: System Timestamp from received network packed
	@param[in] curtick sensor ticks
	@return PLL is in valid state (true)
	*/
  bool UpdatePLL(uint32_t sec, uint32_t nanoSec, uint32_t curtick);
	
	/**
  @brief Returns the initialization status, i.e. true, if SoftwarePLL is initialized, or false otherwise (inconsistent or not enough samples in the fifo buffer)
  *
  @return initialization status
  */
	bool IsInitialized() const { return IsInitialized_; }

protected:
	
  /**
  @brief Sets the initialization status, i.e. true, if SoftwarePLL is initialized, or false otherwise (inconsistent or not enough samples in the fifo buffer)
  *
  @param[in] val initialization status
  */
	void IsInitialized(bool val) { IsInitialized_ = val; }
  
  /**
  @brief Returns the first sensor tick in the fifo buffer
  *
  @return first sensor tick
  */
	uint64_t FirstTick() const { return FirstTick_; }
  
  /**
  @brief Sets the first sensor tick in the fifo buffer
  *
  @param[in] val first sensor tick
  */
	void FirstTick(uint64_t val) { FirstTick_ = val; }
  
  /**
  @brief Returns the first timestamp in the fifo buffer in seconds
  *
  @return first timestamp
  */
	double FirstTimeStamp() const { return FirstTimeStamp_; }
  
  /**
  @brief Sets the first timestamp in the fifo buffer in seconds
  *
  @param[in] val first timestamp
  */
	void FirstTimeStamp(double val) { FirstTimeStamp_ = val; }
  
  /**
  @brief Returns the interpolated slope (gradient of the regression line)
  *
  @return interpolated slope
  */
	double InterpolationSlope() const { return InterpolationSlope_; }
  
  /**
  @brief Sets the interpolated slope (gradient of the regression line)
  *
  @param[in] val interpolated slope
  */
	void InterpolationSlope(double val) { InterpolationSlope_ = val; }
  
  /**
  @brief Returns the max. allowed time difference between estimated and measured timestamp time in seconds
  *
  @return allowed time difference between estimated and measured timestamp
  */
	double AllowedTimeDeviation() const { return AllowedTimeDeviation_; }
  
  /**
  @brief Set the max. allowed time difference between estimated and measured receive timestamp in seconds
  *
  @param[in] val allowed time difference between estimated and measured timestamp
  */
	void AllowedTimeDeviation(double val) { AllowedTimeDeviation_ = val; }
  
  /**
  @brief Returns the counter of extrapolated divergences (number of times the estimated and measured receive timestamp differed more than AllowedTimeDeviation
  *
  @return counter of extrapolated divergences
  */
	uint32_t ExtrapolationDivergenceCounter() const { return ExtrapolationDivergenceCounter_; }
  
  /**
  @brief Sets the counter of extrapolated divergences (number of times the estimated and measured receive timestamp differed more than AllowedTimeDeviation
  *
  @param[in] val counter of extrapolated divergences
  */
	void ExtrapolationDivergenceCounter(uint32_t val) { ExtrapolationDivergenceCounter_ = val; }

	/**
	@brief Pushes measurement timestamp and sensor ticks to the fifo,
				 updates tick fifo and clock (timestamp) fifo
  *
	@param curTimeStamp: measurement timestamp in seconds (receive time)
	@param curtick: sensor ticks
	@return always true
	*/
	bool PushIntoFifo(double curTimeStamp, uint32_t curtick);// update tick fifo and update clock (timestamp) fifo;

	/**
	@brief Extrapolates and returns the measurement timestamp in seconds
				 relative to FirstTimeStamp.
				 The timestamp of a measurement in seconds can be estimated from sensor ticks
				 by ExtraPolateRelativeTimeStamp(tick) + FirstTimeStamp().
	*
	@param tick: sensor ticks
	@return measurement timestamp in seconds relative to FirstTimeStamp
	*/
	double ExtraPolateRelativeTimeStamp(uint32_t tick);

private:
	
	int NumberValInFifo_;
	const int FifoSize_;
	static const double MaxAllowedTimeDeviation_;
	static const uint32_t MaxExtrapolationCounter_;
	std::vector<uint32_t> TickFifo_;
	std::vector<double> ClockFifo_;
	bool IsInitialized_;
	double FirstTimeStamp_;
	double AllowedTimeDeviation_;
	uint64_t FirstTick_;
	uint32_t Lastcurtick_ = 0;
	double InterpolationSlope_;
	bool NearSameTimeStamp(double relTimeStamp1, double relTimeStamp2);
	bool UpdateInterpolationSlope();
	uint32_t ExtrapolationDivergenceCounter_;
	SoftwarePLL(int fifo_length = 7) : FifoSize_(fifo_length), TickFifo_(fifo_length,0), ClockFifo_(fifo_length,0), IsInitialized_(false), FirstTimeStamp_(0), FirstTick_(0), InterpolationSlope_(0)
	{
		AllowedTimeDeviation(SoftwarePLL::MaxAllowedTimeDeviation_); // 1 ms
		NumberValInFifo_ = 0;
	}
	// verhindert, dass ein Objekt von ausserhalb von N erzeugt wird.
	// protected, wenn man von der Klasse noch erben moechte
	SoftwarePLL(const SoftwarePLL&); /* verhindert, dass eine weitere Instanz via
								   Kopier-Konstruktor erstellt werden kann */
	SoftwarePLL & operator = (const SoftwarePLL &); //Verhindert weitere Instanz durch Kopie
	
	static std::map<std::string,SoftwarePLL*> _instances; // list of SoftwarePLL instances, mapped by id
};

#endif