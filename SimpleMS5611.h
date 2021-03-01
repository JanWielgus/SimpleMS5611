/**
 * @file SimpleMS5611.h
 * @author Jan Wielgus (jan.wielgus12@gmail.com)
 * @brief MS5611 barometer library based on the Joop Brokking YMFC-32 Flight Controller code.
 * @date 2021-02-25
 * 
 */

#ifndef SIMPLEMS5611_H
#define SIMPLEMS5611_H

#ifdef ARDUINO
    #include <Arduino.h>
#endif

#include <ITasker.h>
#include <AverageFilter.h>


class SimpleMS5611
{	
	// Device calibration values (like in Joop Brokking code, names from the datasheet)
	uint16_t C[7]; // 0 position is not used (7 to use positions from 1 to 6 as in the datasheet)
	int64_t OFF, OFF_C2, SENS, SENS_C1;
	int32_t dT;


protected:
	static const uint8_t MS5611_Address = 0x77;
	static const uint32_t RequestWaitTime_us = 8300; // time between value request and ready to read from device (check in datasheet)
	
	// readings
	uint32_t rawPressure = 0;
	uint32_t rawTemperature = 0;

	int32_t intPressure;
	float pressure; // pressure in mbar*100 (WHY??) // TODO: add unit in var name (mbar)


public:
	SimpleMS5611();

	/**
	 * @brief Initialize the MS5611 baro.
	 * Need to be called before the first use.
	 * @return false if something went wrong,
	 * true otherwise.
	 */
	virtual bool initialize();

	/**
	 * @brief Requests pressure and temperature from the MS5611
	 * and correct pressure value with temperature.
	 * @return New pressure reading.
	 */
	float readPressure();

	/**
	 * @brief Getter of the last pressure value
	 * read from the device.
	 * @return Last pressure value.
	 */
	float getLastPressure();

	/**
	 * @brief Getter of the last temperature value
	 * read from the device.
	 * @return Last temperature value.
	 */
	//float getLastTemperature();

	
protected:
	void requestPressureFromDevice();
	void getRawPressureFromDevice(); // need to request first!
	void requestTemperatureFromDevice();
	void getRawTemperatureFromDevice(); // need to request first!


	void calculatePressureAndTemperatureFromRawData(); // after requesting raw data
};


#endif
