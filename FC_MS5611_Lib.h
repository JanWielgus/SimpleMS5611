// FC_MS5611_Lib.h
// Based on the Joop Brokking YMFC-32 Flight Controller code
//

#ifndef _FC_MS5611_LIB_h
#define _FC_MS5611_LIB_h

#include "arduino.h"

#include <Wire.h>
#include <FC_TaskPlanner.h>
#include <FC_AverageFilter.h>


class FC_MS5611_Lib
{
 public:
	FC_MS5611_Lib(FC_TaskPlanner* taskPlannerPtr);
	bool initialize(bool needToBeginWire_flag = true);
	void setFastClock();
	float getPressure(); // new pressure value is updated about 111 times per second
	float getSmoothPressure(); // same as getPressure but smoother
	void registerNewBaroReadingFunction(void (*functionPointer)()); // When baro get new reading this function will be called

	
 private:
	void requestPressureFromDevice();
	void getRawPressureFromDevice(); // need to request first!
	void requestTemperatureFromDevice();
	void getRawTemperatreFromDevice(); // need to request first!
	void calculatePressureAndTemperatureFromRawData(); // after requesting raw data


	// friend functions used in the TaskPlanner
	friend void requestPressureStartTask();
	friend void pressureAction();
	friend void temperatureAction();
	
	
	
	
 private:
	FC_TaskPlanner* taskPlanner; // task planner should have capability of storing at least 2 tasks at once
	FC_AverageFilter<int32_t, int32_t, double> pressureFilter; // initialized in the constructor
	
	static const uint8_t MS5611_Address = 0x77;
	static const uint8_t REQUEST_WAIT_TIME = 9; // time in ms between value request and ready to read from device
	
	
	// Device calibration values
	uint16_t C[7]; // 0 position is not used (7 to use positions from 1 to 6 as in the datasheet)
	int64_t OFF, OFF_C2, SENS, SENS_C1;
	int32_t dT;
	
	
	
	// readings
	uint32_t rawPressure;
	uint32_t rawTemperature;
	int32_t intPressure; // temp pressure value (before average pressure is in integer)
	float smoothPressure; // smoother pressure value (in mbar*100)
	float pressure; // pressure in mbar*100
	
	// this counter is used to get temperature every 20 readings
	uint8_t actionCounter = 0;

	void (*newBaroReadingFunctionPointer)() = nullptr;
};


#endif

