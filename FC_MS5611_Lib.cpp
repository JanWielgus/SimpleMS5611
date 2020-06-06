// Based on the Joop Brokking YMFC-32 Flight Controller code
// 
// 

#include <FC_MS5611_Lib.h>


// Ptr to the baro object
// Set up in the constructor
FC_MS5611_Lib* baroPtr;

// Three functions used in the TaskPlanner
void requestPressureStartTask();
void pressureAction();
void temperatureAction();




FC_MS5611_Lib::FC_MS5611_Lib(FC_TaskPlanner* taskPlannerPtr)
	: pressureFilter(20) // average 20 past measurements
{
	this->taskPlanner = taskPlannerPtr;

	// Set up the baro ptr used by the class friend funcitons
	baroPtr = this;
}


bool FC_MS5611_Lib::initialize(bool needToBeginWire_flag)
{
	if (needToBeginWire_flag)
		Wire.begin();
	
	
	// Check if the baro is responding
	Wire.beginTransmission(MS5611_Address);
	if (Wire.endTransmission() != 0)
	{
		// Cannot connect with the MS5611
		
		return false;
	}
	
	
	// Device setup
		//For calculating the pressure the 6 calibration values need to be polled from the MS5611.
		//These 2 byte values are stored in the memory location 0xA2 and up.
	for (int i=1; i<=6; i++)
	{
		Wire.beginTransmission(MS5611_Address);
		Wire.write(0xA0 + i*2);
		Wire.endTransmission();

		Wire.requestFrom(MS5611_Address, 2);
		C[i] = Wire.read() << 8 | Wire.read();
	}
	
	// Pre-calculate some values
	OFF_C2 = C[2] * pow(2, 16);
	SENS_C1 = C[1] * pow(2, 15);
	
	

	// Schedule first baro reading action
	requestPressureStartTask();
	
	

	// The MS5611 needs a few readings to stabilize
	// Read pressure for 400ms
	uint32_t readingEndTime = millis() + 400;
	while (millis() < readingEndTime)
	{
		// as fast as possible
		taskPlanner->runPlanner();
	}
	
	return true;
}


void FC_MS5611_Lib::setFastClock()
{
	Wire.setClock(400000L);
}


pressureType FC_MS5611_Lib::getPressure()
{
	return pressure;
}


pressureType FC_MS5611_Lib::getSmoothPressure()
{
	return smoothPressure;
}


void FC_MS5611_Lib::registerNewBaroReadingFunction(void (*functionPointer)())
{
	newBaroReadingFunctionPointer = functionPointer;
}




void FC_MS5611_Lib::requestPressureFromDevice()
{
	// Request pressure data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x48);
	Wire.endTransmission();
}


void FC_MS5611_Lib::getRawPressureFromDevice()
{
	// Get the pressure data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.requestFrom(MS5611_Address, 3);
	rawPressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
}


void FC_MS5611_Lib::requestTemperatureFromDevice()
{
	// Request temperature data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x58);
	Wire.endTransmission();
}


void FC_MS5611_Lib::getRawTemperatreFromDevice()
{
	// Get the temperature data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.requestFrom(MS5611_Address, 3);
	rawTemperature = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
}


void FC_MS5611_Lib::calculatePressureAndTemperatureFromRawData()
{
	// Calculate pressure as explained in the datasheet of the MS-5611.
	// This part is from Joop Brokking YMFC-AL code
	dT = C[5];
	dT <<= 8;
	dT *= -1;
	dT += rawTemperature;
	OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) * 0.0078125; // 0.0078125 = 1 / pow(2, 7)
	SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) * 0.00390625; // 0.00390625 = 1 / pow(2, 8)
	intPressure = ((rawPressure * SENS) * 0.0000004768371582 - OFF) * 0.000030517578125; // 0.0000004768371582 = 1 / pow(2, 21),   0.000030517578125 = 1 / pow(2, 15)
	
	
	// Make the average from 20 readings
	pressureFilter.addNewSample(intPressure);
	pressure = pressureFilter.getAverage();
	
	
	// Smooth the value
	if (abs(smoothPressure - pressure) > 1)
		smoothPressure = smoothPressure*0.72f + pressure*0.28f;
	else
		smoothPressure = smoothPressure*0.96f + pressure*0.04f;


	// Call function added by the user (if not null)
	if (newBaroReadingFunctionPointer != nullptr)
		newBaroReadingFunctionPointer();
}





void requestPressureStartTask()
{
	baroPtr->requestPressureFromDevice();
	
	// Schedule first pressure action
	baroPtr->taskPlanner->scheduleTaskMicroseconds(pressureAction, FC_MS5611_Lib::REQUEST_WAIT_TIME);
}

void pressureAction()
{
	// get raw data
	baroPtr->actionCounter++;
	baroPtr->getRawPressureFromDevice();
	
	// schedule next action
	if (baroPtr->actionCounter == 20)
	{
		baroPtr->requestTemperatureFromDevice();
		baroPtr->taskPlanner->scheduleTaskMicroseconds(temperatureAction, FC_MS5611_Lib::REQUEST_WAIT_TIME);
	}
	else
	{
		baroPtr->requestPressureFromDevice();
		baroPtr->taskPlanner->scheduleTaskMicroseconds(pressureAction, FC_MS5611_Lib::REQUEST_WAIT_TIME);
	}

	// calculate pressure and temperature
	baroPtr->calculatePressureAndTemperatureFromRawData();
}

void temperatureAction()
{
	// get raw data
	baroPtr->getRawTemperatreFromDevice();
	
	// schedule next action
	baroPtr->actionCounter = 1;
	baroPtr->requestPressureFromDevice();
	baroPtr->taskPlanner->scheduleTaskMicroseconds(pressureAction, FC_MS5611_Lib::REQUEST_WAIT_TIME);

	// calculate pressure and temperature
	baroPtr->calculatePressureAndTemperatureFromRawData();
}


