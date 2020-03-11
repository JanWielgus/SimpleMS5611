// Based on the Joop Brokking YMFC-32 Flight Controller code
// 
// 

#include <FC_MS5611_Lib.h>


// Have to pre-create an object to use the TaskPlanner
// Otherwise it will complicate the program very much
FC_MS5611_Lib baro;


// Three functions used in the TaskPlanner
void requestPressureStartTask();
void pressureAction();
void temperatureAction();



FC_MS5611_Lib::FC_MS5611_Lib()
	: pressureFilter(20) // average 20 past measurements
{
	
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
	taskPlanner.scheduleTask(requestPressureStartTask, 8);
	
	
	
	// The MS5611 needs a few readings to stabilize
	// Read pressure for 400ms
	uint32_t readingEndTime = millis() + 400;
	while (millis() < readingEndTime)
	{
		// as fast as possible
		runBarometer();
	}
	
	return true;
}


void FC_MS5611_Lib::setFastClock()
{
	Wire.setClock(400000L);
}


float FC_MS5611_Lib::getPressure()
{
	return pressure;
}


float FC_MS5611_Lib::getSmoothPressure()
{
	return smoothPressure;
}


void FC_MS5611_Lib::runBarometer()
{
	taskPlanner.runPlanner();
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
	OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
	SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
	intPressure = ((rawPressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
	
	
	// Make the average from 20 readings
	pressureFilter.addNewSample(intPressure);
	pressure = pressureFilter.getAverage();
	
	
	// Smooth the value
	if (abs(lastSmoothPressure - pressure) > 1)
		smoothPressure = smoothPressure*0.72f + pressure*0.28f;
	else
		smoothPressure = smoothPressure*0.96f + pressure*0.04f;
	lastSmoothPressure = smoothPressure;

	// Call function added by the user (if not null)
	if (newBaroReadingFunctionPointer != nullptr)
		newBaroReadingFunctionPointer();
}





void requestPressureStartTask()
{
	baro.requestPressureFromDevice();
	
	// Schedule first pressure action
	baro.taskPlanner.scheduleTask(pressureAction, 9);
}

void pressureAction()
{
	baro.actionCounter++;
	baro.getRawPressureFromDevice();
	baro.calculatePressureAndTemperatureFromRawData();
	
	if (baro.actionCounter == 20)
	{
		baro.requestTemperatureFromDevice();
		baro.taskPlanner.scheduleTask(temperatureAction, 9);
	}
	else
	{
		baro.requestPressureFromDevice();
		baro.taskPlanner.scheduleTask(pressureAction, 9);
	}
}

void temperatureAction()
{
	baro.getRawTemperatreFromDevice();
	baro.calculatePressureAndTemperatureFromRawData();
	baro.requestPressureFromDevice();
	baro.actionCounter = 1;
	baro.taskPlanner.scheduleTask(pressureAction, 9);
}


