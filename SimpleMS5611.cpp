/**
 * @file SimpleMS5611.cpp
 * @author Jan Wielgus (jan.wielgus12@gmail.com)
 * @date 2021-02-26
 * 
 */

#include <SimpleMS5611.h>
#include <Wire.h>


SimpleMS5611::SimpleMS5611()
{
}


bool SimpleMS5611::initialize()
{
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
	
	return true;
}


float SimpleMS5611::readPressure()
{
	requestTemperatureFromDevice();
	delayMicroseconds(RequestWaitTime_us);
	getRawTemperatureFromDevice();
	
	requestPressureFromDevice();
	delayMicroseconds(RequestWaitTime_us);
	getRawPressureFromDevice();

	calculatePressureAndTemperatureFromRawData();

	return pressure;
}


float SimpleMS5611::getPressure()
{
	return pressure;
}


void SimpleMS5611::requestPressureFromDevice()
{
	// Request pressure data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x48);
	Wire.endTransmission();
}


void SimpleMS5611::getRawPressureFromDevice()
{
	// Get the pressure data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.requestFrom(MS5611_Address, 3);
	rawPressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
}


void SimpleMS5611::requestTemperatureFromDevice()
{
	// Request temperature data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x58);
	Wire.endTransmission();
}


void SimpleMS5611::getRawTemperatureFromDevice()
{
	// Get the temperature data
	Wire.beginTransmission(MS5611_Address);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.requestFrom(MS5611_Address, 3);
	rawTemperature = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
}


void SimpleMS5611::calculatePressureAndTemperatureFromRawData()
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
	// TODO: change name of the intPressure variable

	pressure = intPressure; // TODO: probably there is a need to divide intPressure by some power of 10
}

