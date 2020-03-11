/*
    Name:       FC_MS5611_example.ino
    Created:	02/09/2019 13:06:01
    Author:     Jan Wielgus
*/

#include "FC_MS5611_Lib.h"


//FC_MS5611_Lib baro; // Baro object is created inside the library

void showPressure(); // this function show pressure in 80Hz


void setup()
{
	Serial.begin(115200);
	Serial.println("Program has started.");
	
	delay(200);
	
	// Set I2C 400kHz clock
	baro.setFastClock();
	
	while (!baro.initialize())
	{
		// Some problems occured
		Serial.println("Baro cannot be initialized");
		delay(500);
	}
	
	// This function will be executed if new baro reading will be available
	baro.registerNewBaroReadingFunction(showPressure);
	
	Serial.println("Setup completed");
}


void loop()
{
	baro.runBarometer();
}



void showPressure()
{
	Serial.print("Pressure: ");
	Serial.print(baro.getPressure());
	Serial.print(" SmoothPres: ");
	Serial.print(baro.getSmoothPressure());
	Serial.println();
}

