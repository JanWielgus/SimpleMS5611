/*
    Name:       FC_MS5611_example.ino
    Created:	02/09/2019 13:06:01
    Author:     Jan Wielgus
*/

#include <Wire.h>
#include <FC_TaskPlanner.h>
#include <FC_AverageFilter.h> // used inside baro library
#include "FC_MS5611_Lib.h"


// task planner used for baro should have capability of at least 2 tasks at once
// Can be higher when also used fo other purposes
FC_TaskPlanner baroTaskPlanner(3);

// Create barometer instance and pass the task planner
// There CANNOT be more than one instance of the barometer class
// Singleton design pattern is not used, because this make drone code inconsistent
FC_MS5611_Lib baro(&baroTaskPlanner);

// this function show pressure in 80Hz (I'm not sure, maybe 110Hz)
void showPressure();


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
	
	// This function will be executed if new baro reading is available
	baro.registerNewBaroReadingFunction(showPressure);
	
	Serial.println("Setup completed");
}


void loop()
{
	// Remember to call the task planner as fast as possible
	// This is used to take measurements just in time
	// This should be the only thing inside the loop() (except for tasker run() method)
	baroTaskPlanner.runPlanner();
}



void showPressure()
{
	Serial.print("Pressure:");
	Serial.print(baro.getPressure());
	Serial.print(" SmoothPres:");
	Serial.print(baro.getSmoothPressure());
	Serial.println();
}

