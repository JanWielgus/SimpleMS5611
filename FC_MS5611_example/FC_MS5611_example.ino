/*
    Name:       FC_MS5611_example.ino
    Created:	02/09/2019 13:06:01
    Author:     Jan Wielgus
*/

#include "FC_MS5611_Lib.h"
#include "FC_Tasker.h"


//FC_MS5611_Lib baro; // Baro object is created inside the library
FC_SimpleTasker tasker;

void showPressure(); // this function show pressure in 80Hz
void blink(); // control diode blinking


void setup()
{
	Serial.begin(115200);
	Serial.println("Program has started.");
	
	delay(200);
	pinMode(LED_BUILTIN, OUTPUT);
	
	// Add tasker function to show pressure
	tasker.addFunction(showPressure, 9090, 10); // 110Hz
	tasker.addFunction(blink, 100000, 10); // 10Hz (control blinking)
	
	
	// Set I2C 400kHz clock
	baro.setFastClock();
	
	while (!baro.initialize())
	{
		// Some problems occured
		Serial.println("Baro cannot be initialized");
		delay(500);
	}
	
	
	Serial.println("Setup completed");
}


void loop()
{
	baro.runBarometer();
	tasker.runTasker();
}



void showPressure()
{
	Serial.print("Pressure: ");
	Serial.print(baro.getPressure());
	Serial.print(" SmoothPres: ");
	Serial.print(baro.getSmoothPressure());
	Serial.println();
}

void blink()
{
	static bool ledState = false;
	digitalWrite(LED_BUILTIN, ledState);
	ledState = !ledState;
}
