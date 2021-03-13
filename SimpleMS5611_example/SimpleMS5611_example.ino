/**
 * @file SimpleMS5611_example.ino
 * @author Jan Wielgus (jan.wielgus12@gmail.com)
 * @brief Example arduino code for SimpleMS5611 library.
 * @date 2021-03-13
 * 
 */

#include <Wire.h>
#include <SimpleMS5611.h>


SimpleMS5611 baro;


void setup()
{
    Serial.begin(115200);
    Serial.println("Program has started");

    delay(200);

    Wire.begin();

    while (!baro.initialize())
    {
        // Some problems occured
		Serial.println("Problems with baro initialization. Retrying...");
		delay(500);
    }

    Wire.setClock(400000L);

    Serial.println("Baro initialized successfully.");
}


void loop()
{
    float pressure = baro.readPressure();
    // sth = baro.getPressure(); // return last read pressure

    Serial.print("Pressure = ");
    Serial.println(pressure);
}
