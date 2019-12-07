# PZEM017 DC communication module
* Arduino Library
* PlatformIO as IDE
* Used STM32F103C8 (Blue Pill)


## Features:
* Library can read: Voltage,Current and Energy
* Write: high and low voltage alarm, Modbus-Adress, Shunt Range
* Reset Energy
* 

## Moduls:
* Uart to RS485 Bus, 3.3V version
* 

### Example
```c++
#include <Arduino.h>

#include "PZEM017.h"

/* Hardware Serial3 is only available on certain boards.
 * For example the Arduino MEGA 2560
*/
PZEM017 pzem(&Serial3);

void setup() {
  Serial.begin(115200);
  
}

void loop() {

    //pzem.setHIVoltageAlarm(18);
    //pzem.setLOWVoltageAlarm(10);
    pzem.getSlaveParameters();


    float voltage = pzem.voltage();
    if(!isnan(voltage)){
        Serial.print("Voltage: "); 
        Serial.print(voltage); 
        Serial.println("V");
    } else {
        Serial.println("Error reading voltage");
    }

    float current = pzem.current();
    if(!isnan(current)){
        Serial.print("Current: "); 
        Serial.print(current); 
        Serial.println("A");
    } else {
        Serial.println("Error reading current");
    }

    float power = pzem.power();
    if(!isnan(power)){
        Serial.print("Power: "); 
        Serial.print(power); 
        Serial.println("W");
    } else {
        Serial.println("Error reading power");
    }

    float energy = pzem.energy();
    if(!isnan(energy)){
        Serial.print("Energy: "); 
        Serial.print(energy,3); 
        Serial.println("Wh");
    } else {
        Serial.println("Error reading energy");
    }
    if(pzem.VoltHighAlarm())
    {
        Serial.println("OverVoltage");
    }
    if(pzem.VoltLowAlarm())
    {
        Serial.println("UnderVoltage");
    }

    Serial.println();
    delay(2000);
}
```

***
Thank you to [@olehs](https://github.com/olehs) for inspiring this library.
