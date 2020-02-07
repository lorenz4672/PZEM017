#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PZEM017.h"

/* Hardware Serial3 is only available on certain boards.
 * For example the Arduino MEGA 2560
*/

PZEM017 pzem1(5,4,0x02,9500);
PZEM017 pzem2(5,4,0x01,9400);

void setup() {
  Serial.begin(115200);
  //pzem.setAddress(0x02);
  // pzem2.setCurrentShunt(1);
  // pzem1.setCurrentShunt(1);
  
}

void loop() {

    //pzem.setHIVoltageAlarm(18);
    //pzem.setLOWVoltageAlarm(10);
    //pzem1.getSlaveParameters();
    //delay(1000);
        //pzem2.getSlaveParameters();
        

    float voltage = pzem1.voltage();
    if(!isnan(voltage)){
        Serial.print("Voltage: "); 
        Serial.print(voltage); 
        Serial.println("V");
    } else {
        Serial.println("Error reading voltage");
    }

    float current = pzem1.current();
    if(!isnan(current)){
        Serial.print("Current: "); 
        Serial.print(current); 
        Serial.println("A");
    } else {
        Serial.println("Error reading current");
    }

    float power = pzem1.power();
    if(!isnan(power)){
        Serial.print("Power: "); 
        Serial.print(power); 
        Serial.println("W");
    } else {
        Serial.println("Error reading power");
    }

    float energy = pzem1.energy();
    if(!isnan(energy)){
        Serial.print("Energy: "); 
        Serial.print(energy,3); 
        Serial.println("Wh");
    } else {
        Serial.println("Error reading energy");
    }
    if(pzem1.VoltHighAlarm())
    {
        Serial.println("OverVoltage");
    }
    if(pzem1.VoltLowAlarm())
    {
        Serial.println("UnderVoltage");
    }
//---------
Serial.println("---------------------");
delay(1000);
    float voltage2 = pzem2.voltage();
    if(!isnan(voltage2)){
        Serial.print("Voltage: "); 
        Serial.print(voltage2); 
        Serial.println("V");
    } else {
        Serial.println("Error reading voltage");
    }

    float current2 = pzem2.current();
    if(!isnan(current2)){
        Serial.print("Current: "); 
        Serial.print(current2); 
        Serial.println("A");
    } else {
        Serial.println("Error reading current");
    }

    float power2 = pzem2.power();
    if(!isnan(power2)){
        Serial.print("Power: "); 
        Serial.print(power2); 
        Serial.println("W");
    } else {
        Serial.println("Error reading power");
    }

    float energy2 = pzem2.energy();
    if(!isnan(energy2)){
        Serial.print("Energy: "); 
        Serial.print(energy2,3); 
        Serial.println("Wh");
    } else {
        Serial.println("Error reading energy");
    }
    if(pzem2.VoltHighAlarm())
    {
        Serial.println("OverVoltage");
    }
    if(pzem2.VoltLowAlarm())
    {
        Serial.println("UnderVoltage");
    }
    Serial.println();
    delay(2000);
}