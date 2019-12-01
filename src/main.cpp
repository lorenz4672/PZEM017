#include <Arduino.h>

#include "PZEM004Tv30.h"

PZEM004Tv30 pzem(&Serial3);

void setup() {
  Serial.begin(115200);

  Serial.print("Reset Energy");
  pzem.resetEnergy();


  Serial.print("Set address to 0x42");
  pzem.setAddress(0x42);
}

void loop() {
  float volt = pzem.voltage();
  Serial.print("Voltage: ");
  Serial.print(volt);
  Serial.println("V");

  float cur = pzem.current();
  Serial.print("Current: ");
  Serial.print(cur);
  Serial.println("A");

  float powe = pzem.power();
  Serial.print("Power: ");
  Serial.print(powe);
  Serial.println("W");

  float ener = pzem.energy();
  Serial.print("Energy: ");
  Serial.print(ener,3);
  Serial.println("kWh");

  delay(1000);
}