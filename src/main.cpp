#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PZEM017.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* SSID = "TP-LINK_2.4GHz";
const char* PSK = "CB4ED22F52";
const char* MQTT_BROKER = "192.168.0.19";
 
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
/* Hardware Serial3 is only available on certain boards.
 * For example the Arduino MEGA 2560
*/

PZEM017 pzem1(5,4,0x02,9500);
PZEM017 pzem2(5,4,0x01,9400);
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(SSID);
 
    WiFi.begin(SSID, PSK);
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
 
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
void setup() {
  Serial.begin(115200);
      setup_wifi();
    client.setServer(MQTT_BROKER, 1883);
  //pzem.setAddress(0x02);
   pzem2.setCurrentShunt(1);
  // pzem1.setCurrentShunt(1);
  
}
void reconnect() {
    while (!client.connected()) {
        Serial.print("Reconnecting...");
        if (!client.connect("ESP8266Client")) {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}
void loop() {
StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
 
  // JSONencoder["Voltage1"] = "ESP32";
  // JSONencoder["Current1"] = "Temperature";
  // JSONencoder["Power1"] = "Temperature";
  // JSONencoder["Energy1"] = "Temperature";

  // JSONencoder["Voltage2"] = "ESP32";
  // JSONencoder["Current2"] = "Temperature";
  // JSONencoder["Power2"] = "Temperature";
  // JSONencoder["Energy2"] = "Temperature";




    //pzem.setHIVoltageAlarm(18);
    //pzem.setLOWVoltageAlarm(10);
    //pzem1.getSlaveParameters();
    //delay(1000);
        pzem2.getSlaveParameters();
        

    float voltage = pzem1.voltage();
    if(!isnan(voltage)){
        Serial.print("Voltage: "); 
        Serial.print(voltage); 
        JSONencoder["Voltage1"] = voltage;
        Serial.println("V");
    } else {
        Serial.println("Error reading voltage");
    }

    float current = pzem1.current();
    if(!isnan(current)){
        Serial.print("Current: "); 
        Serial.print(current); 
          JSONencoder["Current1"] = current;
        Serial.println("A");
    } else {
        Serial.println("Error reading current");
    }

    float power = pzem1.power();
    if(!isnan(power)){
        Serial.print("Power: "); 
        Serial.print(power); 
          JSONencoder["Power1"] = power;
        Serial.println("W");
    } else {
        Serial.println("Error reading power");
    }

    float energy = pzem1.energy();
    if(!isnan(energy)){
        Serial.print("Energy: "); 
        Serial.print(energy,3); 
          JSONencoder["Energy1"] = energy;
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
        JSONencoder["Voltage2"] = voltage2;
        Serial.println("V");
    } else {
        Serial.println("Error reading voltage");
    }

    float current2 = pzem2.current();
    if(!isnan(current2)){
        Serial.print("Current: "); 
        Serial.print(current2); 
        JSONencoder["Current2"] = current2;
        Serial.println("A");
    } else {
        Serial.println("Error reading current");
    }

    float power2 = pzem2.power();
    if(!isnan(power2)){
        Serial.print("Power: "); 
        Serial.print(power2); 
        JSONencoder["Power2"] = power2;
        Serial.println("W");
    } else {
        Serial.println("Error reading power");
    }

    float energy2 = pzem2.energy();
    if(!isnan(energy2)){
        Serial.print("Energy: "); 
        Serial.print(energy2,3); 
        JSONencoder["Energy2"] = energy2;
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
//built JSON    
      char JSONmessageBuffer[300];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

    if (!client.connected()) {
        reconnect();
    }
    client.loop();
 
    client.publish("test", JSONmessageBuffer);

    delay(2000);
}