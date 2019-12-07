/*
Copyright (c) 2019 O. Lorenz

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


/*
 * PZEM017.h
 *
 * Interface library for a different version: PZEM017 DC Energy Monitor
 * Based on the PZEM004T library by @olehs https://github.com/olehs/PZEM017T
 *
 * Author: O. Lorenz https://github.com/lorenz4672
 * 
 *
*/


#ifndef PZEM017_H
#define PZEM017_H



#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// #define PZEM017_NO_SWSERIAL
#if (not defined(PZEM017_NO_SWSERIAL)) && (defined(__AVR__) || defined(ESP8266) && (not defined(ESP32)))
#define PZEM017_SOFTSERIAL
#endif

#if defined(PZEM017_SOFTSERIAL)
#include <SoftwareSerial.h>
#endif


#define PZEM_DEFAULT_ADDR    0x01


class PZEM017
{
public:
#if defined(PZEM017_SOFTSERIAL)
    PZEM017(uint8_t receivePin, uint8_t transmitPin, uint8_t addr=PZEM_DEFAULT_ADDR);
#endif
    PZEM017(HardwareSerial* port, uint8_t addr=PZEM_DEFAULT_ADDR);
    ~PZEM017();


    float voltage();
    float current();
    float power();
    float energy();
    bool VoltHighAlarm();
    bool VoltLowAlarm();

    bool setAddress(uint8_t addr);
    uint8_t getAddress();

    bool setLOWVoltageAlarm(uint16_t volt);
    bool setHIVoltageAlarm(uint16_t volt);
    bool getPowerAlarm();

    bool resetEnergy();
    bool getSlaveParameters();
    bool setCurrentShunt(uint16_t shunt);
    void search();

private:

    Stream* _serial; // Serial interface
    bool _isSoft;    // Is serial interface software

    uint8_t _addr;   // Device address

    struct {
        float voltage;
        float current;
        float power;
        float energy;
        uint16_t VoltHighAlarm;
        uint16_t VoltLowAlarm;
    }  _currentValues; // Measured values

        struct {
        float HIVoltTHR;
        float LOWVoltTHR;
        uint32_t MODBUS_ADDR;
        uint32_t SHUNT_VAL;
    }  _currentParameters; // Measured values

    uint64_t _lastRead; // Last time values were updated



    void init(uint8_t addr); // Init common to all constructors

    bool updateValues();    // Get most up to date values from device registers and cache them
    uint16_t recieve(uint8_t *resp, uint16_t len); // Receive len bytes into a buffer

    bool sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check=false, uint16_t slave_addr=0xFFFF); // Send 8 byte command

    void setCRC(uint8_t *buf, uint16_t len);           // Set the CRC for a buffer
    bool checkCRC(const uint8_t *buf, uint16_t len);   // Check CRC of buffer

    uint16_t CRC16(const uint8_t *data, uint16_t len); // Calculate CRC of buffer
};

#endif // PZEM017T_H
