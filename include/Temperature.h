#ifndef TemperatureDS18B_H
#define TemperatureDS18B_H

#include <Arduino.h>
#include <OneWire.h>

class TemperatureDallas
{
private:
    const int8_t pin = 2;
    byte data[12];
    byte addr[8];
    OneWire ds = OneWire(pin);

public:
    TemperatureDallas();
    ~TemperatureDallas();

    int begin();
    float getTemperature();
    void resetOneWire();
};

#endif