#include "Temperature.h"

TemperatureDallas::TemperatureDallas() : ds(OneWire(2)) {}
TemperatureDallas::~TemperatureDallas() {}

int TemperatureDallas::begin()
{

    if (!ds.search(addr))
    {
        // no more sensors on chain, reset search
        ds.reset_search();
        return -1000;
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
        return -2000;
    }

    if (addr[0] != 0x10 && addr[0] != 0x28)
    {
        return -3000;
    }
}

float TemperatureDallas::getTemperature()
{
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end

    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    for (int i = 0; i < 9; i++)
    { // we need 9 bytes
        data[i] = ds.read();
    }

    ds.reset_search();

    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB << 8) | LSB); // using two's compliment
    float TemperatureSum = tempRead / 16;
    float correctedTemp = TemperatureSum;

    char formattedTemp[10]; // Adjust the buffer size as needed
    sprintf(formattedTemp, "%.2f", correctedTemp);

    return atof(formattedTemp);
}

void TemperatureDallas::resetOneWire()
{
    ds.reset();
}