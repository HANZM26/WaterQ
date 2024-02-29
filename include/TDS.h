#ifndef TDS_H
#define TDS_H

#include <Arduino.h>

class GravityTDS
{
private:
    const int8_t pinTDS = 39;
    const static int SCOUNT = 30; // sum of sample point
    int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
    int analogBufferTemp[SCOUNT];
    int analogBufferIndex = 0, copyIndex = 0;
    float referenceTemperature;
    float averageVoltage = 0, temperature = 32;
    float tdsValue;

    int getMedianNum(int bArray[], int iFilterLen)
    {
        int bTab[iFilterLen];
        for (byte i = 0; i < iFilterLen; i++)
            bTab[i] = bArray[i];
        int i, j, bTemp;
        for (j = 0; j < iFilterLen - 1; j++)
        {
            for (i = 0; i < iFilterLen - j - 1; i++)
            {
                if (bTab[i] > bTab[i + 1])
                {
                    bTemp = bTab[i];
                    bTab[i] = bTab[i + 1];
                    bTab[i + 1] = bTemp;
                }
            }
        }
        if ((iFilterLen & 1) > 0)
            bTemp = bTab[(iFilterLen - 1) / 2];
        else
            bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
        return bTemp;
    }

public:
    GravityTDS();
    ~GravityTDS();

    void begin();
    void setReferenceTemperature(float temperature);
    void updateTDS();
    float readTDS();
};

#endif