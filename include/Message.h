#include <Arduino.h>

struct MessagePayload {
    uint64_t packetCount;
    float tdsValue;
    float temperatureValue;
    float turbidityValue;
    float pHValue;
    float batteryValue;
    double latitude;
    double longitude;
};