#pragma once

#include <DHT.h>

class DHTSensor {
public:
    DHTSensor(uint8_t pin, uint8_t type);
    void begin();
    float readTemperature(bool isFahrenheit = false);
    float readHumidity();
    float readHeatIndex(bool isFahrenheit = false);

private:
    DHT dht;
};
