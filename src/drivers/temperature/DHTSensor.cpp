#include "DHTSensor.h"

DHTSensor::DHTSensor(uint8_t pin, uint8_t type)
    : dht(pin, type) {}

void DHTSensor::begin() {
    dht.begin();
}

float DHTSensor::readTemperature(bool isFahrenheit) {
    return dht.readTemperature(isFahrenheit);
}

float DHTSensor::readHumidity() {
    return dht.readHumidity();
}

float DHTSensor::readHeatIndex(bool isFahrenheit) {
    float t = readTemperature(isFahrenheit);
    float h = readHumidity();
    return dht.computeHeatIndex(t, h, isFahrenheit);
}
