#ifndef HCSR04_SENSOR_H
#define HCSR04_SENSOR_H

#include <Arduino.h>
#include <HCSR04.h>

class HCSR04Sensor {
  public:
    HCSR04Sensor(uint8_t trigPin, uint8_t echoPin);
    void begin();
    float getDistanceMeters();  // Returns distance in meters

  private:
    uint8_t _trigPin;
    uint8_t _echoPin;
};

#endif
