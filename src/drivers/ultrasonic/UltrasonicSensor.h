#pragma

#include <Arduino.h>
#include <HCSR04.h>

class UltrasonicSensor {
  public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);
    void begin();
    float getDistanceMeters();  // Returns distance in meters

  private:
    uint8_t _trigPin;
    uint8_t _echoPin;
};
