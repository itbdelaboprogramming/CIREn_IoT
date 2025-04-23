#include "HCSR04Sensor.h"

HCSR04Sensor::HCSR04Sensor(uint8_t trigPin, uint8_t echoPin) {
  _trigPin = trigPin;
  _echoPin = echoPin;
}

void HCSR04Sensor::begin() {
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  digitalWrite(_trigPin, LOW);
}

float HCSR04Sensor::getDistanceMeters() {
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  long duration = pulseIn(_echoPin, HIGH, 30000); // Timeout after 30ms
  if (duration == 0) return -1.0; // No echo received

  // Convert duration to distance in meters (speed of sound = 343 m/s)
  float distanceMeters = (duration / 2.0) * 0.000343;
  return distanceMeters;
}
