#include "TCRT5000Sensor.h"

TCRT5000Sensor::TCRT5000Sensor(uint8_t receiverPin)
    : _receiverPin(receiverPin), _ledPin(-1) {
    pinMode(_receiverPin, INPUT);
}

TCRT5000Sensor::TCRT5000Sensor(uint8_t receiverPin, uint8_t ledPin)
    : _receiverPin(receiverPin), _ledPin(ledPin) {
    pinMode(_receiverPin, INPUT);
    pinMode(_ledPin, OUTPUT);
}

bool TCRT5000Sensor::isClose() {
    if (_ledPin == -1) {
        // Passive mode: read directly from receiver
        return !digitalRead(_receiverPin);
    } else {
        // Active mode: pulse the LED, then read
        digitalWrite(_ledPin, HIGH);
        delayMicroseconds(100);  // allow diode to power up
        bool status = !digitalRead(_receiverPin);
        digitalWrite(_ledPin, LOW);
        return status;
    }
}
