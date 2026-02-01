#include <Arduino.h>
#include "drivers/modbus/cwt_sensor.h"
#include "drivers/ciren_comm/ciren_comm.h"

CWTSensor sensor(Serial2);

// Fill with master mac address
uint8_t masterMac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
CIREnSlave slave(5);

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;

void setup() {
    Serial.begin(115200);
    Serial.println("CWT Sensor Starting...");

    sensor.begin(4800);

    Serial.println("Sensor initialized");

    if (slave.findMaster(masterMac) == ESP_OK) {
        Serial.println("Connected to master");
        slave.sendSlaveStatus();
    } else {
        Serial.println("Failed to connect to master");
    }
}

void loop() {
    sensor.task();

    unsigned long currentTime = millis();

    // Sensor Update
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;

        sensor.read_temperature();
        sensor.read_vibration();
    }

    // Get & Print Temperature
    if (sensor.is_temperature_ready()) {
        float temp = sensor.get_temperature();
        Serial.print("Temperature: ");
        Serial.println(temp);

        uint16_t tempData = (uint16_t)(temp * 10);
        slave.sendTemperature(tempData);
    }

    // Get & Print Vibration
    if (sensor.is_vibration_ready()) {
        float vib = sensor.get_vibration();
        Serial.print("Vibration: ");
        Serial.println(vib);

        uint16_t vibData = (uint16_t)(vib * 10);
        slave.sendVibration(vibData);
    }
}