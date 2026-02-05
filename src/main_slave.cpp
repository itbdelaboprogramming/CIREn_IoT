#include <Arduino.h>
#include "drivers/ciren_comm/ciren_comm.h"

#include <ModbusMaster.h>


#define DE_RE_PIN 26
ModbusMaster node;

// Fill with master mac address
uint8_t masterMac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
CIREnSlave slave(5);

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;

void preTransmit() {
    digitalWrite(DE_RE_PIN, HIGH);   // enable transmit
}

void postTransmit() {
    digitalWrite(DE_RE_PIN, LOW);    // enable receive
}

void setup() {
    Serial.begin(115200);
    Serial.println("CWT Sensor Starting...");

    pinMode(DE_RE_PIN, OUTPUT);
    digitalWrite(DE_RE_PIN, LOW);           // start in receive mode
    Serial2.begin(4800, SERIAL_8N1, 16, 17);
    node.begin(1, Serial2);             // slave ID 0x01, stream = Serial2
    node.preTransmission(preTransmit);
    node.postTransmission(postTransmit);

    Serial.println("Sensor initialized");

    // if (slave.findMaster(masterMac) == ESP_OK) {
    //     Serial.println("Connected to master");
    //     slave.sendSlaveStatus();
    // } else {
    //     Serial.println("Failed to connect to master");
    // }
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;

        // Read Temperature
        if (node.readHoldingRegisters(0x0000, 1) == ModbusMaster::ku8MBSuccess) {
            float temp = (float)node.getResponseBuffer(0);
            Serial.print("Temperature: ");
            Serial.println(temp);

            uint16_t tempData = (uint16_t)(temp * 10);
            // slave.sendTemperature(tempData);
        }

        // Read Vibration
        if (node.readHoldingRegisters(0x0003, 1) == ModbusMaster::ku8MBSuccess) {
            float vib = (float)node.getResponseBuffer(0);
            Serial.print("Vibration: ");
            Serial.println(vib);

            uint16_t vibData = (uint16_t)(vib * 10);
            // slave.sendVibration(vibData);
        }
    }
}