#ifdef PRINT_MAC

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    Serial.print("Slave MAC: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop() {
    static unsigned long last = 0;
    if (millis() - last >= 5000) {
        last = millis();
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        Serial.print("Slave MAC: ");
        Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

#elif defined(MAIN_PROGRAM)

#include <Arduino.h>
#include "drivers/ciren_comm/ciren_comm.h"

#include <ModbusMaster.h>


#define DE_RE_PIN 26
ModbusMaster node;

// Fill with master mac address
// D8:13:2A:F0:56:40
uint8_t masterMac[] = {0xD8, 0x13, 0x2A, 0xF0, 0x56, 0x40};
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

    if (slave.findMaster(masterMac) == ESP_OK) {
        Serial.println("Connected to master");
        slave.sendSlaveStatus();
    } else {
        Serial.println("Failed to connect to master");
    }
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;

        // Temperature — reg 0x0000
        if (node.readHoldingRegisters(0x0000, 1) == ModbusMaster::ku8MBSuccess) {
            float temp = (float)node.getResponseBuffer(0);
            Serial.printf("Temperature: %.1f\n", temp);
            slave.sendTemperature((uint16_t)(temp * 10));
        }

        // Velocity — reg 0x0001
        if (node.readHoldingRegisters(0x0001, 1) == ModbusMaster::ku8MBSuccess) {
            float vel = (float)node.getResponseBuffer(0);
            Serial.printf("Velocity: %.2f\n", vel);
            slave.sendVelocity((uint16_t)(vel * 100));
        }

        // Displacement — reg 0x0002
        if (node.readHoldingRegisters(0x0002, 1) == ModbusMaster::ku8MBSuccess) {
            float disp = (float)node.getResponseBuffer(0);
            Serial.printf("Displacement: %.2f\n", disp);
            slave.sendDisplacement((uint16_t)(disp * 100));
        }

        // Acceleration — reg 0x0003
        if (node.readHoldingRegisters(0x0003, 1) == ModbusMaster::ku8MBSuccess) {
            float accel = (float)node.getResponseBuffer(0);
            Serial.printf("Acceleration: %.2f\n", accel);
            slave.sendAcceleration((uint16_t)(accel * 100));
        }

        // Vibration Frequency — reg 0x0022, 2 registers → 32-bit
        if (node.readHoldingRegisters(0x0022, 2) == ModbusMaster::ku8MBSuccess) {
            uint32_t freq = ((uint32_t)node.getResponseBuffer(0) << 16)
                          |  (uint32_t)node.getResponseBuffer(1);
            Serial.printf("Frequency: %lu\n", freq);
            slave.sendFrequency(freq);
        }
    }
}

#endif
