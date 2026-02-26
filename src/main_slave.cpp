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
uint16_t sensorVersion = 0x0A;  // default to single-axis

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

    // Detect sensor version — blocking until valid (0x0A or 0x0B)
    Serial.println("Detecting sensor version...");
    while (true) {
        if (node.readHoldingRegisters(0x07D5, 1) == ModbusMaster::ku8MBSuccess) {
            uint16_t ver = node.getResponseBuffer(0);
            if (ver == 0x0A || ver == 0x0B) {
                sensorVersion = ver;
                Serial.printf("Sensor version: 0x%02X\n", sensorVersion);
                break;
            } else {
                Serial.printf("Invalid version byte: 0x%02X, retrying...\n", ver);
            }
        } else {
            Serial.println("Version read failed, retrying...");
        }
        delay(500);
    }

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

        if (sensorVersion == 0x0B) {
            // --- Tri-axis sensor (0x0B) ---

            // Temperature — reg 0x0000
            if (node.readHoldingRegisters(0x0000, 1) == ModbusMaster::ku8MBSuccess) {
                float temp = (float)node.getResponseBuffer(0);
                Serial.printf("Temperature: %.1f\n", temp);
                slave.sendTemperature((uint16_t)(temp * 10));
            }

            // X/Y/Z Velocity — 0x0001, 0x0002, 0x0003
            if (node.readHoldingRegisters(0x0001, 1) == ModbusMaster::ku8MBSuccess) {
                float vel = (float)node.getResponseBuffer(0);
                Serial.printf("Velocity X: %.2f\n", vel);
                slave.sendVelocityX((uint16_t)(vel * 100));
            }
            if (node.readHoldingRegisters(0x0002, 1) == ModbusMaster::ku8MBSuccess) {
                float vel = (float)node.getResponseBuffer(0);
                Serial.printf("Velocity Y: %.2f\n", vel);
                slave.sendVelocityY((uint16_t)(vel * 100));
            }
            if (node.readHoldingRegisters(0x0003, 1) == ModbusMaster::ku8MBSuccess) {
                float vel = (float)node.getResponseBuffer(0);
                Serial.printf("Velocity Z: %.2f\n", vel);
                slave.sendVelocityZ((uint16_t)(vel * 100));
            }

            // X/Y/Z Displacement — 0x0004, 0x0005, 0x0006
            if (node.readHoldingRegisters(0x0004, 1) == ModbusMaster::ku8MBSuccess) {
                float disp = (float)node.getResponseBuffer(0);
                Serial.printf("Displacement X: %.2f\n", disp);
                slave.sendDisplacementX((uint16_t)(disp * 100));
            }
            if (node.readHoldingRegisters(0x0005, 1) == ModbusMaster::ku8MBSuccess) {
                float disp = (float)node.getResponseBuffer(0);
                Serial.printf("Displacement Y: %.2f\n", disp);
                slave.sendDisplacementY((uint16_t)(disp * 100));
            }
            if (node.readHoldingRegisters(0x0006, 1) == ModbusMaster::ku8MBSuccess) {
                float disp = (float)node.getResponseBuffer(0);
                Serial.printf("Displacement Z: %.2f\n", disp);
                slave.sendDisplacementZ((uint16_t)(disp * 100));
            }

            // X/Y/Z Acceleration — 0x000A, 0x000B, 0x000C
            if (node.readHoldingRegisters(0x000A, 1) == ModbusMaster::ku8MBSuccess) {
                float accel = (float)node.getResponseBuffer(0);
                Serial.printf("Acceleration X: %.2f\n", accel);
                slave.sendAccelerationX((uint16_t)(accel * 100));
            }
            if (node.readHoldingRegisters(0x000B, 1) == ModbusMaster::ku8MBSuccess) {
                float accel = (float)node.getResponseBuffer(0);
                Serial.printf("Acceleration Y: %.2f\n", accel);
                slave.sendAccelerationY((uint16_t)(accel * 100));
            }
            if (node.readHoldingRegisters(0x000C, 1) == ModbusMaster::ku8MBSuccess) {
                float accel = (float)node.getResponseBuffer(0);
                Serial.printf("Acceleration Z: %.2f\n", accel);
                slave.sendAccelerationZ((uint16_t)(accel * 100));
            }

            // X Frequency — 0x0021 (2 regs), Y — 0x0023 (2 regs), Z — 0x0025 (2 regs)
            if (node.readHoldingRegisters(0x0021, 2) == ModbusMaster::ku8MBSuccess) {
                uint32_t freq = ((uint32_t)node.getResponseBuffer(0) << 16)
                              |  (uint32_t)node.getResponseBuffer(1);
                Serial.printf("Frequency X: %lu\n", freq);
                slave.sendFrequencyX(freq);
            }
            if (node.readHoldingRegisters(0x0023, 2) == ModbusMaster::ku8MBSuccess) {
                uint32_t freq = ((uint32_t)node.getResponseBuffer(0) << 16)
                              |  (uint32_t)node.getResponseBuffer(1);
                Serial.printf("Frequency Y: %lu\n", freq);
                slave.sendFrequencyY(freq);
            }
            if (node.readHoldingRegisters(0x0025, 2) == ModbusMaster::ku8MBSuccess) {
                uint32_t freq = ((uint32_t)node.getResponseBuffer(0) << 16)
                              |  (uint32_t)node.getResponseBuffer(1);
                Serial.printf("Frequency Z: %lu\n", freq);
                slave.sendFrequencyZ(freq);
            }

        } else {
            // --- Single-axis sensor (0x0A) ---

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
}

#endif
