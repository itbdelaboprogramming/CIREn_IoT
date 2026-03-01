#ifdef SENSOR_ID_CONFIG

#include <Arduino.h>
#include <ModbusMaster.h>

#define DE_RE_PIN           5
#define CWT_REG_DEVICE_ADDR 0x07D0
#define CWT_REG_VERSION     0x07D5

ModbusMaster node;

void preTransmit() { digitalWrite(DE_RE_PIN, HIGH); }
void postTransmit() { digitalWrite(DE_RE_PIN, LOW);  }

static void printMenu() {
    Serial.println();
    Serial.println("=== CWT Sensor ID Config ===");
    Serial.println("Connect ONE sensor at a time for reliable discovery.");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  s              - Discover connected sensor (broadcast to 0xFF)");
    Serial.println("  r <id>         - Read device address register from sensor at <id>");
    Serial.println("  w <id> <new>   - Write new ID to sensor currently at <id>");
    Serial.println();
}

static void discoverSensor() {
    Serial.println("Querying broadcast address 0xFF...");
    node.begin(0xFF, Serial2);
    delay(50);
    if (node.readHoldingRegisters(CWT_REG_DEVICE_ADDR, 1) == ModbusMaster::ku8MBSuccess) {
        uint16_t currentId = node.getResponseBuffer(0);
        Serial.printf("  Found sensor! Current ID: 0x%02X (%d)\n", currentId, currentId);
        if (node.readHoldingRegisters(CWT_REG_VERSION, 1) == ModbusMaster::ku8MBSuccess) {
            uint16_t ver = node.getResponseBuffer(0);
            if (ver == 0x0B) {
                Serial.println("  Sensor type: Tri-axis (0x0B)");
            } else {
                Serial.printf("  Sensor type: Single-axis / version 0x%02X\n", ver);
            }
        }
    } else {
        Serial.println("  No response (check connection, power, baud rate).");
    }
}

static void readId(uint8_t id) {
    Serial.printf("Reading device address register from sensor at ID 0x%02X...\n", id);
    node.begin(id, Serial2);
    delay(50);
    if (node.readHoldingRegisters(CWT_REG_DEVICE_ADDR, 1) == ModbusMaster::ku8MBSuccess) {
        uint16_t val = node.getResponseBuffer(0);
        Serial.printf("  Reported ID: 0x%02X (%d)\n", val, val);
    } else {
        Serial.printf("  No response from ID 0x%02X.\n", id);
    }
}

static void writeId(uint8_t currentId, uint8_t newId) {
    Serial.printf("Writing new ID 0x%02X to sensor at 0x%02X...\n", newId, currentId);
    node.begin(currentId, Serial2);
    delay(50);
    if (node.writeSingleRegister(CWT_REG_DEVICE_ADDR, newId) == ModbusMaster::ku8MBSuccess) {
        Serial.println("  Write succeeded. Verifying...");
        delay(200);
        node.begin(newId, Serial2);
        delay(50);
        if (node.readHoldingRegisters(CWT_REG_DEVICE_ADDR, 1) == ModbusMaster::ku8MBSuccess) {
            uint16_t val = node.getResponseBuffer(0);
            if (val == newId) {
                Serial.printf("  Verified: sensor now at ID 0x%02X (%d)\n", newId, newId);
            } else {
                Serial.printf("  Mismatch: sensor reports 0x%02X instead of 0x%02X\n", val, newId);
            }
        } else {
            Serial.printf("  Verification failed: no response at new ID 0x%02X.\n", newId);
        }
    } else {
        Serial.println("  Write failed.");
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(DE_RE_PIN, OUTPUT);
    digitalWrite(DE_RE_PIN, LOW);
    Serial2.begin(4800, SERIAL_8N1, 16, 17);
    node.preTransmission(preTransmit);
    node.postTransmission(postTransmit);

    printMenu();
    Serial.println("Auto-discovering on startup...");
    discoverSensor();
    printMenu();
}

void loop() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    char cmd = line.charAt(0);
    if (cmd == 's' || cmd == 'S') {
        discoverSensor();
    } else if (cmd == 'r' || cmd == 'R') {
        int space = line.indexOf(' ');
        if (space < 0) { Serial.println("Usage: r <id>"); return; }
        uint8_t id = (uint8_t)strtol(line.substring(space + 1).c_str(), nullptr, 0);
        readId(id);
    } else if (cmd == 'w' || cmd == 'W') {
        int sp1 = line.indexOf(' ');
        if (sp1 < 0) { Serial.println("Usage: w <id> <new_id>"); return; }
        int sp2 = line.indexOf(' ', sp1 + 1);
        if (sp2 < 0) { Serial.println("Usage: w <id> <new_id>"); return; }
        uint8_t curId = (uint8_t)strtol(line.substring(sp1 + 1, sp2).c_str(), nullptr, 0);
        uint8_t newId = (uint8_t)strtol(line.substring(sp2 + 1).c_str(), nullptr, 0);
        writeId(curId, newId);
    } else {
        printMenu();
    }
}

#endif  // SENSOR_ID_CONFIG
