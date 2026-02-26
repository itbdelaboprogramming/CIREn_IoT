#ifdef PRINT_MAC

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    Serial.print("Master MAC: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop() {
    static unsigned long last = 0;
    if (millis() - last >= 5000) {
        last = millis();
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        Serial.print("Master MAC: ");
        Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

#elif defined(MAIN_PROGRAM)

#include <Arduino.h>
#include "drivers/ciren_comm/ciren_comm.h"

// 88:57:21:8D:99:F8
uint8_t slave1Mac[] = {0x88, 0x57, 0x21, 0x8D, 0x99, 0xF8};
// uint8_t slave2Mac[] = {0x24, 0x0A, 0xC4, 0x78, 0x9A, 0xBC};
CIREnMaster master;

#ifdef MASTER_COMPACT_PRINT

#define TOTAL_METRICS 19
#define COMPACT_PRINT_INTERVAL 1000

struct SlaveData {
    float   values[TOTAL_METRICS];
    bool    received[TOTAL_METRICS];
    uint8_t id;
    bool    active;
};

static SlaveData     slaveStore[CIREN_MAX_SLAVES];
static uint8_t       slaveStoreCount = 0;
static unsigned long lastPrintTime   = 0;

static const char* metricLabel[TOTAL_METRICS] = {
    nullptr,   // 0 SYSTEM
    "Temp",    // 1
    "Vib",     // 2
    "Vel",     // 3
    "Disp",    // 4
    "Accel",   // 5
    "Freq",    // 6
    "Vel_X",   // 7
    "Vel_Y",   // 8
    "Vel_Z",   // 9
    "Disp_X",  // 10
    "Disp_Y",  // 11
    "Disp_Z",  // 12
    "Accel_X", // 13
    "Accel_Y", // 14
    "Accel_Z", // 15
    "Freq_X",  // 16
    "Freq_Y",  // 17
    "Freq_Z",  // 18
};

static void onMessageCompact(const CIREnMessage* msg) {
    if (msg->metric_type == METRIC_TYPE_SYSTEM) return;

    int slot = -1;
    for (int i = 0; i < slaveStoreCount; i++) {
        if (slaveStore[i].id == msg->id) { slot = i; break; }
    }
    if (slot < 0 && slaveStoreCount < CIREN_MAX_SLAVES) {
        slot = slaveStoreCount++;
        memset(&slaveStore[slot], 0, sizeof(SlaveData));
        slaveStore[slot].id     = msg->id;
        slaveStore[slot].active = true;
    }
    if (slot < 0) return;

    uint8_t t = msg->metric_type;
    if (t == 0 || t >= TOTAL_METRICS) return;

    bool isFreq = (t == METRIC_TYPE_FREQUENCY  ||
                   t == METRIC_TYPE_FREQUENCY_X ||
                   t == METRIC_TYPE_FREQUENCY_Y ||
                   t == METRIC_TYPE_FREQUENCY_Z);

    if (isFreq) {
        slaveStore[slot].values[t] = (float)msg->data.frequency_data;
    } else if (t == METRIC_TYPE_TEMPERATURE) {
        slaveStore[slot].values[t] = msg->data.sensor_data / 10.0f;
    } else {
        slaveStore[slot].values[t] = msg->data.sensor_data / 100.0f;
    }
    slaveStore[slot].received[t] = true;
}

#endif  // MASTER_COMPACT_PRINT

void setup() {
    Serial.begin(115200);
    Serial.println("CIRen Master Starting...");

    if (master.init() == ESP_OK) {
        Serial.println("Master initialized");

        if (master.registerSlave(slave1Mac) == ESP_OK) {
            Serial.println("Slave 1 registered");
        }

        // if (master.registerSlave(slave2Mac) == ESP_OK) {
        //     Serial.println("Slave 2 registered");
        // }

        master.slaveTurnOn(slave1Mac);
        // master.slaveTurnOn(slave2Mac);
    }

    else {
        Serial.println("Master init failed");
    }

#ifdef MASTER_COMPACT_PRINT
    master.setMessageCallback(onMessageCompact);
#endif
}

void loop() {
#ifdef MASTER_COMPACT_PRINT
    if (millis() - lastPrintTime >= COMPACT_PRINT_INTERVAL) {
        lastPrintTime = millis();
        for (int s = 0; s < slaveStoreCount; s++) {
            SlaveData& sd = slaveStore[s];
            if (!sd.active) continue;
            Serial.printf("[ID %d]", sd.id);
            for (int t = 1; t < TOTAL_METRICS; t++) {
                if (metricLabel[t] == nullptr) continue;
                bool isFreq = (t == METRIC_TYPE_FREQUENCY  ||
                               t == METRIC_TYPE_FREQUENCY_X ||
                               t == METRIC_TYPE_FREQUENCY_Y ||
                               t == METRIC_TYPE_FREQUENCY_Z);
                if (sd.received[t]) {
                    if (isFreq) Serial.printf(" %s:%.0f", metricLabel[t], sd.values[t]);
                    else        Serial.printf(" %s:%.2f", metricLabel[t], sd.values[t]);
                } else {
                    Serial.printf(" %s:--", metricLabel[t]);
                }
            }
            Serial.println();
        }
    }
#else
    delay(10);
#endif
}

#endif
