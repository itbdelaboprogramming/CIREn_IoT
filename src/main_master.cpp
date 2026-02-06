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
}

void loop() {
    delay(10);
}

#endif
