/**
 * @file getAcceleration_receiver.ino
 * @brief Receive LIS331HH acceleration packets via ESP-NOW
 * @details
 *   - Receives struct {ax, ay, az}
 *   - Prints to Serial
 *   - Compatible with Arduino-ESP32 v3.x (ESP-IDF v5.x)
 */

#include <WiFi.h>
#include <esp_now.h>

// Data structure (must match sender)
typedef struct __attribute__((packed)) {
  int16_t ax;
  int16_t ay;
  int16_t az;
} AccelPacket;

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(AccelPacket)) {
    Serial.println("Received packet with invalid size");
    return;
  }

  AccelPacket recvData;
  memcpy(&recvData, data, sizeof(recvData));

  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);

  Serial.print("From: ");
  Serial.print(macStr);
  Serial.printf(" | X:%d mg | Y:%d mg | Z:%d mg\n", recvData.ax, recvData.ay, recvData.az);
}

void setup() {
  Serial.begin(115200);

  // Init WiFi & ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW Receiver ready ✅");
}

void loop() {
  // Nothing needed, data handled in callback
}
