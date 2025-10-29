#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <DFRobot_LIS.h>
#include <pinout/devkitc.h>

// ===== CONFIGURATION =====
#define DEBUG_MODE true

// IMPORTANT: Replace with your receiver's MAC address
uint8_t receiverAddress[] = {0x40, 0x22, 0xD8, 0xE8, 0x8B, 0x88};

// Timing
const unsigned long SEND_INTERVAL = 2000; // Send data every 2 seconds
const unsigned long SENSOR_READ_INTERVAL = 1000;

// ESP-NOW Data Structure (must match receiver)
typedef struct {
  uint8_t sensorType; // 1=DHT, 2=LIS331HH, 3=MAX6675
  float value1;       // Acceleration X (in mg)
  float value2;       // Acceleration Y (in mg)
  float value3;       // Acceleration Z (in mg)
  unsigned long timestamp;
} SensorData;

// Global Variables
// Using I2C communication with LIS331HH at address 0x18
DFRobot_LIS331HH_I2C acce(&Wire, 0x18);

SensorData sensorData;
esp_now_peer_info_t peerInfo;
unsigned long lastSendTime = 0;
unsigned long lastReadTime = 0;
long ax = 0, ay = 0, az = 0;

// Function Declarations
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void readSensor();
void sendData();

void setup() {
  Serial.begin(115200);
  
  Serial.println("\n=== LIS331HH Accelerometer Module ===");
  
  // Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Initialize LIS331HH
  while(!acce.begin()) {
    delay(1000);
    Serial.println("LIS331HH initialization failed, please check the connection and I2C address setting");
  }
  
  // Get chip id
  Serial.print("Chip ID: 0x");
  Serial.println(acce.getID(), HEX);
  
  /**
   * Set range: Range(g)
   *   eLis331hh_6g = 6,   // ±6g
   *   eLis331hh_12g = 12, // ±12g
   *   eLis331hh_24g = 24  // ±24g
   */
  acce.setRange(DFRobot_LIS::eLis331hh_6g);
  Serial.println("Range set to ±6g");
  
  /**
   * Set data measurement rate:
   *   ePowerDown_0HZ = 0,
   *   eLowPower_halfHZ,
   *   eLowPower_1HZ,
   *   eLowPower_2HZ,
   *   eLowPower_5HZ,
   *   eLowPower_10HZ,
   *   eNormal_50HZ,
   *   eNormal_100HZ,
   *   eNormal_400HZ,
   *   eNormal_1000HZ,
   */
  acce.setAcquireRate(DFRobot_LIS::eNormal_1000HZ);
  Serial.println("Acquire rate set to 1000Hz");
  
  Serial.println("LIS331HH initialized successfully");
  
  // Set WiFi mode
  WiFi.mode(WIFI_STA);
  
  // Get and print MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer (receiver)
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized successfully");
  Serial.println("NOTE: Update receiverAddress[] with your receiver's MAC!");
  Serial.println("\n=== Starting acceleration measurements ===");
  
  // Initialize sensor data structure
  sensorData.sensorType = 2; // LIS331HH sensor type (keeping 2 for compatibility)
  
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Read sensor at defined interval
  if (currentMillis - lastReadTime >= SENSOR_READ_INTERVAL) {
    lastReadTime = currentMillis;
    readSensor();
  }
  
  // Send data at defined interval
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;
    sendData();
  }
}

void readSensor() {
  // Get the acceleration in the three directions of xyz
  // The measurement range is ±6g (set in setup)
  ax = acce.readAccX(); // Get the acceleration in the x direction (mg)
  ay = acce.readAccY(); // Get the acceleration in the y direction (mg)
  az = acce.readAccZ(); // Get the acceleration in the z direction (mg)
  
  if (DEBUG_MODE) {
    Serial.print("x: ");
    Serial.print(ax);
    Serial.print(" mg \ty: ");
    Serial.print(ay);
    Serial.print(" mg \tz: ");
    Serial.print(az);
    Serial.println(" mg");
  }
}

void sendData() {
  // Prepare data packet
  // Convert long to float for consistency with data structure
  sensorData.value1 = (float)ax;
  sensorData.value2 = (float)ay;
  sensorData.value3 = (float)az;
  sensorData.timestamp = millis();
  
  // Send data
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&sensorData, sizeof(sensorData));
  
  if (result == ESP_OK) {
    if (DEBUG_MODE) {
      Serial.println("Data sent successfully");
    }
  } else {
    Serial.println("Error sending data");
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG_MODE) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  }
}