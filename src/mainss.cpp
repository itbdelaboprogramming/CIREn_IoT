// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_now.h>
// #include <pinout/devkitc.h>
// #include <state_machine.h>
// #include <logger.h>
// #include <Wire.h>
// #include <U8g2lib.h>
// #include <Adafruit_NeoPixel.h>

// // ===== DEBUG MODE CONFIGURATION =====
// #define DEBUG_MODE true

// // Timing intervals (in milliseconds)
// const unsigned int PRINT_INTERVAL = 1000;
// const unsigned int SCREEN_UPDATE_INTERVAL = 200;
// const unsigned int DATA_PRINT_INTERVAL = 1000;
// const unsigned int TIMEOUT_INTERVAL = 5000; // Data timeout

// // --- Button Pins ---
// #define BTN_SELECT 33
// #define BTN_DOWN   32
// #define BTN_UP     34
// #define LED_PIN    16
// // 40:22:D8:E8:8B:88
// // ESP-NOW Data Structures
// typedef struct {
//   uint8_t sensorType; // 1=DHT, 2=BNO, 3=MAX6675
//   float value1;
//   float value2;
//   float value3;
//   unsigned long timestamp;
// } SensorData;

// typedef struct {
//   uint8_t mac[6];
//   SensorData data;
//   unsigned long lastUpdate;
//   bool active;
//   char name[20];
// } ConnectedDevice;

// // Function Declarations
// void hardware_init();
// void state_configuration();
// void state_main();
// void state_request_id();
// void state_introduction();
// void screen_draw_intro();
// void screen_draw_configuration();
// void screen_draw_request_id();
// void screen_draw_device_data(int deviceIndex);
// void led_set_color(uint8_t r, uint8_t g, uint8_t b);
// void button_handle_input();
// void button_next_page();
// void print_current_device_data();
// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
// void check_device_timeouts();

// // Global Variables
// StateMachine programState;
// StateMainMenu mainMenuState;
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// #define NUMPIXELS 1
// // Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// const char *TAG_SETUP = "SETUP";
// const char *TAG_MAIN = "MAIN_STATE";
// const char *TAG_ESPNOW = "ESP_NOW";

// // Connected devices array (max 10 devices)
// #define MAX_DEVICES 10
// ConnectedDevice devices[MAX_DEVICES];
// int deviceCount = 0;
// int selectedDevice = 0;

// // Millis timestamp holders
// unsigned long millisPrint = 0;
// unsigned long millisScreenUpdate = 0;
// unsigned long millisDataPrint = 0;

// bool isConfigured = false;
// uint32_t statusStartTime = 0;
// uint8_t macAddress[6];
// bool lastStateSelect = HIGH, lastStateUp = HIGH, lastStateDown = HIGH;
// char macStr[32];

// void setup() {
//   Serial.begin(115200);
  
//   if (DEBUG_MODE) {
//     LOGI(TAG_SETUP, "Starting setup...");
//   }
  
//   hardware_init();
  
//   // Initialize WiFi in STA mode for ESP-NOW
//   WiFi.mode(WIFI_STA);
  
//   // Get MAC address
//   esp_read_mac(macAddress, ESP_MAC_WIFI_STA);
  
//   if (DEBUG_MODE) {
//     LOGI(TAG_SETUP, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
//          macAddress[0], macAddress[1], macAddress[2],
//          macAddress[3], macAddress[4], macAddress[5]);
//   }
  
//   // Initialize ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     if (DEBUG_MODE) {
//       LOGE(TAG_ESPNOW, "Error initializing ESP-NOW");
//     }
//     return;
//   }
  
//   // Register receive callback
//   esp_now_register_recv_cb(OnDataRecv);
  
//   if (DEBUG_MODE) {
//     LOGI(TAG_SETUP, "ESP-NOW initialized successfully");
//   }
  
//   // Initialize devices array
//   for (int i = 0; i < MAX_DEVICES; i++) {
//     devices[i].active = false;
//     devices[i].lastUpdate = 0;
//   }
  
//   programState = StateMachine::STATE_INTRO;
// }

// void loop() {
//   check_device_timeouts();
  
//   switch (programState) {
//     case StateMachine::STATE_INTRO:
//       if (DEBUG_MODE) {
//         LOGI(TAG_MAIN, "State: Intro");
//       }
//       state_introduction();
//       break;
//     case StateMachine::STATE_REQUEST_ID:
//       if (DEBUG_MODE) {
//         LOGI(TAG_MAIN, "State: Request ID");
//       }
//       state_request_id();
//       break;
//     case StateMachine::STATE_CONFIGURATION:
//       if (DEBUG_MODE) {
//         LOGI(TAG_MAIN, "State: Configuration");
//       }
//       state_configuration();
//       break;
//     case StateMachine::STATE_MAIN:
//       state_main();
//       break;
//   }
// }

// // ESP-NOW Receive Callback
// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//   if (len != sizeof(SensorData)) {
//     if (DEBUG_MODE) {
//       LOGE(TAG_ESPNOW, "Invalid data size received");
//     }
//     return;
//   }
  
//   SensorData receivedData;
//   memcpy(&receivedData, incomingData, sizeof(receivedData));
  
//   // Find or add device
//   int deviceIndex = -1;
//   for (int i = 0; i < deviceCount; i++) {
//     if (memcmp(devices[i].mac, mac, 6) == 0) {
//       deviceIndex = i;
//       break;
//     }
//   }
  
//   // New device
//   if (deviceIndex == -1 && deviceCount < MAX_DEVICES) {
//     deviceIndex = deviceCount;
//     memcpy(devices[deviceIndex].mac, mac, 6);
//     deviceCount++;
    
//     // Set device name based on sensor type
//     switch (receivedData.sensorType) {
//       case 1:
//         snprintf(devices[deviceIndex].name, 20, "DHT Module");
//         break;
//       case 2:
//         snprintf(devices[deviceIndex].name, 20, "BNO Module");
//         break;
//       case 3:
//         snprintf(devices[deviceIndex].name, 20, "MAX6675 Module");
//         break;
//       default:
//         snprintf(devices[deviceIndex].name, 20, "Unknown Module");
//         break;
//     }
    
//     if (DEBUG_MODE) {
//       LOGI(TAG_ESPNOW, "New device connected: %s", devices[deviceIndex].name);
//     }
//   }
  
//   if (deviceIndex != -1) {
//     devices[deviceIndex].data = receivedData;
//     devices[deviceIndex].lastUpdate = millis();
//     devices[deviceIndex].active = true;
    
//     if (DEBUG_MODE) {
//       LOGI(TAG_ESPNOW, "Data received from %s - Type: %d, V1: %.2f, V2: %.2f, V3: %.2f",
//            devices[deviceIndex].name, receivedData.sensorType,
//            receivedData.value1, receivedData.value2, receivedData.value3);
//     }
//   }
// }

// void check_device_timeouts() {
//   unsigned long currentMillis = millis();
//   for (int i = 0; i < deviceCount; i++) {
//     if (devices[i].active && (currentMillis - devices[i].lastUpdate > TIMEOUT_INTERVAL)) {
//       devices[i].active = false;
//       if (DEBUG_MODE) {
//         LOGI(TAG_ESPNOW, "Device %s timeout", devices[i].name);
//       }
//     }
//   }
// }

// void state_request_id() {
//   button_handle_input();
  
//   snprintf(macStr, sizeof(macStr), "MAC:\n%02X:%02X:%02X:%02X:%02X:%02X", 
//            macAddress[0], macAddress[1], macAddress[2],
//            macAddress[3], macAddress[4], macAddress[5]);
  
//   screen_draw_request_id();
//   delay(5000);
//   programState = StateMachine::STATE_CONFIGURATION;
// }

// void state_introduction() {
//   button_handle_input();
  
//   if (millis() - millisScreenUpdate >= SCREEN_UPDATE_INTERVAL) {
//     screen_draw_intro();
//     millisScreenUpdate = millis();
//   }
// }

// void state_configuration() {
//   button_handle_input();
  
//   if (millis() - millisScreenUpdate >= SCREEN_UPDATE_INTERVAL) {
//     screen_draw_configuration();
//     millisScreenUpdate = millis();
//   }
// }

// void state_main() {
//   button_handle_input();
  
//   if (millis() - millisScreenUpdate >= SCREEN_UPDATE_INTERVAL) {
//     if (deviceCount > 0 && selectedDevice < deviceCount) {
//       screen_draw_device_data(selectedDevice);
//     } else {
//       u8g2.clearBuffer();
//       u8g2.setFont(u8g2_font_6x12_tr);
//       u8g2.drawStr(0, 30, "No devices");
//       u8g2.drawStr(0, 42, "connected");
//       u8g2.sendBuffer();
//     }
//     millisScreenUpdate = millis();
//   }
  
//   if (!DEBUG_MODE) {
//     print_current_device_data();
//   }
// }

// void print_current_device_data() {
//   if (millis() - millisDataPrint >= DATA_PRINT_INTERVAL) {
//     millisDataPrint = millis();
    
//     if (deviceCount == 0 || selectedDevice >= deviceCount) {
//       Serial.println("No device selected");
//       return;
//     }
    
//     ConnectedDevice *dev = &devices[selectedDevice];
    
//     if (!dev->active) {
//       Serial.println("Device inactive");
//       return;
//     }
    
//     Serial.print(dev->name);
//     Serial.print(" - ");
    
//     switch (dev->data.sensorType) {
//       case 1: // DHT
//         Serial.print("Temp: ");
//         Serial.print(dev->data.value1);
//         Serial.print("°C, Humidity: ");
//         Serial.print(dev->data.value2);
//         Serial.println("%");
//         break;
//       case 2: // BNO
//         Serial.print("Ax: ");
//         Serial.print(dev->data.value1);
//         Serial.print(", Ay: ");
//         Serial.print(dev->data.value2);
//         Serial.print(", Az: ");
//         Serial.println(dev->data.value3);
//         break;
//       case 3: // MAX6675
//         Serial.print("Extreme Temp: ");
//         Serial.print(dev->data.value1);
//         Serial.println("°C");
//         break;
//     }
//   }
// }

// void hardware_init() {
//   // pinMode(LED_BUILTIN, OUTPUT);
//   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
//   u8g2.begin();
//   pinMode(BTN_SELECT, INPUT_PULLUP);
//   pinMode(BTN_DOWN, INPUT_PULLUP);
//   pinMode(BTN_UP, INPUT_PULLUP);
//   // pixels.begin();
//   // led_set_color(255, 0, 0); // Red on startup
// }

// void button_handle_input() {
//   bool selectPressed = !digitalRead(BTN_SELECT);
//   bool upPressed = !digitalRead(BTN_UP);
//   bool downPressed = !digitalRead(BTN_DOWN);
  
//   if (selectPressed && lastStateSelect == false) {
//     button_next_page();
//   }
  
//   if (programState == StateMachine::STATE_CONFIGURATION) {
//     if (downPressed && lastStateDown == false) {
//       if (selectedDevice < deviceCount - 1) selectedDevice++;
//     }
//     if (upPressed && lastStateUp == false) {
//       if (selectedDevice > 0) selectedDevice--;
//     }
//   }
  
//   lastStateSelect = selectPressed;
//   lastStateDown = downPressed;
//   lastStateUp = upPressed;
// }

// void button_next_page() {
//   if (programState == StateMachine::STATE_INTRO) {
//     programState = isConfigured ? StateMachine::STATE_MAIN : StateMachine::STATE_REQUEST_ID;
//     led_set_color(0, 255, 0);
//     statusStartTime = millis();
//   } else if (programState == StateMachine::STATE_CONFIGURATION) {
//     led_set_color(255, 0, 255);
//     programState = StateMachine::STATE_MAIN;
//   } else if (programState == StateMachine::STATE_MAIN) {
//     led_set_color(0, 0, 255);
//     programState = StateMachine::STATE_INTRO;
//   }
// }

// void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
//   // pixels.setPixelColor(0, pixels.Color(r, g, b));
//   // pixels.show();
// }

// void screen_draw_intro() {
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_ncenB14_tr);
//   u8g2.drawStr(6, 25, "ITB de Labo");
//   u8g2.setFont(u8g2_font_6x12_tr);
//   u8g2.drawStr(10, 40, "ESP-NOW Receiver");
//   u8g2.setFont(u8g2_font_4x6_tf);
  
//   char devStr[32];
//   snprintf(devStr, 32, "Devices: %d", deviceCount);
//   u8g2.drawStr(0, 55, devStr);
//   u8g2.drawStr(0, 63, DEBUG_MODE ? "Debug: ON" : "Debug: OFF");
//   u8g2.sendBuffer();
// }

// void screen_draw_request_id() {
//   u8g2.clearBuffer();
//   char buffer_mac[32];
//   snprintf(buffer_mac, sizeof(buffer_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
//            macAddress[0], macAddress[1], macAddress[2],
//            macAddress[3], macAddress[4], macAddress[5]);
  
//   u8g2.setFont(u8g2_font_4x6_tf);
//   u8g2.drawStr(0, 6, buffer_mac);
//   u8g2.setFont(u8g2_font_6x12_tr);
//   u8g2.drawStr(20, 20, "Receiver ID");
  
//   unsigned long elapsed = (millis() - statusStartTime) / 1000;
//   char buffer_time[32];
//   sprintf(buffer_time, "Elapsed: %lus", elapsed);
//   u8g2.setFont(u8g2_font_6x10_tr);
//   u8g2.drawStr(15, 35, buffer_time);
//   u8g2.setFont(u8g2_font_4x6_tf);
//   u8g2.drawStr(0, 63, "Status: Ready");
//   u8g2.sendBuffer();
// }

// void screen_draw_configuration() {
//   u8g2.clearBuffer();
//   u8g2.setFont(u8g2_font_6x12_tr);
  
//   if (deviceCount == 0) {
//     u8g2.drawStr(10, 30, "No devices");
//     u8g2.drawStr(20, 42, "connected");
//   } else {
//     int topVisible = selectedDevice / 3 * 3;
    
//     for (int i = 0; i < 3; i++) {
//       int index = topVisible + i;
//       if (index >= deviceCount) break;
      
//       int y = 15 + i * 15;
//       if (index == selectedDevice) {
//         u8g2.drawBox(0, y - 10, 128, 14);
//         u8g2.setDrawColor(0);
//         u8g2.drawStr(5, y, devices[index].name);
//         u8g2.setDrawColor(1);
//       } else {
//         u8g2.drawStr(5, y, devices[index].name);
//       }
//     }
//   }
  
//   char buffer_footer[64];
//   snprintf(buffer_footer, sizeof(buffer_footer),
//            "Devices: %d/%d", deviceCount, MAX_DEVICES);
//   u8g2.setFont(u8g2_font_4x6_tf);
//   u8g2.drawStr(0, 55, buffer_footer);
//   u8g2.drawStr(0, 63, "Status: OK");
//   u8g2.sendBuffer();
// }

// void screen_draw_device_data(int deviceIndex) {
//   u8g2.clearBuffer();
  
//   if (deviceIndex >= deviceCount) {
//     u8g2.setFont(u8g2_font_6x12_tr);
//     u8g2.drawStr(10, 30, "Invalid device");
//     u8g2.sendBuffer();
//     return;
//   }
  
//   ConnectedDevice *dev = &devices[deviceIndex];
  
//   u8g2.setFont(u8g2_font_6x12_tr);
//   u8g2.drawStr(0, 10, dev->name);
  
//   char buffer[32];
//   u8g2.setFont(u8g2_font_4x6_tf);
  
//   if (!dev->active) {
//     u8g2.drawStr(0, 25, "Status: OFFLINE");
//   } else {
//     switch (dev->data.sensorType) {
//       case 1: // DHT
//         snprintf(buffer, 32, "Temp: %.2fC", dev->data.value1);
//         u8g2.drawStr(0, 22, buffer);
//         snprintf(buffer, 32, "Humidity: %.2f%%", dev->data.value2);
//         u8g2.drawStr(0, 32, buffer);
//         break;
//       case 2: // BNO
//         snprintf(buffer, 32, "X: %.2f", dev->data.value1);
//         u8g2.drawStr(0, 22, buffer);
//         snprintf(buffer, 32, "Y: %.2f", dev->data.value2);
//         u8g2.drawStr(0, 32, buffer);
//         snprintf(buffer, 32, "Z: %.2f", dev->data.value3);
//         u8g2.drawStr(0, 42, buffer);
//         break;
//       case 3: // MAX6675
//         snprintf(buffer, 32, "Extreme: %.2fC", dev->data.value1);
//         u8g2.drawStr(0, 22, buffer);
//         break;
//     }
//   }
  
//   snprintf(buffer, 32, "MAC: %02X:%02X:%02X",
//            dev->mac[3], dev->mac[4], dev->mac[5]);
//   u8g2.drawStr(0, 52, buffer);
//   u8g2.drawStr(0, 63, dev->active ? "Status: Active" : "Status: Timeout");
  
//   u8g2.sendBuffer();
// }