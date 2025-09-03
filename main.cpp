#include <Arduino.h>

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#include <pinout/devkitc.h>
#include <state_machine.h>
#include <logger.h>
#include <Wire.h>

#include <DHT.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <max6675.h>

/*
  ============================
    Table of Contents (TOC)
  ============================
  [1] Constants        : Configuration constants for timing and state.
  [2] Function Declarations : Prototypes for user-defined functions.
  [3] Global Variables : Object instances and sensor data storage.
  [4] Setup            : Initialization routine run once at boot.
  [5] Loop             : Main execution loop using non-blocking logic.
  [6] Function Definitions : Implementations of declared functions.
*/

// [1] ========================= CONSTANTS =========================
// Timing intervals (in milliseconds)
const unsigned int PRINT_INTERVAL = 1000;
const unsigned int BLINK_INTERVAL = 500;
const unsigned int SENSOR_INTERVAL = 1000;
const unsigned int SCREEN_UPDATE_INTERVAL = 200;
const unsigned int ESPNOW_SEARCH_INTERVAL = 5000;

// --- WiFi & ESP-NOW ---
const char *WIFI_SSID = "CASPIA_NET_Plus";
const char *WIFI_PASS = "01101995";

// Main module MAC address to search for (replace with your main module's MAC)
// D8:13:2A:F0:56:40
uint8_t mainModuleMac[6] = {0xD8, 0x13, 0x2A, 0xF0, 0x56, 0x40}; // Will be discovered

// --- Button Pins ---
#define BTN_SELECT 32
#define BTN_DOWN   36
#define BTN_UP     25
#define LED_PIN    16

#define MAX6675_SPI_SCK_PIN GPIO_PIN14
#define MAX6675_SPI_MOSI_PIN GPIO_PIN13
#define MAX6675_SPI_MISO_PIN GPIO_PIN12
#define MAX6675_SPI_CS_PIN GPIO_PIN15

#define DHT_DATA_PIN GPIO_PIN33 

// ESP-NOW data structure
typedef struct {
  float temperature;
  float humidity;
  float max6675_temp;
  uint32_t timestamp;
  char sensor_id[32];
} sensor_data_t;

// [2] ============== FUNCTION DECLARATIONS ==============
void hardware_init();

void state_configuration();
void state_main();
void state_request_id();
void state_introduction();

void screen_draw_intro();
void screen_draw_configuration();
void screen_draw_request_id();

void screen_draw_main_temperature();
void screen_draw_main_vibration();
void screen_draw_main_extreme_temperature();
void screen_draw_main_environment();
void screen_draw_main_settings();
void screen_draw_main_reset();

void led_set_color(uint8_t r, uint8_t g, uint8_t b); 
void update_sensor();

void button_handle_input();
void button_next_page();

// ESP-NOW helpers
void espnow_init();
void espnow_send_data();
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataReceived(const uint8_t * mac, const uint8_t *incomingData, int len);
bool espnow_search_main_module();
void espnow_add_peer();

// [3] ============== GLOBAL VARIABLES ==============

// State machine object
StateMachine programState;
StateMainMenu mainMenuState;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
DHT dht(DHT_DATA_PIN, DHT22); // Pin for DHT sensor
MAX6675 max6675(MAX6675_SPI_SCK_PIN, MAX6675_SPI_CS_PIN, MAX6675_SPI_MISO_PIN);

#define NUMPIXELS   1
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

esp_err_t ret;

// ESP-NOW variables
sensor_data_t sensorData;
bool espnowInitialized = false;
bool mainModuleFound = false;
esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
unsigned long lastSearchTime = 0;

// Logger TAG
const char *TAG_SETUP = "SETUP";
const char *TAG_MAIN = "MAIN_STATE";
const char *TAG_ESPNOW = "ESP_NOW";

// Millis timestamp holders
unsigned long millisPrint = 0;
unsigned long millisLed = 0;
unsigned long millisSensor = 0;
unsigned long millisSendSensorData = 0;
unsigned long millisScreenUpdate = 0;

// Onboard LED state (HIGH = ON, LOW = OFF)
bool ledState = LOW;
bool configState = false;

float DHT_Humidity = 0.0;
float DHT_Temperature = 0.0;
float MAX6675_Temperature = 0.0;

// Global or static variables to simulate button press
bool serialSelectPressed = false;
bool serialUpPressed = false;
bool serialDownPressed = false;

bool isConfigured = false;
uint32_t statusStartTime = 0;
uint8_t macAddress[6];

// --- Menu State ---
const char* menuItems[6] = {
  "Temp & Humidity", "Vibration", "Extreme Temperature",
  "Enviroment", "Settings", "Reset"
};
int selectedMenu = 0;
bool lastStateSelect = HIGH, lastStateUp = HIGH, lastStateDown = HIGH;
char macStr[32]; // Buffer for MAC address string


// [4] ========================= SETUP =========================
void setup() {

  LOGI(TAG_SETUP, "Starting setup...");
  hardware_init();
  LOGI(TAG_SETUP, "Hardware initialized.");

  // Connect WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  LOGI(TAG_SETUP, "WiFi mode set to STA for ESP-NOW");

  // Initialize ESP-NOW
  espnow_init();
  
  programState = StateMachine::STATE_INTRO; 
}

// [5] ========================= LOOP =========================
void loop() {
  // Search for main module if not found
  if (!mainModuleFound && millis() - lastSearchTime > ESPNOW_SEARCH_INTERVAL) {
    espnow_search_main_module();
    lastSearchTime = millis();
  }

  switch (programState) {
    case StateMachine::STATE_INTRO:
      LOGI(TAG_MAIN, "State: Intro");
      state_introduction();
      break;
    case StateMachine::STATE_REQUEST_ID:
      LOGI(TAG_MAIN, "State: Request ID");
      state_request_id();
      break;
    case StateMachine::STATE_CONFIGURATION:
      LOGI(TAG_MAIN, "State: Configuration");
      state_configuration();
      break;
    case StateMachine::STATE_MAIN:
      // LOGI(TAG_MAIN, "State: Main");
      state_main();
      break;
  };
}

// [6] ============== FUNCTION DEFINITIONS ==============

void espnow_init() {
  // Get MAC address
  esp_wifi_get_mac(WIFI_IF_STA, macAddress);
  snprintf(sensorData.sensor_id, sizeof(sensorData.sensor_id), "SM-%02X%02X%02X", 
           macAddress[3], macAddress[4], macAddress[5]);

  if (esp_now_init() != ESP_OK) {
    LOGE(TAG_ESPNOW, "Error initializing ESP-NOW");
    return;
  }
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
  espnowInitialized = true;
  LOGI(TAG_ESPNOW, "ESP-NOW initialized successfully");
  LOGI(TAG_ESPNOW, "Sensor ID: %s", sensorData.sensor_id);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastSendStatus = status;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    LOGI(TAG_ESPNOW, "Data sent successfully to %s", macStr);
  } else {
    LOGE(TAG_ESPNOW, "Failed to send data to %s", macStr);
  }
}

void onDataReceived(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  LOGI(TAG_ESPNOW, "Received data from %s, length: %d", macStr, len);
}

bool espnow_search_main_module() {
  // For now, we'll use a broadcast approach or predefined MAC
  // In a real implementation, you might broadcast a discovery message
  // and wait for main module responses
  
  LOGI(TAG_ESPNOW, "Searching for main module...");
  
  // Example: Set a known main module MAC address
  // Replace these values with your actual main module MAC address
  uint8_t knownMainMac[6] = {0x24, 0x62, 0xAB, 0xD0, 0x00, 0x00}; // Example MAC
  
  // Copy to global variable
  // memcpy(mainModuleMac, knownMainMac, 6);
  
  // Add as peer
  espnow_add_peer();
  
  mainModuleFound = true; // Set to true for now
  LOGI(TAG_ESPNOW, "Main module MAC set to: %02X:%02X:%02X:%02X:%02X:%02X",
       mainModuleMac[0], mainModuleMac[1], mainModuleMac[2],
       mainModuleMac[3], mainModuleMac[4], mainModuleMac[5]);
  
  return mainModuleFound;
}

void espnow_add_peer() {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mainModuleMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    LOGE(TAG_ESPNOW, "Failed to add peer");
    mainModuleFound = false;
  } else {
    LOGI(TAG_ESPNOW, "Peer added successfully");
  }
}

void espnow_send_data() {
  if (!espnowInitialized || !mainModuleFound) {
    LOGW(TAG_ESPNOW, "ESP-NOW not initialized or main module not found");
    return;
  }

  // Prepare data
  sensorData.temperature = DHT_Temperature;
  sensorData.humidity = DHT_Humidity;
  sensorData.max6675_temp = MAX6675_Temperature;
  sensorData.timestamp = millis();

  // Send data
  esp_err_t result = esp_now_send(mainModuleMac, (uint8_t *) &sensorData, sizeof(sensorData));
  
  if (result == ESP_OK) {
    LOGI(TAG_ESPNOW, "Data sent -> Temp: %.2f, Hum: %.2f, MaxT: %.2f", 
         DHT_Temperature, DHT_Humidity, MAX6675_Temperature);
  } else {
    LOGE(TAG_ESPNOW, "Error sending data: %s", esp_err_to_name(result));
  }
}

void state_request_id() {
  button_handle_input();

  ret = esp_wifi_get_mac(WIFI_IF_STA, macAddress);
  if (ret != ESP_OK) {
    LOGE(TAG_MAIN, "Error getting MAC address: %s", esp_err_to_name(ret));
  } 
  
  snprintf(macStr, sizeof(macStr), "MAC:\n%02X:%02X:%02X:%02X:%02X:%02X", 
  macAddress[0], macAddress[1], macAddress[2],
  macAddress[3], macAddress[4], macAddress[5]);
  LOGI(TAG_MAIN, "MAC address: %s", macStr);
  
  screen_draw_request_id();

  delay(5000); 

  programState = StateMachine::STATE_CONFIGURATION;
}

void state_introduction() {
  button_handle_input();

  if(millis() - millisScreenUpdate >= SCREEN_UPDATE_INTERVAL){
    screen_draw_intro();
  }

  if(millis() - millisPrint >= PRINT_INTERVAL) {
    millisPrint = millis();
    LOGI(TAG_MAIN, "In introduction state.");
  }

}

void state_configuration() {
  button_handle_input();

  if(millis() - millisScreenUpdate >= SCREEN_UPDATE_INTERVAL) {
    screen_draw_configuration();
    millisScreenUpdate = millis();
  }
}

void state_main() {
  button_handle_input();

  if(millis() - millisScreenUpdate >= SCREEN_UPDATE_INTERVAL) {
    switch (mainMenuState) {
      case StateMainMenu::STATE_TEMP_HUMIDITY:
        LOGI(TAG_MAIN,"[MENU_1]");
        screen_draw_main_temperature();
        break;
      case StateMainMenu::STATE_VIBRATION:
        LOGI(TAG_MAIN,"[MENU_2]");
        screen_draw_main_vibration();
        break;
      case StateMainMenu::STATE_EXTREME_TEMP:
        LOGI(TAG_MAIN,"[MENU_3]");
        screen_draw_main_extreme_temperature();
        break;
      case StateMainMenu::STATE_ENVIRONMENT:
        LOGI(TAG_MAIN,"[MENU_4]");
        screen_draw_main_environment();
        break;
      case StateMainMenu::STATE_SETTINGS:
        LOGI(TAG_MAIN,"[MENU_5]");
        screen_draw_main_settings();
        break;
      case StateMainMenu::STATE_RESET:
        LOGI(TAG_MAIN,"[MENU_6]");
        screen_draw_main_reset();
        break;
    }
    millisScreenUpdate = millis();
  }

  update_sensor();
}

void update_sensor() {
  if(millis() - millisSensor >= SENSOR_INTERVAL) {
    millisSensor = millis();

    DHT_Humidity = dht.readHumidity();
    ret = isnan(DHT_Humidity);
    if (ret) {
      LOGE(TAG_MAIN, "Failed to read humidity from DHT sensor.");
      return;
    }

    DHT_Temperature = dht.readTemperature();
    ret = isnan(DHT_Temperature);
    if (ret) {
      LOGE(TAG_MAIN, "Failed to read temperature from DHT sensor.");
      return;
    }

    MAX6675_Temperature = max6675.readCelsius();
    ret = isnan(MAX6675_Temperature);
    if (ret) {
      LOGE(TAG_MAIN, "Failed to read temperature from MAX6675 sensor.");
      return;
    }

    LOGI(TAG_MAIN, "Humidity: %.2f%%, Temperature: %.2f°C, MAX6675: %.2f°C", 
         DHT_Humidity, DHT_Temperature, MAX6675_Temperature);

    espnow_send_data();
  }
}

// Initialize hardware
void hardware_init() {
  Serial.begin(115200);

  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup I2C & display
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  

  // Setup DHT sensor
  dht.begin();

  // Setup for display
  u8g2.begin();

  // Setup the navigation buttons
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);

  pixels.begin();
}

void button_handle_input() {
  bool selectPressed = !digitalRead(BTN_SELECT);
  bool upPressed = !digitalRead(BTN_UP);
  bool downPressed = !digitalRead(BTN_DOWN);

  // LOGI(TAG_MAIN, "Button states - Select: %d, Up: %d, Down: %d", selectPressed, upPressed, downPressed);

  if (selectPressed && lastStateSelect == false) {
    if(programState == StateMachine::STATE_CONFIGURATION) {
      if (selectedMenu == 0) {
        mainMenuState = StateMainMenu::STATE_TEMP_HUMIDITY;
      } else if (selectedMenu == 1) {
        mainMenuState = StateMainMenu::STATE_VIBRATION;
      } else if (selectedMenu == 2) {
        mainMenuState = StateMainMenu::STATE_EXTREME_TEMP;
      } else if (selectedMenu == 3) {
        mainMenuState = StateMainMenu::STATE_ENVIRONMENT;
      } else if (selectedMenu == 4) {
        mainMenuState = StateMainMenu::STATE_SETTINGS;
      } else if (selectedMenu == 5) {
        mainMenuState = StateMainMenu::STATE_RESET;
      }
    }
    button_next_page();
  }

  if (programState == StateMachine::STATE_CONFIGURATION) {
    if (downPressed && lastStateDown == false) {
      if (selectedMenu < 5) selectedMenu++;
    }
    if (upPressed && lastStateUp == false) {
      if (selectedMenu > 0) selectedMenu--;
    }
  }

  lastStateSelect = selectPressed;
  lastStateDown = downPressed;
  lastStateUp = upPressed;
}

void button_next_page() {
  if (programState == StateMachine::STATE_INTRO) {
    if(isConfigured){
      programState = StateMachine::STATE_MAIN;
      led_set_color(255, 255, 0); // Yellow for configuration
    } else {
      programState = StateMachine::STATE_REQUEST_ID;
      led_set_color(0, 255, 0); // Green for main
    }
    statusStartTime = millis();

  } else if (programState == StateMachine::STATE_CONFIGURATION) {
    led_set_color(255, 0, 255); // Purple for menu
    programState = StateMachine::STATE_MAIN;

  } else if (programState == StateMachine::STATE_MAIN) {
    led_set_color(0, 0, 255); // Blue
    programState = StateMachine::STATE_INTRO;
  }
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}


void screen_draw_intro() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(6, 25, "ITB de Labo");
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(20, 40, "Sensor Module");

  // Show ESP-NOW status
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 55, "ESP-NOW Protocol");

  // Connection status
  if (espnowInitialized) {
    if (mainModuleFound) {
      u8g2.drawStr(0, 63, "Main Module: Found");
    } else {
      u8g2.drawStr(0, 63, "Main Module: Searching...");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: Initializing");
  }

  u8g2.sendBuffer();
}

void screen_draw_request_id() {
  u8g2.clearBuffer();

  char buffer_mac[32];
  snprintf(buffer_mac, sizeof(buffer_mac), "%02X:%02X:%02X:%02X:%02X:%02X", 
            macAddress[0], macAddress[1], macAddress[2],
            macAddress[3], macAddress[4], macAddress[5]);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 6, buffer_mac);
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(20, 20, "Requesting ID...");

  unsigned long elapsed = (millis() - statusStartTime) / 1000;
  
  char buffer_time[32];
  u8g2.setFont(u8g2_font_6x10_tr);
  sprintf(buffer_time, "Elapsed: %lus", elapsed);
  u8g2.drawStr(15, 35, buffer_time);

  // Show ESP-NOW status
  u8g2.setFont(u8g2_font_4x6_tf);
  if (espnowInitialized) {
    if (mainModuleFound) {
      u8g2.drawStr(0, 63, "ESP-NOW: Connected");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: Searching...");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: Init");
  }

  u8g2.sendBuffer();
}

void screen_draw_configuration() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  int topVisible = selectedMenu / 3 * 3;

  for (int i = 0; i < 3; i++) {
    int index = topVisible + i;
    if (index >= 6) break;

    int y = 15 + i * 15;
    if (index == selectedMenu) {
      u8g2.drawBox(0, y - 10, 128, 14);
      u8g2.setDrawColor(0);
      u8g2.drawStr(5, y, menuItems[index]);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(5, y, menuItems[index]);
    }
  }

  char buffer_footer[64];
  snprintf(buffer_footer, sizeof(buffer_footer), 
           "SM-%d %02X:%02X:%02X:%02X:%02X:%02X", 1,
           macAddress[0], macAddress[1], macAddress[2],
           macAddress[3], macAddress[4], macAddress[5]);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 55, buffer_footer);

  if (mainModuleFound) {
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      u8g2.drawStr(0, 63, "ESP-NOW: OK");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: SEND FAIL");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: NO PEER");
  }

  u8g2.sendBuffer();
}

void screen_draw_main_temperature() {
  u8g2.clearBuffer();

  char buffer_temperature[32];
  snprintf(buffer_temperature, sizeof(buffer_temperature), "Humidity: %.2f%%", DHT_Humidity);
  char buffer_humidity[32];
  snprintf(buffer_humidity, sizeof(buffer_humidity), "Temperature: %.2fC", DHT_Temperature);

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, buffer_humidity);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, buffer_temperature);

  // Show main module MAC if found
  if (mainModuleFound) {
    char mainMacStr[32];
    snprintf(mainMacStr, sizeof(mainMacStr), "Main: %02X:%02X:%02X:%02X:%02X:%02X",
             mainModuleMac[0], mainModuleMac[1], mainModuleMac[2],
             mainModuleMac[3], mainModuleMac[4], mainModuleMac[5]);
    u8g2.drawStr(0, 30, mainMacStr);
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  if (mainModuleFound) {
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      u8g2.drawStr(0, 63, "ESP-NOW: UP");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: FAIL");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: SEARCHING");
  }

  u8g2.sendBuffer();
}

void screen_draw_main_vibration() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "Vibration");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, "No data available");

  if (mainModuleFound) {
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      u8g2.drawStr(0, 63, "ESP-NOW: OK");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: FAIL");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: NO PEER");
  }

  u8g2.sendBuffer();
}

void screen_draw_main_environment() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "Environment");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, "No data available");

  if (mainModuleFound) {
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      u8g2.drawStr(0, 63, "ESP-NOW: OK");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: FAIL");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: NO PEER");
  }

  u8g2.sendBuffer();
}

void screen_draw_main_settings() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "Settings");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, "No settings available");

  if (mainModuleFound) {
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      u8g2.drawStr(0, 63, "ESP-NOW: OK");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: FAIL");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: NO PEER");
  }

  u8g2.sendBuffer();
}

void screen_draw_main_reset() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "Reset");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, "Press to reset device");

  if (mainModuleFound) {
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      u8g2.drawStr(0, 63, "ESP-NOW: OK");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: FAIL");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: NO PEER");
  }

  u8g2.sendBuffer();
}

void screen_draw_main_extreme_temperature() {
  u8g2.clearBuffer();

  char buffer_extreme_temp[32];
  snprintf(buffer_extreme_temp, sizeof(buffer_extreme_temp), "Extreme Temp: %.2fC", MAX6675_Temperature);

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, buffer_extreme_temp);

  if (mainModuleFound) {
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      u8g2.drawStr(0, 63, "ESP-NOW: OK");
    } else {
      u8g2.drawStr(0, 63, "ESP-NOW: FAIL");
    }
  } else {
    u8g2.drawStr(0, 63, "ESP-NOW: SEARCHING");
  }

  u8g2.sendBuffer();
}