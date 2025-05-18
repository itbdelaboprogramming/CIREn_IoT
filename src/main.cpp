#include <Arduino.h>

#include <WiFi.h>
#include <esp_wifi.h>

#include <pinout/devkitc.h>
#include <state_machine.h>
#include <logger.h>
#include <ESP_SSD1306.h>
#include <Wire.h>

#include <drivers/canbus/Canbus.h>



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
const unsigned int DISPLAY_INTERVAL = 2000;
const unsigned int CAN_HEARTBEAT = 100;


// [2] ============== FUNCTION DECLARATIONS ==============
void hardware_init();
void led_blink();

void send_heartbeat();
void state_configuration();
void state_main();
void state_request_id();

// [3] ============== GLOBAL VARIABLES ==============

// State machine object
StateMachine programState;
ESP_SSD1306 display(-1); // I2C pins for ESP32 DevKitC

spi_device_handle_t spiHandle;
MCP2515 mcp2515(&spiHandle); 
Canbus canbus(SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CS_PIN, 0, spiHandle, mcp2515); // SCK, MOSI, MISO, CS, INT

esp_err_t ret;

// Logger TAG
const char *TAG_SETUP = "SETUP";
const char *TAG_MAIN = "MAIN_STATE";
const char *TAG_CAN = "CAN_STATE";

// Millis timestamp holders
unsigned long millisPrint = 0;
unsigned long millisLed = 500;
unsigned long millisCAN = 0;

// Onboard LED state (HIGH = ON, LOW = OFF)
bool ledState = LOW;
bool configState = false;

// [4] ========================= SETUP =========================
void setup() {

  LOGI(TAG_SETUP, "Starting setup...");
  hardware_init();
  LOGI(TAG_SETUP, "Hardware initialized.");

  canbus.init();

  programState = StateMachine::STATE_REQUEST_ID; 
}

// [5] ========================= LOOP =========================
void loop() {
  
  switch (programState) {
    case StateMachine::STATE_REQUEST_ID:
      LOGI(TAG_MAIN, "State: Request ID");
      state_request_id();
      break;
    case StateMachine::STATE_CONFIGURATION:
      LOGI(TAG_MAIN, "State: Configuration");
      state_configuration();
      break;
    case StateMachine::STATE_MAIN:
      LOGI(TAG_MAIN, "State: Main");
      state_main();
      break;
  };


}

// [6] ============== FUNCTION DEFINITIONS ==============

void state_request_id() {
  uint8_t macAddress[6];
  char macStr[32]; // large enough buffer

  ret = esp_wifi_get_mac(WIFI_IF_STA, macAddress);
  if (ret != ESP_OK) {
    LOGE(TAG_CAN, "Error getting MAC address: %s", esp_err_to_name(ret));
  } else {
    snprintf(macStr, sizeof(macStr), "MAC:\n%02X:%02X:%02X:%02X:%02X:%02X", 
             macAddress[0], macAddress[1], macAddress[2],
             macAddress[3], macAddress[4], macAddress[5]);

    LOGI(TAG_CAN, "MAC address: %s", macStr);

    // Clear and print to display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Requesting ID...");
    display.println(macStr);
    display.display(); // push buffer to screen
  }

  ret = canbus.requestId(macAddress[0], macAddress[1], macAddress[2], 
                         macAddress[3], macAddress[4], macAddress[5]);
  if (ret != ESP_OK) {
    LOGE(TAG_CAN, "Error requesting ID: %s", esp_err_to_name(ret));
    return;
  } else {
    LOGI(TAG_CAN, "ID request successful.");
    LOGI(TAG_CAN, "Device ID: %d", canbus.getRxDeviceId());
  }

    display.println("ID received:");
    display.println(canbus.getRxDeviceId());
    display.display(); // push buffer to screen
    delay(5000); // Wait for 2 seconds to show ID
  programState = StateMachine::STATE_CONFIGURATION;
}

void state_configuration() {
  send_heartbeat();
  programState = StateMachine::STATE_MAIN; // Transition to next state
}

void state_main() {
  send_heartbeat();
  led_blink(); 
}

void send_heartbeat() {
  if(millis() - millisCAN >= CAN_HEARTBEAT) {
    millisCAN = millis();
    canbus.setMessageheartbeat(); 
    canbus.send(); 
  }
}

// Initialize hardware
void hardware_init() {
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.begin();


  // Setup LED
  pinMode(GPIO_PIN13, OUTPUT);

  // Setup I2C & display
  Wire.begin(21, 22);  // SDA = 21, SCL = 22
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true); // addr, reset


}

// Toggle onboard LED using non-blocking millis timing
void led_blink() {
  if (millis() - millisLed >= BLINK_INTERVAL) {
    millisLed = millis();
    ledState = !ledState;
    digitalWrite(GPIO_PIN13, ledState);
    LOGI(TAG_MAIN, "LED state: %s", ledState ? "ON" : "OFF");
        
  }
}

