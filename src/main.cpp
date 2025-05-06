#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <pinout/devkitc.h>
#include <state_machine.h>
#include <logger.h>
#include <ESP_SSD1306.h>
#include <Wire.h>

#include <drivers/canbus/canbus.h>



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
const unsigned int CAN_INTERVAL = 100;


// [2] ============== FUNCTION DECLARATIONS ==============
void hardware_init();
void led_blink();

void state_configuration();
void state_main();
void state_request_id();

// [3] ============== GLOBAL VARIABLES ==============

// State machine object
StateMachine programState;
ESP_SSD1306 display(-1); // I2C pins for ESP32 DevKitC

spi_device_handle_t spiHandle;
MCP2515 mcp2515(&spiHandle); 

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


// [4] ========================= SETUP =========================
void setup() {

  LOGI(TAG_SETUP, "Starting setup...");
  hardware_init();
  LOGI(TAG_SETUP, "Hardware initialized.");


  ret = fInitializeSPI_Channel(SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, VSPI_HOST, true);
  if (ret != ESP_OK) {
    LOGE(TAG_SETUP, "Error initializing SPI channel: %s", esp_err_to_name(ret));
    return;
  }
  
  ret = fInitializeSPI_Devices(VSPI_HOST, spiHandle, SPI_CS_PIN);
  if (ret != ESP_OK) {
    LOGE(TAG_SETUP, "Error initializing SPI device: %s", esp_err_to_name(ret));
    return;
  }

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

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
        // Request ID from CAN bus    
      struct can_frame frame;
      frame.can_id = 0x001;
      frame.can_dlc = 4;
      frame.data[0] = 0x01; // Request ID command
      frame.data[1] = 0x02; // Request ID command
      frame.data[2] = 0x03; // Request ID command
      frame.data[3] = 0x04; // Request ID command

      ret = mcp2515.sendMessage(&frame);
      if (ret != ESP_OK) {
        LOGE(TAG_MAIN, "Error sending CAN message: %s", esp_err_to_name(ret));
      } else {
        LOGI(TAG_MAIN, "CAN message sent successfully.");
      }
      state_main();
      break;
  };


}

// [6] ============== FUNCTION DEFINITIONS ==============

void state_request_id() {

  // Request ID from CAN bus    
  struct can_frame frame;
  frame.can_id = 0x001;
  frame.can_dlc = 4;
  frame.data[0] = 0x01; // Request ID command
  frame.data[1] = 0x02; // Request ID command
  frame.data[2] = 0x03; // Request ID command
  frame.data[3] = 0x04; // Request ID command

  mcp2515.sendMessage(&frame);

  programState = StateMachine::STATE_CONFIGURATION; // Transition to next state
}

void state_configuration() {
  programState = StateMachine::STATE_MAIN; // Transition to next state
}

void state_main() {
  led_blink(); 
}

// Initialize hardware
void hardware_init() {
  Serial.begin(9600);

  // Setup LED
  pinMode(GPIO_PIN13, OUTPUT);

  // Setup I2C & display
  // Wire.begin(21, 22);  // SDA = 21, SCL = 22
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true); // addr, reset


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

