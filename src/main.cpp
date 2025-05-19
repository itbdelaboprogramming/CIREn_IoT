#include <Arduino.h>

#include <WiFi.h>
#include <esp_wifi.h>

#include <pinout/devkitc.h>
#include <state_machine.h>
#include <logger.h>
#include <ESP_SSD1306.h>
#include <Wire.h>

#include <DHT.h>
#include <drivers/canbus/Canbus.h>
#include <canbus_id.h>

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
const unsigned int CAN_HEARTBEAT = 100;

// [2] ============== FUNCTION DECLARATIONS ==============
void hardware_init();
void led_blink();

void state_configuration();
void state_main();
void state_request_id();

void send_heartbeat();
void send_sensor_data();
void update_sensor();
// [3] ============== GLOBAL VARIABLES ==============

// State machine object
StateMachine programState;
ESP_SSD1306 display(-1); // I2C pins for ESP32 DevKitC

spi_device_handle_t spiHandle;
MCP2515 mcp2515(&spiHandle); 
Canbus canbus(SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CS_PIN, GPIO_PIN6, spiHandle, mcp2515); // SCK, MOSI, MISO, CS, INT

DHT dht(GPIO_PIN33, DHT22); // Pin for DHT sensor

esp_err_t ret;

// Logger TAG
const char *TAG_SETUP = "SETUP";
const char *TAG_MAIN = "MAIN_STATE";
const char *TAG_CAN = "CAN_STATE";

// Millis timestamp holders
unsigned long millisPrint = 0;
unsigned long millisLed = 0;
unsigned long millisCAN = 0;
unsigned long millisSensor = 0;
unsigned long millisSendSensorData = 0;

// Onboard LED state (HIGH = ON, LOW = OFF)
bool ledState = LOW;
bool configState = false;

float DHT_Humidity = 0.0;
float DHT_Temperature = 0.0;


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

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  uint8_t macAddress[6];
  char macStr[32]; // large enough buffer

  ret = esp_wifi_get_mac(WIFI_IF_STA, macAddress);
  if (ret != ESP_OK) {
    LOGE(TAG_CAN, "Error getting MAC address: %s", esp_err_to_name(ret));
  } 

  snprintf(macStr, sizeof(macStr), "MAC:\n%02X:%02X:%02X:%02X:%02X:%02X", 
            macAddress[0], macAddress[1], macAddress[2],
            macAddress[3], macAddress[4], macAddress[5]);
  LOGI(TAG_CAN, "MAC address: %s", macStr);

  display.println("Requesting ID...");
  display.println(macStr);
  display.display(); 

  ret = canbus.requestId(macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
  if (ret != ESP_OK) {
    LOGE(TAG_CAN, "Error requesting ID: %s", esp_err_to_name(ret));
    return;
  }

  LOGI(TAG_CAN, "ID request successful.");
  LOGI(TAG_CAN, "Device ID: %d", canbus.getDeviceId());

  ret = canbus.filterMessageByDeviceId(canbus.getDeviceId());
  if (ret != ESP_OK) {
    LOGE(TAG_CAN, "Error setting filter for ID: %s", esp_err_to_name(ret));
    return;
  }

  display.println("ID received:");
  display.println(canbus.getDeviceId());
  display.display();

  delay(5000); 

  programState = StateMachine::STATE_CONFIGURATION;
}

void state_configuration() {
  send_heartbeat();
  programState = StateMachine::STATE_MAIN; // Transition to next state
}

void state_main() {
  send_heartbeat();
  led_blink(); 
  update_sensor();
  send_sensor_data();
}

void send_heartbeat() {
  if(millis() - millisCAN >= CAN_HEARTBEAT) {
    millisCAN = millis();
    canbus.setMessageheartbeat(); 
    canbus.send(); 
  }
}

void update_sensor() {

  // Update DHT sensor
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

    LOGI(TAG_MAIN, "Humidity: %.2f%%, Temperature: %.2f°C", DHT_Humidity, DHT_Temperature);

  }

}

void send_sensor_data() {
  if(millis() - millisSendSensorData >= SENSOR_INTERVAL) {
    millisSendSensorData = millis();

    canbus.setExtendedId(canbus.getDeviceId(), sensorId::DHT22, dhtDataId::HUMIDITY);
    canbus.setMessageFloat(DHT_Humidity);
    canbus.send();
    LOGI(TAG_MAIN, "Humidity sent: %.2f%%", DHT_Humidity);

    canbus.setExtendedId(canbus.getDeviceId(), sensorId::DHT22, dhtDataId::TEMPERATURE);
    canbus.setMessageFloat(DHT_Temperature);
    canbus.send();
    LOGI(TAG_MAIN, "Temperature sent: %.2f°C", DHT_Temperature);
  }
}

// Initialize hardware
void hardware_init() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin();

  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup I2C & display
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true); // addr, reset

  // Setup DHT sensor
  dht.begin();
}

// Toggle onboard LED using non-blocking millis timing
void led_blink() {
  if (millis() - millisLed >= BLINK_INTERVAL) {
    millisLed = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    LOGI(TAG_MAIN, "LED state: %s", ledState ? "ON" : "OFF");  
  }
}

