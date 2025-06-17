#include <Arduino.h>

#include <WiFi.h>
#include <esp_wifi.h>

#include <pinout/devkitc.h>
#include <state_machine.h>
#include <logger.h>
#include <Wire.h>

#include <DHT.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
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
const unsigned int SCREEN_UPDATE_INTERVAL = 200;

// --- Button Pins ---
#define BTN_SELECT 32
#define BTN_DOWN   33
#define BTN_UP     25
#define LED_PIN    18

// [2] ============== FUNCTION DECLARATIONS ==============
void hardware_init();
void led_blink();

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

void can_send_heartbeat();
void can_send_sensor_data();

void led_set_color(uint8_t r, uint8_t g, uint8_t b); 
void update_sensor();

void button_handle_input();
void button_next_page();
// [3] ============== GLOBAL VARIABLES ==============

// State machine object
StateMachine programState;
spi_device_handle_t spiHandle;
MCP2515 mcp2515(&spiHandle); 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Canbus canbus(SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CS_PIN, GPIO_PIN6, spiHandle, mcp2515); // SCK, MOSI, MISO, CS, INT
DHT dht(GPIO_PIN33, DHT22); // Pin for DHT sensor

#define NUMPIXELS   1
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

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
unsigned long millisScreenUpdate = 0;

// Onboard LED state (HIGH = ON, LOW = OFF)
bool ledState = LOW;
bool configState = false;

float DHT_Humidity = 0.0;
float DHT_Temperature = 0.0;

bool isConfigured = false;
uint32_t statusStartTime = 0;

// --- Menu State ---
const char* menuItems[6] = {
  "Temp & Humidity", "Vibration", "Extreme Temperature",
  "Enviroment", "Settings", "Reset"
};
int selectedMenu = 0;
bool lastStateSelect = HIGH, lastStateUp = HIGH, lastStateDown = HIGH;


// [4] ========================= SETUP =========================
void setup() {

  LOGI(TAG_SETUP, "Starting setup...");
  hardware_init();
  LOGI(TAG_SETUP, "Hardware initialized.");

  canbus.init();

  programState = StateMachine::STATE_INTRO; 
}

// [5] ========================= LOOP =========================
void loop() {
  
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
      LOGI(TAG_MAIN, "State: Main");
      state_main();
      break;
  };


}

// [6] ============== FUNCTION DEFINITIONS ==============

void state_request_id() {
  button_handle_input();

  screen_draw_request_id();

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

  // display.println("Requesting ID...");
  // display.println(macStr);
  // display.display(); 

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

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(20, 60, "SUCCESS! ID:XX");
  u8g2.sendBuffer();

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
  can_send_heartbeat();
  programState = StateMachine::STATE_MAIN; // Transition to next state
}

void state_main() {
  can_send_heartbeat();
  led_blink(); 
  update_sensor();
  can_send_sensor_data();
}

void can_send_heartbeat() {
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

void can_send_sensor_data() {
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
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true); // addr, reset

  // Setup DHT sensor
  dht.begin();

  // Setup for display
  u8g2.begin();

  // Setup the navigation buttons
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
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

void button_handle_input() {
  bool selectPressed = !digitalRead(BTN_SELECT);
  bool upPressed = !digitalRead(BTN_UP);
  bool downPressed = !digitalRead(BTN_DOWN);

  unsigned long now = millis();

  if (selectPressed && lastStateSelect == false) {
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
      led_set_color(255, 255, 0); // Green for configuration
    } else {
      programState = StateMachine::STATE_CONFIGURATION;
      led_set_color(0, 255, 0); // Red for main
    }

    statusStartTime = millis();

  } else if (programState == StateMachine::STATE_CONFIGURATION) {
    programState = StateMachine::STATE_MAIN;
    led_set_color(255, 0, 255); // Purple for menu

  } else {
    programState = StateMachine::STATE_INTRO;
    led_set_color(0, 0, 255); // Back to blue
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
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(16, 63, "Furqon - Nauval - Zaqi");
  u8g2.sendBuffer();
}

void screen_draw_request_id() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 6, "A0:B1:C2:D3:E4:F5");
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(20, 27, "Requesting id...");

  unsigned long elapsed = (millis() - statusStartTime) / 1000;
  
  char buffer[32];
  u8g2.setFont(u8g2_font_6x10_tr);
  sprintf(buffer, "Elapsed Time: %lus", elapsed);
  u8g2.drawStr(15, 40, buffer);

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

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "SM-XX          A0:B1:C2:D3:E4:F5");

  u8g2.sendBuffer();

}


