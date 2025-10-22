#include <Arduino.h>

#include <WiFi.h>

#include <pinout/devkitc.h>
#include <state_machine.h>
#include <logger.h>
#include <Wire.h>

#include <DHT.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <max6675.h>
#include <DFRobot_BNO055.h>

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

// ===== DEBUG MODE CONFIGURATION =====
// Set to true to enable full debug logging
// Set to false to only print current menu data
#define DEBUG_MODE false
// ====================================

// Timing intervals (in milliseconds)
const unsigned int PRINT_INTERVAL = 1000;
const unsigned int BLINK_INTERVAL = 500;
const unsigned int SENSOR_INTERVAL = 1000;
const unsigned int SCREEN_UPDATE_INTERVAL = 200;
const unsigned int DATA_PRINT_INTERVAL = 1000;

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

typedef DFRobot_BNO055_IIC    BNO;

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
void print_current_menu_data();

void button_handle_input();
void button_next_page();

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

// Logger TAG
const char *TAG_SETUP = "SETUP";
const char *TAG_MAIN = "MAIN_STATE";

BNO   bno(&Wire, 0x28);
BNO::sEulAnalog_t sEul;
BNO::sAxisAnalog_t sAcc;

// Millis timestamp holders
unsigned long millisPrint = 0;
unsigned long millisLed = 0;
unsigned long millisSensor = 0;
unsigned long millisScreenUpdate = 0;
unsigned long millisDataPrint = 0;

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
  "Temp & Humidity", "Acceleration", "Extreme Temperature",
  "Enviroment", "Settings", "Reset"
};
int selectedMenu = 0;
bool lastStateSelect = HIGH, lastStateUp = HIGH, lastStateDown = HIGH;
char macStr[32]; // Buffer for MAC address string


// [4] ========================= SETUP =========================
void setup() {

  Serial.begin(115200);
  
  if (DEBUG_MODE) {
    LOGI(TAG_SETUP, "Starting setup...");
  }
  
  hardware_init();
  
  if (DEBUG_MODE) {
    LOGI(TAG_SETUP, "Hardware initialized.");
  }
  
  programState = StateMachine::STATE_INTRO; 
}

// [5] ========================= LOOP =========================
void loop() {
  switch (programState) {
    case StateMachine::STATE_INTRO:
      if (DEBUG_MODE) {
        LOGI(TAG_MAIN, "State: Intro");
      }
      state_introduction();
      break;
    case StateMachine::STATE_REQUEST_ID:
      if (DEBUG_MODE) {
        LOGI(TAG_MAIN, "State: Request ID");
      }
      state_request_id();
      break;
    case StateMachine::STATE_CONFIGURATION:
      if (DEBUG_MODE) {
        LOGI(TAG_MAIN, "State: Configuration");
      }
      state_configuration();
      break;
    case StateMachine::STATE_MAIN:
      state_main();
      break;
  };
}

// [6] ============== FUNCTION DEFINITIONS ==============

void state_request_id() {
  button_handle_input();

  ret = esp_read_mac(macAddress, ESP_MAC_WIFI_STA);
  if (ret != ESP_OK) {
    if (DEBUG_MODE) {
      LOGE(TAG_MAIN, "Error getting MAC address: %s", esp_err_to_name(ret));
    }
  } 
  
  snprintf(macStr, sizeof(macStr), "MAC:\n%02X:%02X:%02X:%02X:%02X:%02X", 
  macAddress[0], macAddress[1], macAddress[2],
  macAddress[3], macAddress[4], macAddress[5]);
  
  if (DEBUG_MODE) {
    LOGI(TAG_MAIN, "MAC address: %s", macStr);
  }
  
  screen_draw_request_id();

  delay(5000); 

  programState = StateMachine::STATE_CONFIGURATION;
}

void state_introduction() {
  button_handle_input();

  if(millis() - millisScreenUpdate >= SCREEN_UPDATE_INTERVAL){
    screen_draw_intro();
  }

  if (DEBUG_MODE) {
    if(millis() - millisPrint >= PRINT_INTERVAL) {
      millisPrint = millis();
      LOGI(TAG_MAIN, "In introduction state.");
    }
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
        if (DEBUG_MODE) {
          LOGI(TAG_MAIN,"[MENU_1]");
        }
        screen_draw_main_temperature();
        break;
      case StateMainMenu::STATE_VIBRATION:
        if (DEBUG_MODE) {
          LOGI(TAG_MAIN,"[MENU_2]");
        }
        screen_draw_main_vibration();
        break;
      case StateMainMenu::STATE_EXTREME_TEMP:
        if (DEBUG_MODE) {
          LOGI(TAG_MAIN,"[MENU_3]");
        }
        screen_draw_main_extreme_temperature();
        break;
      case StateMainMenu::STATE_ENVIRONMENT:
        if (DEBUG_MODE) {
          LOGI(TAG_MAIN,"[MENU_4]");
        }
        screen_draw_main_environment();
        break;
      case StateMainMenu::STATE_SETTINGS:
        if (DEBUG_MODE) {
          LOGI(TAG_MAIN,"[MENU_5]");
        }
        screen_draw_main_settings();
        break;
      case StateMainMenu::STATE_RESET:
        if (DEBUG_MODE) {
          LOGI(TAG_MAIN,"[MENU_6]");
        }
        screen_draw_main_reset();
        break;
    }
    millisScreenUpdate = millis();
  }

  update_sensor();
  
  // Print current menu data based on mode
  if (!DEBUG_MODE) {
    print_current_menu_data();
  }
}

void update_sensor() {
  if(millis() - millisSensor >= SENSOR_INTERVAL) {
    millisSensor = millis();

    DHT_Humidity = dht.readHumidity();
    ret = isnan(DHT_Humidity);
    if (ret) {
      if (DEBUG_MODE) {
        LOGE(TAG_MAIN, "Failed to read humidity from DHT sensor.");
      }
      return;
    }

    DHT_Temperature = dht.readTemperature();
    ret = isnan(DHT_Temperature);
    if (ret) {
      if (DEBUG_MODE) {
        LOGE(TAG_MAIN, "Failed to read temperature from DHT sensor.");
      }
      return;
    }

    MAX6675_Temperature = max6675.readCelsius();
    ret = isnan(MAX6675_Temperature);
    if (ret) {
      if (DEBUG_MODE) {
        LOGE(TAG_MAIN, "Failed to read temperature from MAX6675 sensor.");
      }
      return;
    }

    sEul = bno.getEul();
    sAcc = bno.getAxis(BNO::eAxisAcc);

    if (DEBUG_MODE) {
      LOGI(TAG_MAIN, "Humidity: %.2f%%, Temperature: %.2f°C, MAX6675: %.2f°C", 
           DHT_Humidity, DHT_Temperature, MAX6675_Temperature);
    }
  }
}

void print_current_menu_data() {
  if(millis() - millisDataPrint >= DATA_PRINT_INTERVAL) {
    millisDataPrint = millis();
    
    switch (mainMenuState) {
      case StateMainMenu::STATE_TEMP_HUMIDITY:
        Serial.print("Temp: ");
        Serial.print(DHT_Temperature);
        Serial.print("°C, Humidity: ");
        Serial.print(DHT_Humidity);
        Serial.println("%");
        break;
        
      case StateMainMenu::STATE_VIBRATION:
        Serial.print("Ax: ");
        Serial.print(sAcc.x);
        Serial.print(", Ay: ");
        Serial.print(sAcc.y);
        Serial.print(", Az: ");
        Serial.println(sAcc.z);
        break;
        
      case StateMainMenu::STATE_EXTREME_TEMP:
        Serial.print("Extreme Temp: ");
        Serial.print(MAX6675_Temperature);
        Serial.println("°C");
        break;
        
      case StateMainMenu::STATE_ENVIRONMENT:
        Serial.println("Environment: No data available");
        break;
        
      case StateMainMenu::STATE_SETTINGS:
        Serial.println("Settings: No data available");
        break;
        
      case StateMainMenu::STATE_RESET:
        Serial.println("Reset: Standby");
        break;
    }
  }
}

// Initialize hardware
void hardware_init() {
  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup I2C & display
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  

  // Setup DHT sensor
  dht.begin();

  // Setup for display
  u8g2.begin();

  // Setup BNO055
  bno.reset();
  while(bno.begin() != BNO::eStatusOK) {
    if (DEBUG_MODE) {
      Serial.println("bno begin failed");
    }
    delay(2000);
  }

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

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 55, "Standalone Mode");
  
  if (DEBUG_MODE) {
    u8g2.drawStr(0, 63, "Debug: ON");
  } else {
    u8g2.drawStr(0, 63, "Debug: OFF");
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
  u8g2.drawStr(20, 20, "Device ID");

  unsigned long elapsed = (millis() - statusStartTime) / 1000;
  
  char buffer_time[32];
  u8g2.setFont(u8g2_font_6x10_tr);
  sprintf(buffer_time, "Elapsed: %lus", elapsed);
  u8g2.drawStr(15, 35, buffer_time);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "Status: Ready");

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
  u8g2.drawStr(0, 63, "Status: OK");

  u8g2.sendBuffer();
}

void screen_draw_main_temperature() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  char buffer_temperature[32];
  snprintf(buffer_temperature, sizeof(buffer_temperature), "Humidity: %.2f%%", DHT_Humidity);
  char buffer_humidity[32];
  snprintf(buffer_humidity, sizeof(buffer_humidity), "Temperature: %.2fC", DHT_Temperature);


  u8g2.drawStr(0, 10, buffer_humidity);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, buffer_temperature);

  char buffer_mac[32];
  snprintf(buffer_mac, sizeof(buffer_mac), "Device: %02X:%02X:%02X",
           macAddress[3], macAddress[4], macAddress[5]);
  u8g2.drawStr(0, 30, buffer_mac);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "Status: Active");

  u8g2.sendBuffer();
}

void screen_draw_main_vibration() {
  u8g2.clearBuffer();

  char buffer_x_axis[32];
  snprintf(buffer_x_axis, sizeof(buffer_x_axis), "X-Axis: %.2f", sAcc.x);
  char buffer_y_axis[32];
  snprintf(buffer_y_axis, sizeof(buffer_y_axis), "Y-Axis: %.2f", sAcc.y);
  char buffer_z_axis[32];
  snprintf(buffer_z_axis, sizeof(buffer_z_axis), "Z-Axis: %.2f", sAcc.z);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 10, buffer_x_axis);
  u8g2.drawStr(0, 20, buffer_y_axis);
  u8g2.drawStr(0, 30, buffer_z_axis);

  u8g2.drawStr(0, 63, "Status: Monitoring");

  u8g2.sendBuffer();
}

void screen_draw_main_environment() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "Environment");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, "No data available");

  u8g2.drawStr(0, 63, "Status: OK");

  u8g2.sendBuffer();
}

void screen_draw_main_settings() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "Settings");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, "No settings available");

  u8g2.drawStr(0, 63, "Status: OK");

  u8g2.sendBuffer();
}

void screen_draw_main_reset() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "Reset");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 20, "Press to reset device");

  u8g2.drawStr(0, 63, "Status: Standby");

  u8g2.sendBuffer();
}

void screen_draw_main_extreme_temperature() {
  u8g2.clearBuffer();

  char buffer_extreme_temp[32];
  snprintf(buffer_extreme_temp, sizeof(buffer_extreme_temp), "Extreme Temp: %.2fC", MAX6675_Temperature);

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, buffer_extreme_temp);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "Status: Monitoring");

  u8g2.sendBuffer();
}