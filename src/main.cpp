#include <Arduino.h>
#include <pinout/devkitv1.h>
#include <drivers/temperature/DHTSensor.h>
#include <drivers/proximity/TCRT5000Sensor.h>
#include <drivers/ultrasonic/UltrasonicSensor.h>

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
const unsigned int DHT_UPDATE_INTERVAL = 2000;
const unsigned int PROXIMITY_UPDATE_INTERVAL = 200;
const unsigned int ULTRASONIC_UPDATE_INTERVAL = 1000;

// [2] ============== FUNCTION DECLARATIONS ==============
void hardware_init();
void led_blink();
void update_temperature();
void update_proximity();
void update_ultrasonic();

// [3] ============== GLOBAL VARIABLES ==============
// Millis timestamp holders
unsigned long millisPrint = 0;
unsigned long millisLed = 0;
unsigned long millisDHT = 0;
unsigned long millisProximity = 0;
unsigned long millisUltrasonic = 0;

// Onboard LED state (HIGH = ON, LOW = OFF)
bool ledState = LOW;

// IR proximity sensor object
TCRT5000Sensor proximitySensor(PIN_GPIO34);
bool proximityState = false;

// DHT sensor object and values
DHTSensor dhtSensor(PIN_GPIO4, DHT22);
float temperature = 0.0;
float humidity = 0.0;

// Ultrasonic sensor object
UltrasonicSensor ultrasonicSensor(PIN_GPIO12, PIN_GPIO13);
float distance = 0.0; // Distance in meters

// [4] ========================= SETUP =========================
void setup() {
  hardware_init();
}

// [5] ========================= LOOP =========================
void loop() {
  
  // Print message every PRINT_INTERVAL ms
  if (millis() - millisPrint >= PRINT_INTERVAL) {
    millisPrint = millis();
    Serial.println("Hello, I'm ESP32 Devkit V1 built using Platform.io");
  }
  
  led_blink(); 
  update_temperature();
  update_proximity();
  update_ultrasonic();

}

// [6] ============== FUNCTION DEFINITIONS ==============

// Initialize hardware
void hardware_init() {
  Serial.begin(115200);
  dhtSensor.begin();
  pinMode(LED_BUILTIN, OUTPUT);
}

// Toggle onboard LED using non-blocking millis timing
void led_blink() {
  if (millis() - millisLed >= BLINK_INTERVAL) {
    millisLed = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
}

// Read from DHT sensor if enough time has passed
void update_temperature() {
  if (millis() - millisDHT >= DHT_UPDATE_INTERVAL) {
    millisDHT = millis();
    temperature = dhtSensor.readTemperature(); 
    humidity = dhtSensor.readHumidity();
  }
}

// Read from proximity sensor if enough time has passed
void update_proximity() {
  if (millis() - millisProximity >= PROXIMITY_UPDATE_INTERVAL) {
    millisProximity = millis();
    proximityState = proximitySensor.isClose();
  }
}

// Read from ultrasonic sensor if enough time has passed
void update_ultrasonic() {
  if (millis() - millisUltrasonic >= ULTRASONIC_UPDATE_INTERVAL) {
    millisUltrasonic = millis();
    distance = ultrasonicSensor.getDistanceMeters();
  }
}