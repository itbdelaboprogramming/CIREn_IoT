#include <Arduino.h>
#include <pinout/devkitv1.h>

// Constants
const unsigned long BLINK_INTERVAL = 500;

// Globals for non-blocking LED
unsigned long previousMillis = 0;
bool ledState = LOW;

// === Function declarations ===
void hardware_init();
void led_blink();

// === Setup ===
void setup() {
  hardware_init();
}

// === Loop ===
void loop() {
  led_blink();
  Serial.println("Hello, I'm ESP32 Devkit V1 built using Platform.io");
}

// === Definitions ===
void hardware_init() {

  // Setup serial communication
  Serial.begin(115200);

  // Setup onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void led_blink() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= BLINK_INTERVAL) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
}
