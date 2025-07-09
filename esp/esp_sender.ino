#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "DHT20.h"
#include <Wire.h>
#include <NewPing.h>


#define raspiId 1


// ROTARY ENCODER CONFIG
#define ROTARY_ENCODER_A_PIN 19
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 16
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4


AiEsp32RotaryEncoder rotaryEncoder(
 ROTARY_ENCODER_A_PIN,
 ROTARY_ENCODER_B_PIN,
 ROTARY_ENCODER_BUTTON_PIN,
 ROTARY_ENCODER_VCC_PIN,
 ROTARY_ENCODER_STEPS
);


// DHT20 CONFIG
DHT20 DHT;


// KY-037 SOUND SENSOR
#define SOUND_SENSOR_PIN 35


// HC-SR04 CONFIG
#define TRIG_PIN 25
#define ECHO_PIN 26
#define MAX_DISTANCE 50
#define MIN_VALID_DISTANCE 3
#define MAX_VALID_DISTANCE 50


NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
uint8_t ultrasonic = 0;


// ISR for rotary encoder
void IRAM_ATTR readEncoderISR() {
 rotaryEncoder.readEncoder_ISR();
}


// Setup
void setup() {
 Serial.begin(115200);
 Wire.begin();  // I2C for DHT20
 DHT.begin();


 rotaryEncoder.begin();
 rotaryEncoder.setup(readEncoderISR);
 rotaryEncoder.setBoundaries(0, 1000, false);
 rotaryEncoder.setAcceleration(250);


 pinMode(SOUND_SENSOR_PIN, INPUT);


 // Tidak perlu print CSV header lagi
}


// Ultrasonic reading with median filter
void readUltrasonic() {
 const int samples = 5;
 uint8_t readings[samples];


 for (int i = 0; i < samples; i++) {
   delay(50);
   readings[i] = sonar.ping_cm();
 }


 // Sort for median
 for (int i = 0; i < samples - 1; i++) {
   for (int j = i + 1; j < samples; j++) {
     if (readings[i] > readings[j]) {
       uint8_t temp = readings[i];
       readings[i] = readings[j];
       readings[j] = temp;
     }
   }
 }


 ultrasonic = readings[samples / 2];
 if (ultrasonic < MIN_VALID_DISTANCE || ultrasonic > MAX_VALID_DISTANCE) {
   ultrasonic = 0;
 }
}


// Loop
void loop() {
 static unsigned long lastRead = 0;
 if (millis() - lastRead >= 1000) {
   lastRead = millis();


   // Read sensors
   DHT.read();
   float temp = DHT.getTemperature();
   float hum = DHT.getHumidity();
   int sound = digitalRead(SOUND_SENSOR_PIN) == LOW ? 1 : 0;
   readUltrasonic();
   int rotaryVal = rotaryEncoder.readEncoder();


   // Kirim data sebagai JSON satu baris
   Serial.print("{\"raspiID\":");
   Serial.print(raspiId);
   Serial.print(",\"temperature\":");
   Serial.print(temp, 1);
   Serial.print(",\"humidity\":");
   Serial.print(hum, 1);
   Serial.print(",\"sound\":");
   Serial.print(sound);
   Serial.print(",\"distance_cm\":");
   Serial.print(ultrasonic);
   Serial.print(",\"rotary_value\":");
   Serial.print(rotaryVal);
   Serial.println("}");
 }
}





