#include <CAN.h>

#define TX_GPIO_NUM   5
#define RX_GPIO_NUM   4

void setup() {
  Serial.begin (115200);
  while (!Serial);
  delay (1000);

  Serial.println ("CAN Receiver");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    uint8_t data[8];
    for (int i = 0; i < packetSize && i < 8; i++) {
      data[i] = CAN.read();
    }

    // Parse data
    uint8_t id = data[0];
    uint8_t ultrasonic = data[1];
    int8_t temp = data[2];
    uint8_t hum = data[3];
    uint16_t tcrt = (data[4] << 8) | data[5];
    uint16_t heading = (data[6] << 8) | data[7];

    // Print
    Serial.println(" ------------------------------------------------ ");
    Serial.print("🆔 ID: "); Serial.println(id);
    Serial.print("📏 Ultrasonic: "); Serial.print(ultrasonic); Serial.println(" cm");
    Serial.print("🌡️ Temp: "); Serial.print(temp); Serial.println(" °C");
    Serial.print("💧 Hum: "); Serial.print(hum); Serial.println(" %");
    Serial.print("📶 TCRT: "); Serial.println(tcrt);
    Serial.print("🧭 Heading: "); Serial.println(heading / 10.0, 1);
  }
}
