#include <CAN.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!CAN.begin(500E3)) {
    Serial.println("CAN init failed");
    while (1);
  }

  Serial.println("CAN Sender Ready");
}

void loop() {
  CAN.beginPacket(0x123);  // CAN ID
  CAN.write('H');
  CAN.write('i');
  CAN.endPacket();
  delay(1000);
}
