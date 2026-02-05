
// /*
//  * MASTER IMPLEMENTATION EXAMPLE
//  *
//  * To use this device as a master instead of slave, replace the code above with:
//  *
//  */

// #include <Arduino.h>
// #include "drivers/ciren_comm/ciren_comm.h"
// uint8_t slave1Mac[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56};
// uint8_t slave2Mac[] = {0x24, 0x0A, 0xC4, 0x78, 0x9A, 0xBC};
// CIREnMaster master;

// void setup() {
//     Serial.begin(115200);
//     Serial.println("CIRen Master Starting...");

//     if (master.init() == ESP_OK) {
//         Serial.println("Master initialized");

//         if (master.registerSlave(slave1Mac) == ESP_OK) {
//             Serial.println("Slave 1 registered");
//         }

//         if (master.registerSlave(slave2Mac) == ESP_OK) {
//             Serial.println("Slave 2 registered");
//         }
        
//         master.slaveTurnOn(slave1Mac);
//         master.slaveTurnOn(slave2Mac);
//     } 
    
//     else {
//         Serial.println("Master init failed");
//     }
// }
// void loop() {
//     delay(10);
// }
