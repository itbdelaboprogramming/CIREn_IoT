

/**！
 * @file getAcceleration.ino
 * @brief Get the acceleration in the three directions of xyz, the range can be ±6g, ±12g or ±24g
 * @n When using SPI, chip select pin can be modified by changing the value of LIS331HH_CS
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2021-01-16
   * @url https://github.com/DFRobot/DFRobot_LIS
 */

#include <DFRobot_LIS.h>
#include <WiFi.h>
#include <esp_now.h>


//When using I2C communication, use the following program to construct an object by DFRobot_LIS331HH_I2C
/*!
 * @brief Constructor 
   * @param pWire I2c controller
 * @param addr  I2C address(0x18/0x19)
 */
DFRobot_LIS331HH_I2C acce(&Wire,0x18);
// DFRobot_LIS331HH_I2C acce;

//When using SPI communication, use the following program to construct an object by DFRobot_LIS331HH_SPI
#if defined(ESP32) || defined(ESP8266)
#define LIS331HH_CS  D3
#elif defined(__AVR__) || defined(ARDUINO_SAM_ZERO)
#define LIS331HH_CS 3
#elif (defined NRF5)
#define LIS331HH_CS 2  //The pin on the development board with the corresponding silkscreen printed as P2
#endif
/*!
 * @brief Constructor 
 * @param cs : Chip selection pinChip selection pin
 * @param spi :SPI controller
 */
//DFRobot_LIS331HH_SPI acce(/*cs = */LIS331HH_CS,&SPI);
//DFRobot_LIS331HH_SPI acce(/*cs = */LIS331HH_CS);

// Structure to send data in one packet
typedef struct __attribute__((packed)) {
  int16_t ax;  // acceleration in mg
  int16_t ay;
  int16_t az;
} AccelPacket;

// Create a global packet
AccelPacket accelData;

// Peer MAC address (replace with your receiver ESP32’s MAC)
// # D8:13:2A:F0:99:CC
uint8_t peerMac[] = {0xD8, 0x13, 0x2A, 0xF0, 0x99, 0xCC};  



// Callback when data is sent
// New ESP-NOW callback signature for ESP-IDF v5.x
void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  char macStr[18];
//  snprintf(macStr, sizeof(macStr),
//           "%02X:%02X:%02X:%02X:%02X:%02X",
//           info->des_addr[0], info->des_addr[1], info->des_addr[2],
//           info->des_addr[3], info->des_addr[4], info->des_addr[5]);
//
//  Serial.print("Last Packet Sent to: ");
//  Serial.print(macStr);
//  Serial.print(" Status: ");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success Sending Packet" : "Failed Sending Packet");
}


// Modular function to send acceleration data
void sendAccelData(int16_t ax, int16_t ay, int16_t az) {
  accelData.ax = ax;
  accelData.ay = ay;
  accelData.az = az;

  esp_err_t result = esp_now_send(peerMac, (uint8_t *)&accelData, sizeof(accelData));

  if (result == ESP_OK) {
    Serial.println("Data queued for sending...");
  } else {
    Serial.print("Error sending data: ");
    Serial.println(result);
  }
}



void setup(void){

  Serial.begin(115200);
  //Chip initialization
  while(!acce.begin()){
     delay(1000);
     Serial.println("Initialization failed, please check the connection and I2C address setting");
    }
  //Get chip id
  Serial.print("chip id : ");
  Serial.println(acce.getID(),HEX);
  
  /**
    set range:Range(g)
              eLis331hh_6g = 6,/<±6g>/
              eLis331hh_12g = 12,/<±12g>/
              eLis331hh_24g = 24/<±24g>/
  */
  acce.setRange(/*range = */DFRobot_LIS::eLis331hh_6g);
  /**
    Set data measurement rate：
      ePowerDown_0HZ = 0,
      eLowPower_halfHZ,
      eLowPower_1HZ,
      eLowPower_2HZ,
      eLowPower_5HZ,
      eLowPower_10HZ,
      eNormal_50HZ,
      eNormal_100HZ,
      eNormal_400HZ,
      eNormal_1000HZ,
  */
  acce.setAcquireRate(/*rate = */DFRobot_LIS::eNormal_1000HZ);
//  Serial.print("Acceleration:\n");

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW initialized.");
  delay(1000);
}

void loop(void){

    //Get the acceleration in the three directions of xyz
    //The mearsurement range can be ±6g, ±12g or ±24g, set by the setRange() function
    long ax,ay,az;
    ax = acce.readAccX();//Get the acceleration in the x direction
    ay = acce.readAccY();//Get the acceleration in the y direction
    az = acce.readAccZ();//Get the acceleration in the z direction
    
    //    Serial.print("x: "); //print acceleration
    //    Serial.print(ax);
    //    Serial.print(" mg \ty: ");
    //    Serial.print(ay);
    //    Serial.print(" mg \tz: ");
    //    Serial.print(az);
    //    Serial.println(" mg");
    
    sendAccelData(ax, ay, az);
    delay(20);
}
