/**
 * @file Canbus.h
 * @date 2025-02-05
 * @class Canbus
 * @brief This class provides an interface for CAN bus communication on STM32 microcontrollers.
 * 
 * The Canbus class inherits from CanbusInterface and implements methods for initializing the CAN peripheral,
 * sending and receiving messages, setting filters, and handling various types of data messages.
 */

 #pragma once

#include <Arduino.h>
#include "mcp2515.h"
#include "../spi_api/ESP32_SPI_API.h"
#include <SPI.h>
#include <logger.h>




 struct CANHeader {
    uint16_t standardId = 0;
    uint32_t extendedId = 0;
    bool isExtended = false;
    uint8_t dlc = 8;
    bool rtr = false;
};


enum SensorId {

};

/**
 * This object implementation done with the assumption of 
 * only one SPI device on the bus
**/

 class Canbus {
    public:
        Canbus(int SCK_PIN,int MOSI_PIN,int MISO_PIN,int CS_PIN,int INT_PIN, spi_device_handle_t& spiHandle, MCP2515& mcp2515)
        : SCK_PIN(SCK_PIN), MOSI_PIN(MOSI_PIN), MISO_PIN(MISO_PIN), CS_PIN(CS_PIN), INT_PIN(INT_PIN), spiHandle(spiHandle), mcp2515(mcp2515) {
            this->baudRate = 500000; // Default baud rate
        }
        int init();        
        int send();
    
        void setStandardId(uint16_t deviceId, uint16_t SensorId);
        void setExtendedId(uint16_t deviceId, uint16_t SensorId);
        void setDeviceId(uint16_t deviceId);

        int requestId(uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5);
    
        int filterMessageByDeviceId(uint16_t deviceId);
        int filterMessageBySensorId(uint16_t SensorId);
        
        void setMessageheartbeat();

        void setMessageFloat(float data);
        void setMessageTwoFloat(float data1, float data2);
        void setMessageDouble(double data);

        void setMessageInt8(int8_t data);
        void setMessageInt8Two(int8_t data1, int8_t data2);
        void setMessageInt8Three(int8_t data1, int8_t data2, int8_t data3);
        void setMessageInt8Four(int8_t data1, int8_t data2, int8_t data3, int8_t data4);
        

        void setMessageInt16(int16_t data);
        void setMessageInt16Two(int16_t data1, int16_t data2);
        void setMessageInt16Three(int16_t data1, int16_t data2, int16_t data3);
        void setMessageInt16Four(int16_t data1, int16_t data2, int16_t data3, int16_t data4);

        void setMessageInt32(int32_t data);
        void setMessageInt32Two(int32_t data1, int32_t data2);

        void setMessageUint8(uint8_t data);
        void setMessageUint8Two(uint8_t data1, uint8_t data2);
        void setMessageUint8Three(uint8_t data1, uint8_t data2, uint8_t data3);
        void setMessageUint8Four(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
        void setMessageUint8Five(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5);
        void setMessageUint8Six(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6);
        void setMessageUint8Seven(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);
        void setMessageUint8Eight(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8);

        void setMessageUint16(uint16_t data);
        void setMessageUint16Two(uint16_t data1, uint16_t data2);
        void setMessageUint16Three(uint16_t data1, uint16_t data2, uint16_t data3);
        void setMessageUint16Four(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
        
        void setMessageUint32(uint32_t data);
        void setMessageUint32Two(uint32_t data1, uint32_t data2);

        int getDeviceId();
        // int getRxSensorId();
        // int getRxheaderStdId();
    
        // void getMessageRaw(uint8_t* data);
        // void rxCallback(int packetSize);
    
    private:
        int rxPin;
        int txPin;
        int baudRate;

        int SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN, INT_PIN;

        uint8_t macAddress[6];
    
        uint8_t rxMailbox[8]; 
        uint8_t txMailbox[8];
    
        uint16_t deviceId;
    
        CANHeader txHeader;
        CANHeader rxHeader;

        spi_device_handle_t& spiHandle;
        MCP2515& mcp2515;
    
        uint32_t calculateCrc(uint8_t* data, int length);
        void configureFilter();
        static void rxCallback(int packetSize);
    };