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
    
        void filterMessageByDeviceId(uint16_t deviceId);
        void filterMessageBySensorId(uint16_t SensorId);
        void filterMessageById(uint16_t id);
        void filterMessageForIdRequest(uint8_t* macAddress);
        
        void setMessageheartbeat();
    
        int getRxDeviceId();
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
    
        uint16_t calculateCrc(uint8_t* data, int length);
        void configureFilter();
        static void rxCallback(int packetSize);
    };