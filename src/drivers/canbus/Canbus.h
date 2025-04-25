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

 #include "CAN.h"
 
 class Canbus{
     
 public:

     Canbus();
 
     /**
      * @brief Initializes the CAN peripheral.
      * 
      * This method sets up the CAN hardware and configures the necessary filters and interrupts.
      */
     void init();
 
     /**
      * @brief Sends a CAN message.
      * @return int Returns 0 on success, or a negative value on failure.
      */
     int send();
 
     /**
      * @brief Sets the device ID for CAN messages.
      * @param deviceId The device ID to be set.
      */
     void setDeviceId(uint16_t deviceId);
 
     /**
      * @brief Sets the standard ID for CAN messages.
      * @param deviceId The device ID to be set.
      * @param commandId The command ID to be set.
      */
     void setStandardId(uint16_t deviceId, uint16_t commandId);
 
     /**
      * @brief Filters incoming CAN messages by device ID.
      * @param deviceId The device ID to filter by.
      */
     void filterByIdDevice(uint16_t deviceId);
 
     /**
      * @brief Filters incoming CAN messages by command ID.
      * @param commandId The command ID to filter by.
      */
     void filterByIdCommand(uint16_t commandId);
 
     /**
      * @brief Filters incoming CAN messages by both device ID and command ID.
      * @param deviceId The device ID to filter by.
      * @param commandId The command ID to filter by.
      */
     void filterById(uint16_t deviceId, uint16_t commandId);
 
     /**
      * @brief Retrieves the device ID from the received CAN message.
      * @return int The device ID of the received message.
      */
     int getRxDeviceId();
 
     /**
      * @brief Retrieves the command ID from the received CAN message.
      * @return int The command ID of the received message.
      */
     int getRxCommandId();
 
 
     /**
      * @brief Retrieves the standard ID from the received CAN message header.
      * @return int The standard ID of the received message.
      */
     int getRxheaderStdId();
 
     /**
      * @brief Get the Message in 8-bytes of uint8_t char
      * @param data Pointer to store the raw 8-byte data
      */
     void getMessageRaw(uint8_t* data);
 
 private:
 
     /**
      * @brief Configures the CAN filter based on the current device and command IDs.
      */
     void configureFilter();
 
     /**
      * @brief Handles the received CAN message.
      */
     void handleReceivedMessage();
 };