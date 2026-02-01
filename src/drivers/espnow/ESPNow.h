/**
 * @file ESPNow.h
 * @date 2025-01-22
 * @class ESPNow
 * @brief This class provides an interface for ESP-NOW wireless communication on ESP32.
 *
 * The ESPNow class implements methods for initializing ESP-NOW, sending sensor data,
 * heartbeat packets, and receiving command packets for remote control.
 */

#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <logger.h>

// Packet type definitions
#define PACKET_TYPE_SENSOR_DATA  0x01
#define PACKET_TYPE_HEARTBEAT    0x02
#define PACKET_TYPE_COMMAND      0x03

// Command definitions
#define CMD_OFF  0x00
#define CMD_ON   0x01

/**
 * @struct SensorDataPacket
 * @brief Structure for sensor data transmission (56 bytes)
 */
struct SensorDataPacket {
    uint8_t packetType;          // 0x01 = Sensor Data
    uint8_t senderMac[6];        // Device identification
    uint32_t timestamp;          // millis() timestamp
    float dht_temperature;       // DHT22 temp (°C)
    float dht_humidity;          // DHT22 humidity (%)
    float max6675_temperature;   // Thermocouple temp (°C)
    float euler_pitch;           // BNO055 pitch
    float euler_roll;            // BNO055 roll
    float euler_yaw;             // BNO055 yaw
    float acc_x;                 // Acceleration X (m/s²)
    float acc_y;                 // Acceleration Y (m/s²)
    float acc_z;                 // Acceleration Z (m/s²)
    uint16_t crc;                // CRC16 validation
};

/**
 * @struct HeartbeatPacket
 * @brief Structure for heartbeat transmission (16 bytes)
 */
struct HeartbeatPacket {
    uint8_t packetType;          // 0x02 = Heartbeat
    uint8_t senderMac[6];        // Device identification
    uint32_t timestamp;          // millis() timestamp
    uint8_t status;              // Operating status (0=OFF, 1=ON)
    uint16_t sequenceNumber;     // Packet counter
    uint16_t crc;                // CRC16 validation
};

/**
 * @struct CommandPacket
 * @brief Structure for command reception (12 bytes)
 */
struct CommandPacket {
    uint8_t packetType;          // 0x03 = Command
    uint8_t senderMac[6];        // Sender identification
    uint8_t command;             // 0x00=OFF, 0x01=ON
    uint32_t timestamp;          // millis() timestamp
};

/**
 * @class ESPNow
 * @brief ESP-NOW communication driver for ESP32
 */
class ESPNow {
public:
    /**
     * @brief Constructor
     * @param peerMac MAC address of peer device
     */
    ESPNow(const uint8_t* peerMac);

    /**
     * @brief Initialize ESP-NOW communication
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t init();

    /**
     * @brief Add peer device to ESP-NOW
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t addPeer();

    /**
     * @brief Send sensor data packet
     * @param packet SensorDataPacket to send
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t sendSensorData(SensorDataPacket& packet);

    /**
     * @brief Send heartbeat packet
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t sendHeartbeat();

    /**
     * @brief Check if peer is added
     * @return true if peer is added, false otherwise
     */
    bool isPeerAdded();

    /**
     * @brief Get current operating state
     * @return true if ON, false if OFF
     */
    bool getOperatingState();

    /**
     * @brief Get last received command
     * @return Last command byte (0x00=OFF, 0x01=ON)
     */
    uint8_t getLastCommand();

    /**
     * @brief Get status string
     * @return Status string ("OK", "NOT_INIT", "NO_PEER")
     */
    const char* getStatusString();

    /**
     * @brief Static callback for ESP-NOW send completion
     */
    static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

    /**
     * @brief Static callback for ESP-NOW data reception
     */
    static void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len);

private:
    uint8_t peerMacAddress[6];   // Peer MAC address
    bool initialized;             // Initialization status
    bool peerAdded;              // Peer registration status
    bool operatingState;         // Current ON/OFF state
    uint8_t lastCommand;         // Last received command
    uint16_t heartbeatSequence;  // Heartbeat packet counter

    /**
     * @brief Calculate CRC16 for data validation
     * @param data Pointer to data buffer
     * @param length Length of data
     * @return CRC16 value
     */
    uint16_t calculateCRC(uint8_t* data, int length);

    /**
     * @brief Process received command packet
     * @param packet CommandPacket received
     */
    void processCommand(const CommandPacket* packet);

    // Static instance pointer for callbacks
    static ESPNow* instance;
};
