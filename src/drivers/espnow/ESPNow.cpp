/**
 * @file ESPNow.cpp
 * @brief Implementation of ESP-NOW communication driver
 */

#include "ESPNow.h"

const char *TAG_ESPNOW_LIB = "ESPNOW_LIB";

// Static instance pointer for callbacks
ESPNow* ESPNow::instance = nullptr;

/**
 * @brief Constructor
 */
ESPNow::ESPNow(const uint8_t* peerMac) {
    memcpy(peerMacAddress, peerMac, 6);
    initialized = false;
    peerAdded = false;
    operatingState = false;  // Default to OFF state
    lastCommand = CMD_OFF;
    heartbeatSequence = 0;
    instance = this;  // Set static instance for callbacks
}

/**
 * @brief Initialize ESP-NOW communication
 */
esp_err_t ESPNow::init() {
    // Set WiFi mode to STA (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Initialize ESP-NOW
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        LOGE(TAG_ESPNOW_LIB, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register callbacks
    ret = esp_now_register_send_cb(onDataSent);
    if (ret != ESP_OK) {
        LOGE(TAG_ESPNOW_LIB, "Failed to register send callback: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_now_register_recv_cb(onDataReceived);
    if (ret != ESP_OK) {
        LOGE(TAG_ESPNOW_LIB, "Failed to register recv callback: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = true;
    LOGI(TAG_ESPNOW_LIB, "ESP-NOW initialized successfully");
    return ESP_OK;
}

/**
 * @brief Add peer device to ESP-NOW
 */
esp_err_t ESPNow::addPeer() {
    if (!initialized) {
        LOGE(TAG_ESPNOW_LIB, "ESP-NOW not initialized");
        return ESP_FAIL;
    }

    // Configure peer info
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMacAddress, 6);
    peerInfo.channel = 0;  // Use current channel
    peerInfo.encrypt = false;  // No encryption
    peerInfo.ifidx = WIFI_IF_STA;

    // Add peer
    esp_err_t ret = esp_now_add_peer(&peerInfo);
    if (ret != ESP_OK) {
        LOGE(TAG_ESPNOW_LIB, "Failed to add peer: %s", esp_err_to_name(ret));
        return ret;
    }

    peerAdded = true;
    LOGI(TAG_ESPNOW_LIB, "Peer added: %02X:%02X:%02X:%02X:%02X:%02X",
         peerMacAddress[0], peerMacAddress[1], peerMacAddress[2],
         peerMacAddress[3], peerMacAddress[4], peerMacAddress[5]);
    return ESP_OK;
}

/**
 * @brief Send sensor data packet
 */
esp_err_t ESPNow::sendSensorData(SensorDataPacket& packet) {
    if (!initialized || !peerAdded) {
        return ESP_FAIL;
    }

    // Set packet type
    packet.packetType = PACKET_TYPE_SENSOR_DATA;

    // Calculate CRC (exclude CRC field itself)
    packet.crc = calculateCRC((uint8_t*)&packet, sizeof(SensorDataPacket) - sizeof(uint16_t));

    // Send packet
    esp_err_t ret = esp_now_send(peerMacAddress, (uint8_t*)&packet, sizeof(SensorDataPacket));
    return ret;
}

/**
 * @brief Send heartbeat packet
 */
esp_err_t ESPNow::sendHeartbeat() {
    if (!initialized || !peerAdded) {
        return ESP_FAIL;
    }

    HeartbeatPacket packet;
    packet.packetType = PACKET_TYPE_HEARTBEAT;

    // Get MAC address
    esp_read_mac(packet.senderMac, ESP_MAC_WIFI_STA);

    packet.timestamp = millis();
    packet.status = operatingState ? 1 : 0;
    packet.sequenceNumber = heartbeatSequence++;

    // Calculate CRC (exclude CRC field itself)
    packet.crc = calculateCRC((uint8_t*)&packet, sizeof(HeartbeatPacket) - sizeof(uint16_t));

    // Send packet
    esp_err_t ret = esp_now_send(peerMacAddress, (uint8_t*)&packet, sizeof(HeartbeatPacket));
    return ret;
}

/**
 * @brief Check if peer is added
 */
bool ESPNow::isPeerAdded() {
    return peerAdded;
}

/**
 * @brief Get current operating state
 */
bool ESPNow::getOperatingState() {
    return operatingState;
}

/**
 * @brief Get last received command
 */
uint8_t ESPNow::getLastCommand() {
    return lastCommand;
}

/**
 * @brief Get status string
 */
const char* ESPNow::getStatusString() {
    if (!initialized) {
        return "NOT_INIT";
    }
    if (!peerAdded) {
        return "NO_PEER";
    }
    return "OK";
}

/**
 * @brief Static callback for ESP-NOW send completion
 */
void ESPNow::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional: Log send status for debugging
    // Can be used to track transmission success rate
}

/**
 * @brief Static callback for ESP-NOW data reception
 */
void ESPNow::onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (instance == nullptr || data == nullptr || len < 1) {
        return;
    }

    // Check packet type
    uint8_t packetType = data[0];

    if (packetType == PACKET_TYPE_COMMAND && len == sizeof(CommandPacket)) {
        // Process command packet
        const CommandPacket* cmdPacket = (const CommandPacket*)data;
        instance->processCommand(cmdPacket);
    }
}

/**
 * @brief Calculate CRC16 for data validation
 */
uint16_t ESPNow::calculateCRC(uint8_t* data, int length) {
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Process received command packet
 */
void ESPNow::processCommand(const CommandPacket* packet) {
    if (packet == nullptr) {
        return;
    }

    lastCommand = packet->command;

    // Update operating state based on command
    if (packet->command == CMD_ON) {
        operatingState = true;
        LOGI(TAG_ESPNOW_LIB, "Command received: ON (0x01)");
    } else if (packet->command == CMD_OFF) {
        operatingState = false;
        LOGI(TAG_ESPNOW_LIB, "Command received: OFF (0x00)");
    } else {
        LOGW(TAG_ESPNOW_LIB, "Unknown command: 0x%02X", packet->command);
    }
}
