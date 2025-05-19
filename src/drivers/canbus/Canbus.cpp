#include "Canbus.h"

const char *TAG_CAN_LIB = "CAN_STATE";

int Canbus::init() {
    esp_err_t ret;

    ret = fInitializeSPI_Channel(this->SCK_PIN, this->MOSI_PIN, this->MISO_PIN, VSPI_HOST, true);
    if (ret != ESP_OK) {
      LOGE(TAG_CAN_LIB, "Error initializing SPI channel: %s", esp_err_to_name(ret));
      return ret;
    }
    
    ret = fInitializeSPI_Devices(VSPI_HOST, spiHandle, this->CS_PIN);
    if (ret != ESP_OK) {
      LOGE(TAG_CAN_LIB, "Error initializing SPI device: %s", esp_err_to_name(ret));
      return ret;
    }

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    return ESP_OK;
}

int Canbus::send() {

    esp_err_t ret;

    struct can_frame frame;
    frame.can_id = this->txHeader.isExtended ? this->txHeader.extendedId : this->txHeader.standardId;
    frame.can_dlc = this->txHeader.dlc;

    for (int i = 0; i < this->txHeader.dlc; i++) {
        frame.data[i] = this->txMailbox[i];
    }

    if(txHeader.isExtended) {

        ret = mcp2515.sendMessage(MCP2515::TXB1, &frame);
        if (ret != ESP_OK) {
            LOGE(TAG_CAN_LIB, "Error sending CAN message: %s", esp_err_to_name(ret));
        } else {
            LOGI(TAG_CAN_LIB, "CAN message (Extended) sent successfully.");
            LOGI(TAG_CAN_LIB, "Data: %02X %02X %02X %02X %02X %02X %02X %02X", frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        }
        
    } else {

        ret = mcp2515.sendMessage(&frame);
        if (ret != ESP_OK) {
            LOGE(TAG_CAN_LIB, "Error sending CAN message: %s", esp_err_to_name(ret));
        } else {
            LOGI(TAG_CAN_LIB, "CAN message (Standard) sent successfully.");
        }
    }

    return ESP_OK;
}

int Canbus::requestId(uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5) {
    
    this->macAddress[0] = c0;
    this->macAddress[1] = c1;
    this->macAddress[2] = c2;
    this->macAddress[3] = c3;
    this->macAddress[4] = c4;
    this->macAddress[5] = c5;

    uint32_t crc = calculateCrc(this->macAddress, 6);
    LOGI(TAG_CAN_LIB, "CRC: %X", crc);

    txHeader.extendedId = (crc << 3 | 5) | CAN_EFF_FLAG; 
    txHeader.dlc = 6; 
    txHeader.rtr = false; 
    txHeader.isExtended = true; 

    this->txMailbox[0] = this->macAddress[0];
    this->txMailbox[1] = this->macAddress[1];
    this->txMailbox[2] = this->macAddress[2];
    this->txMailbox[3] = this->macAddress[3];
    this->txMailbox[4] = this->macAddress[4];
    this->txMailbox[5] = this->macAddress[5];

    // Set Filter for the response message
    mcp2515.reset();
    mcp2515.setFilterMask(MCP2515::MASK0, true, 0x7FFFF); 
    mcp2515.setFilter(MCP2515::RXF0, true, (crc << 3 | 5) ); 
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    this->send();
    
    while(!mcp2515.checkReceive()) {
        LOGI(TAG_CAN_LIB, "Waiting for response...");
        delay(100); 
        
        this->send();
    }

    can_frame frame;
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        LOGI(TAG_CAN_LIB, "Successfully receive response message.");
        LOGI(TAG_CAN_LIB, "Received ID: %d", frame.data[1] << 8 | frame.data[0]);
        this->deviceId = frame.data[1] << 8 | frame.data[0]; // Mask to get the ID
        return ESP_OK; // Return the received ID
    }

    return ESP_FAIL; // Error in receiving ID


}

uint32_t Canbus::calculateCrc(uint8_t* data, int length) {
    uint16_t crc = 0xFFFF; // Initial CRC value

    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR byte into least sig. byte of crc

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001; // Polynomial
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void Canbus::setStandardId(uint16_t deviceId, uint16_t SensorId) {
    this->txHeader.standardId = (deviceId << 8) | SensorId;
    this->txHeader.isExtended = false;
}
void Canbus::setExtendedId(uint16_t deviceId, uint16_t SensorId, uint16_t dataId) {
    this->txHeader.extendedId = ((((deviceId << 4) | SensorId) << 4) | dataId)| CAN_EFF_FLAG;
    this->txHeader.isExtended = true;
}

void Canbus::setDeviceId(uint16_t deviceId) {
    this->deviceId = deviceId;
}

int Canbus::filterMessageByDeviceId(uint16_t id) {

    this->mcp2515.setFilterMask(MCP2515::MASK0, true, 0xFFFF); // Set the filter mask
    this->mcp2515.setFilter(MCP2515::RXF0, true, (id | CAN_EFF_FLAG)); // Set the filter for the device ID
    this->mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    this->mcp2515.setNormalMode();
    
    return ESP_OK;

}

int Canbus::filterMessageBySensorId(uint16_t id) {

    this->mcp2515.setFilterMask(MCP2515::MASK0, true, 0xFF << 16); // Set the filter mask
    this->mcp2515.setFilter(MCP2515::RXF0, true, (id << 16 | CAN_EFF_FLAG)); // Set the filter for the sensor ID
    this->mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    this->mcp2515.setNormalMode();
    
    return ESP_OK;

}

int Canbus::getDeviceId() {
    return this->deviceId;
}

void Canbus::setMessageheartbeat() {
    this->txHeader.extendedId = (this->deviceId << 8 | 0) | CAN_EFF_FLAG; // Set standard ID for heartbeat message
    this->txHeader.dlc = 1; // Set the data length code (DLC) to 8 bytes
    this->txHeader.rtr = false; // Set the remote transmission request (RTR) flag to false
    this->txHeader.isExtended = true; // Set the extended ID flag to false

    this->txMailbox[0] = 0x01; // Set the heartbeat message data
}

void Canbus::setMessageFloat(float data) {
    memcpy(this->txMailbox, &data, sizeof(data)); // Copy the float data to the mailbox
    this->txHeader.dlc = sizeof(data); // Set the data length code (DLC) to the size of the float
}
