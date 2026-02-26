#include "ciren_comm.h"

CIREnMaster* CIREnMaster::instance = nullptr;
CIREnSlave* CIREnSlave::instance = nullptr;
bool CIREnMaster::cirenInstanceActive = false;
bool CIREnSlave::cirenInstanceActive = false;

CIREnMaster::CIREnMaster() {
    if (cirenInstanceActive) {
        LOGE("CIREN_MASTER", "Cannot instantiate CIREnMaster: CIRen instance already active");
        return;
    }
    cirenInstanceActive = true;
    instance = this;
    initialized = false;
    slaveCount = 0;
}

CIREnMaster::~CIREnMaster() {
    if (initialized) {
        esp_now_deinit();
    }
    cirenInstanceActive = false;
    instance = nullptr;
}

int CIREnMaster::init() {
    if (initialized) {
        LOGW("CIREN_MASTER", "Already initialized");
        return ESP_OK;
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        LOGE("CIREN_MASTER", "ESP-NOW init failed");
        return ESP_FAIL;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    initialized = true;
    LOGI("CIREN_MASTER", "Initialized successfully");
    return ESP_OK;
}

int CIREnMaster::registerSlave(uint8_t* macAddress) {
    if (!initialized) {
        LOGE("CIREN_MASTER", "Not initialized");
        return ESP_FAIL;
    }

    if (slaveCount >= CIREN_MAX_SLAVES) {
        LOGE("CIREN_MASTER", "Maximum slave limit reached");
        return ESP_FAIL;
    }

    memcpy(slaveList[slaveCount], macAddress, 6);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, macAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        LOGE("CIREN_MASTER", "Failed to add peer");
        return ESP_FAIL;
    }

    slaveCount++;
    LOGI("CIREN_MASTER", "Slave registered: %02X:%02X:%02X:%02X:%02X:%02X",
         macAddress[0], macAddress[1], macAddress[2],
         macAddress[3], macAddress[4], macAddress[5]);
    return ESP_OK;
}

esp_err_t CIREnMaster::sendCommand(uint8_t* macAddress, uint8_t command) {
    if (!initialized) {
        LOGE("CIREN_MASTER", "Not initialized");
        return ESP_FAIL;
    }

    int slaveIndex = findSlaveIndex(macAddress);
    if (slaveIndex < 0) {
        LOGE("CIREN_MASTER", "Slave not found");
        return ESP_FAIL;
    }

    CIREnMessage message;
    message.id = 0;
    message.metric_type = METRIC_TYPE_SYSTEM;
    message.data.system_data = command;

    esp_err_t result = esp_now_send(macAddress, (uint8_t*)&message, sizeof(CIREnMessage));

    if (result != ESP_OK) {
        LOGE("CIREN_MASTER", "Send command failed");
    }

    return result;
}

esp_err_t CIREnMaster::slaveTurnOn(uint8_t* macAddress) {
    LOGI("CIREN_MASTER", "Sending turn-on command");
    return sendCommand(macAddress, CMD_TURN_ON);
}

esp_err_t CIREnMaster::slaveTurnOff(uint8_t* macAddress) {
    LOGI("CIREN_MASTER", "Sending turn-off command");
    return sendCommand(macAddress, CMD_TURN_OFF);
}

int CIREnMaster::findSlaveIndex(uint8_t* macAddress) {
    for (int i = 0; i < slaveCount; i++) {
        if (memcmp(slaveList[i], macAddress, 6) == 0) {
            return i;
        }
    }
    return -1;
}

void CIREnMaster::processMessage(const CIREnMessage* message) {
    if (message->metric_type == METRIC_TYPE_SYSTEM) return;
    switch (message->metric_type) {
        case METRIC_TYPE_TEMPERATURE:
            Serial.printf("[ID %d] Temperature:  %.1f\n", message->id, message->data.sensor_data / 10.0f);
            break;
        case METRIC_TYPE_VELOCITY:
            Serial.printf("[ID %d] Velocity:     %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_DISPLACEMENT:
            Serial.printf("[ID %d] Displacement: %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_ACCELERATION:
            Serial.printf("[ID %d] Acceleration: %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_FREQUENCY:
            Serial.printf("[ID %d] Freq:         %lu Hz\n", message->id, message->data.frequency_data);
            break;
        case METRIC_TYPE_VELOCITY_X:
            Serial.printf("[ID %d] Velocity X:   %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_VELOCITY_Y:
            Serial.printf("[ID %d] Velocity Y:   %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_VELOCITY_Z:
            Serial.printf("[ID %d] Velocity Z:   %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_DISPLACEMENT_X:
            Serial.printf("[ID %d] Displace X:   %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_DISPLACEMENT_Y:
            Serial.printf("[ID %d] Displace Y:   %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_DISPLACEMENT_Z:
            Serial.printf("[ID %d] Displace Z:   %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_ACCELERATION_X:
            Serial.printf("[ID %d] Accel X:      %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_ACCELERATION_Y:
            Serial.printf("[ID %d] Accel Y:      %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_ACCELERATION_Z:
            Serial.printf("[ID %d] Accel Z:      %.2f\n", message->id, message->data.sensor_data / 100.0f);
            break;
        case METRIC_TYPE_FREQUENCY_X:
            Serial.printf("[ID %d] Freq X:       %lu Hz\n", message->id, message->data.frequency_data);
            break;
        case METRIC_TYPE_FREQUENCY_Y:
            Serial.printf("[ID %d] Freq Y:       %lu Hz\n", message->id, message->data.frequency_data);
            break;
        case METRIC_TYPE_FREQUENCY_Z:
            Serial.printf("[ID %d] Freq Z:       %lu Hz\n", message->id, message->data.frequency_data);
            break;
        default:
            Serial.printf("[ID %d] Metric=%d Data=%d\n", message->id, message->metric_type, message->data.sensor_data);
    }
}

void CIREnMaster::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (instance == nullptr) {
        return;
    }

    if (status == ESP_NOW_SEND_SUCCESS) {
        LOGI("CIREN_MASTER", "Message sent successfully");
    } else {
        LOGE("CIREN_MASTER", "Message send failed");
    }
}

void CIREnMaster::onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (instance == nullptr) {
        return;
    }

    if (len != sizeof(CIREnMessage)) {
        LOGE("CIREN_MASTER", "Invalid message size");
        return;
    }

    const CIREnMessage* message = (const CIREnMessage*)data;
    instance->processMessage(message);
}

CIREnSlave::CIREnSlave(uint8_t device_id) {
    if (cirenInstanceActive) {
        LOGE("CIREN_SLAVE", "Cannot instantiate CIREnSlave: CIRen instance already active");
        return;
    }
    cirenInstanceActive = true;
    instance = this;
    deviceId = device_id;
    masterPaired = false;
    initialized = false;
}

CIREnSlave::~CIREnSlave() {
    if (initialized) {
        esp_now_deinit();
    }
    cirenInstanceActive = false;
    instance = nullptr;
}

int CIREnSlave::findMaster(uint8_t* macAddress) {
    if (initialized) {
        LOGW("CIREN_SLAVE", "Already initialized");
        return ESP_OK;
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        LOGE("CIREN_SLAVE", "ESP-NOW init failed");
        return ESP_FAIL;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    memcpy(masterMac, macAddress, 6);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, macAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        LOGE("CIREN_SLAVE", "Failed to add master peer");
        return ESP_FAIL;
    }

    masterPaired = true;
    initialized = true;
    LOGI("CIREN_SLAVE", "Master paired: %02X:%02X:%02X:%02X:%02X:%02X",
         macAddress[0], macAddress[1], macAddress[2],
         macAddress[3], macAddress[4], macAddress[5]);
    return ESP_OK;
}

esp_err_t CIREnSlave::sendMessage(uint8_t metric_type, uint16_t value) {
    if (!initialized || !masterPaired) {
        LOGE("CIREN_SLAVE", "Not initialized or master not paired");
        return ESP_FAIL;
    }

    CIREnMessage message;
    message.id = deviceId;
    message.metric_type = metric_type;
    message.data.sensor_data = value;

    esp_err_t result = esp_now_send(masterMac, (uint8_t*)&message, sizeof(CIREnMessage));

    if (result != ESP_OK) {
        LOGE("CIREN_SLAVE", "Send message failed");
    }

    return result;
}

esp_err_t CIREnSlave::sendTemperature(uint16_t temp) {
    LOGI("CIREN_SLAVE", "Sending temperature: %d", temp);
    return sendMessage(METRIC_TYPE_TEMPERATURE, temp);
}

esp_err_t CIREnSlave::sendVibration(uint16_t vib) {
    LOGI("CIREN_SLAVE", "Sending vibration: %d", vib);
    return sendMessage(METRIC_TYPE_VIBRATION, vib);
}

esp_err_t CIREnSlave::sendVelocity(uint16_t vel) {
    LOGI("CIREN_SLAVE", "Sending velocity: %d", vel);
    return sendMessage(METRIC_TYPE_VELOCITY, vel);
}

esp_err_t CIREnSlave::sendDisplacement(uint16_t disp) {
    LOGI("CIREN_SLAVE", "Sending displacement: %d", disp);
    return sendMessage(METRIC_TYPE_DISPLACEMENT, disp);
}

esp_err_t CIREnSlave::sendAcceleration(uint16_t accel) {
    LOGI("CIREN_SLAVE", "Sending acceleration: %d", accel);
    return sendMessage(METRIC_TYPE_ACCELERATION, accel);
}

esp_err_t CIREnSlave::sendFrequency(uint32_t freq) {
    if (!initialized || !masterPaired) {
        LOGE("CIREN_SLAVE", "Not initialized or master not paired");
        return ESP_FAIL;
    }
    CIREnMessage message;
    message.id = deviceId;
    message.metric_type = METRIC_TYPE_FREQUENCY;
    message.data.frequency_data = freq;
    esp_err_t result = esp_now_send(masterMac, (uint8_t*)&message, sizeof(CIREnMessage));
    if (result != ESP_OK) {
        LOGE("CIREN_SLAVE", "Send frequency failed");
    }
    return result;
}

esp_err_t CIREnSlave::sendVelocityX(uint16_t vel) {
    LOGI("CIREN_SLAVE", "Sending velocity X: %d", vel);
    return sendMessage(METRIC_TYPE_VELOCITY_X, vel);
}

esp_err_t CIREnSlave::sendVelocityY(uint16_t vel) {
    LOGI("CIREN_SLAVE", "Sending velocity Y: %d", vel);
    return sendMessage(METRIC_TYPE_VELOCITY_Y, vel);
}

esp_err_t CIREnSlave::sendVelocityZ(uint16_t vel) {
    LOGI("CIREN_SLAVE", "Sending velocity Z: %d", vel);
    return sendMessage(METRIC_TYPE_VELOCITY_Z, vel);
}

esp_err_t CIREnSlave::sendDisplacementX(uint16_t disp) {
    LOGI("CIREN_SLAVE", "Sending displacement X: %d", disp);
    return sendMessage(METRIC_TYPE_DISPLACEMENT_X, disp);
}

esp_err_t CIREnSlave::sendDisplacementY(uint16_t disp) {
    LOGI("CIREN_SLAVE", "Sending displacement Y: %d", disp);
    return sendMessage(METRIC_TYPE_DISPLACEMENT_Y, disp);
}

esp_err_t CIREnSlave::sendDisplacementZ(uint16_t disp) {
    LOGI("CIREN_SLAVE", "Sending displacement Z: %d", disp);
    return sendMessage(METRIC_TYPE_DISPLACEMENT_Z, disp);
}

esp_err_t CIREnSlave::sendAccelerationX(uint16_t accel) {
    LOGI("CIREN_SLAVE", "Sending acceleration X: %d", accel);
    return sendMessage(METRIC_TYPE_ACCELERATION_X, accel);
}

esp_err_t CIREnSlave::sendAccelerationY(uint16_t accel) {
    LOGI("CIREN_SLAVE", "Sending acceleration Y: %d", accel);
    return sendMessage(METRIC_TYPE_ACCELERATION_Y, accel);
}

esp_err_t CIREnSlave::sendAccelerationZ(uint16_t accel) {
    LOGI("CIREN_SLAVE", "Sending acceleration Z: %d", accel);
    return sendMessage(METRIC_TYPE_ACCELERATION_Z, accel);
}

esp_err_t CIREnSlave::sendFrequencyX(uint32_t freq) {
    if (!initialized || !masterPaired) {
        LOGE("CIREN_SLAVE", "Not initialized or master not paired");
        return ESP_FAIL;
    }
    CIREnMessage message;
    message.id = deviceId;
    message.metric_type = METRIC_TYPE_FREQUENCY_X;
    message.data.frequency_data = freq;
    esp_err_t result = esp_now_send(masterMac, (uint8_t*)&message, sizeof(CIREnMessage));
    if (result != ESP_OK) LOGE("CIREN_SLAVE", "Send frequency X failed");
    return result;
}

esp_err_t CIREnSlave::sendFrequencyY(uint32_t freq) {
    if (!initialized || !masterPaired) {
        LOGE("CIREN_SLAVE", "Not initialized or master not paired");
        return ESP_FAIL;
    }
    CIREnMessage message;
    message.id = deviceId;
    message.metric_type = METRIC_TYPE_FREQUENCY_Y;
    message.data.frequency_data = freq;
    esp_err_t result = esp_now_send(masterMac, (uint8_t*)&message, sizeof(CIREnMessage));
    if (result != ESP_OK) LOGE("CIREN_SLAVE", "Send frequency Y failed");
    return result;
}

esp_err_t CIREnSlave::sendFrequencyZ(uint32_t freq) {
    if (!initialized || !masterPaired) {
        LOGE("CIREN_SLAVE", "Not initialized or master not paired");
        return ESP_FAIL;
    }
    CIREnMessage message;
    message.id = deviceId;
    message.metric_type = METRIC_TYPE_FREQUENCY_Z;
    message.data.frequency_data = freq;
    esp_err_t result = esp_now_send(masterMac, (uint8_t*)&message, sizeof(CIREnMessage));
    if (result != ESP_OK) LOGE("CIREN_SLAVE", "Send frequency Z failed");
    return result;
}

esp_err_t CIREnSlave::sendSlaveStatus() {
    if (!initialized || !masterPaired) {
        LOGE("CIREN_SLAVE", "Not initialized or master not paired");
        return ESP_FAIL;
    }

    CIREnMessage message;
    message.id = deviceId;
    message.metric_type = METRIC_TYPE_SYSTEM;
    message.data.system_data = 1;

    esp_err_t result = esp_now_send(masterMac, (uint8_t*)&message, sizeof(CIREnMessage));

    if (result != ESP_OK) {
        LOGE("CIREN_SLAVE", "Send status failed");
    } else {
        LOGI("CIREN_SLAVE", "Status sent");
    }

    return result;
}

void CIREnSlave::processCommand(uint8_t command) {
    LOGI("CIREN_SLAVE", "Command received: %d", command);
}

void CIREnSlave::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (instance == nullptr) {
        return;
    }

    if (status == ESP_NOW_SEND_SUCCESS) {
        LOGI("CIREN_SLAVE", "Message sent successfully");
    } else {
        LOGE("CIREN_SLAVE", "Message send failed");
    }
}

void CIREnSlave::onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (instance == nullptr) {
        return;
    }

    if (len != sizeof(CIREnMessage)) {
        LOGE("CIREN_SLAVE", "Invalid message size");
        return;
    }

    const CIREnMessage* message = (const CIREnMessage*)data;

    if (message->metric_type == METRIC_TYPE_SYSTEM) {
        instance->processCommand(message->data.system_data);
        LOGI("CIREN_SLAVE", "Received command: %d", message->data.system_data);
    }
}
