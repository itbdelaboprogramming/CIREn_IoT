#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "logger.h"
#include "ciren_comm_types.h"

class CIREnMaster {
public:
    CIREnMaster();
    ~CIREnMaster();

    int init();
    int registerSlave(uint8_t* macAddress);
    esp_err_t slaveTurnOn(uint8_t* macAddress);
    esp_err_t slaveTurnOff(uint8_t* macAddress);

    static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len);

private:
    bool initialized;
    uint8_t slaveList[CIREN_MAX_SLAVES][6];
    uint8_t slaveCount;

    static CIREnMaster* instance;
    static bool cirenInstanceActive;

    esp_err_t sendCommand(uint8_t* macAddress, uint8_t command);
    int findSlaveIndex(uint8_t* macAddress);
    void processMessage(const CIREnMessage* message);
};

class CIREnSlave {
public:
    CIREnSlave(uint8_t device_id);
    ~CIREnSlave();

    int findMaster(uint8_t* macAddress);
    esp_err_t sendTemperature(uint16_t temp);
    esp_err_t sendVibration(uint16_t vib);
    esp_err_t sendVelocity(uint16_t vel);
    esp_err_t sendDisplacement(uint16_t disp);
    esp_err_t sendAcceleration(uint16_t accel);
    esp_err_t sendFrequency(uint32_t freq);
    esp_err_t sendSlaveStatus();

    static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len);

private:
    uint8_t deviceId;
    uint8_t masterMac[6];
    bool masterPaired;
    bool initialized;

    static CIREnSlave* instance;
    static bool cirenInstanceActive;

    esp_err_t sendMessage(uint8_t metric_type, uint16_t value);
    void processCommand(uint8_t command);
};
