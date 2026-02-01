#pragma once

#include <Arduino.h>
#include <ModbusRTU.h>

#define CWT_SLAVE_ID 0x01
#define CWT_REG_TEMPERATURE 0x0000
#define CWT_REG_VIBRATION 0x0003
#define CWT_REG_BAUDRATE 0x07D1

#define CWT_BAUD_4800 0x01
#define CWT_BAUD_9600 0x02

class CWTSensor {
    friend bool onTemperatureRead(Modbus::ResultCode event, uint16_t transId, void* data);
    friend bool onVibrationRead(Modbus::ResultCode event, uint16_t transId, void* data);

public:
    CWTSensor(HardwareSerial& serial = Serial2);
    ~CWTSensor();

    bool begin(int baudrate = 4800);
    void task();

    void read_temperature();
    void read_vibration();
    void set_baudrate(int baudrate = 4800);

    int get_temperature();
    int get_vibration();

    bool is_temperature_ready();
    bool is_vibration_ready();

private:
    ModbusRTU _modbus;
    HardwareSerial& _serial;

    int _temperature;
    int _vibration;

    bool _temp_ready;
    bool _vib_ready;

    uint16_t _temp_buffer;
    uint16_t _vib_buffer;

    uint16_t baudrate_to_register_value(int baudrate);
};
