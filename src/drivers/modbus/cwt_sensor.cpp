#include "cwt_sensor.h"

CWTSensor* _instance = nullptr;

bool onTemperatureRead(Modbus::ResultCode event, uint16_t transId, void* data) {
    if (_instance && event == Modbus::EX_SUCCESS) {
        _instance->_temperature = _instance->_temp_buffer;
        _instance->_temp_ready = true;
    }
    return true;
}

bool onVibrationRead(Modbus::ResultCode event, uint16_t transId, void* data) {
    if (_instance && event == Modbus::EX_SUCCESS) {
        _instance->_vibration = _instance->_vib_buffer;
        _instance->_vib_ready = true;
    }
    return true;
}

CWTSensor::CWTSensor(HardwareSerial& serial)
    : _serial(serial), _temperature(0), _vibration(0),
      _temp_ready(false), _vib_ready(false),
      _temp_buffer(0), _vib_buffer(0) {
    _instance = this;
}

CWTSensor::~CWTSensor() {
    _instance = nullptr;
}

bool CWTSensor::begin(int baudrate) {
    _serial.begin(baudrate, SERIAL_8N1, 16, 17);
    _modbus.begin(&_serial);
    _modbus.master();
    return true;
}

void CWTSensor::task() {
    _modbus.task();
}

void CWTSensor::read_temperature() {
    _temp_ready = false;
    _modbus.readHreg(CWT_SLAVE_ID, CWT_REG_TEMPERATURE, &_temp_buffer, 1, onTemperatureRead);
}

void CWTSensor::read_vibration() {
    _vib_ready = false;
    _modbus.readHreg(CWT_SLAVE_ID, CWT_REG_VIBRATION, &_vib_buffer, 1, onVibrationRead);
}

void CWTSensor::set_baudrate(int baudrate) {
    uint16_t reg_value = baudrate_to_register_value(baudrate);
    _modbus.writeHreg(CWT_SLAVE_ID, CWT_REG_BAUDRATE, reg_value);
}

int CWTSensor::get_temperature() {
    return _temperature;
}

int CWTSensor::get_vibration() {
    return _vibration;
}

bool CWTSensor::is_temperature_ready() {
    return _temp_ready;
}

bool CWTSensor::is_vibration_ready() {
    return _vib_ready;
}

uint16_t CWTSensor::baudrate_to_register_value(int baudrate) {
    switch(baudrate) {
        case 4800: return CWT_BAUD_4800;
        case 9600: return CWT_BAUD_9600;
        default: return CWT_BAUD_4800;
    }
}
