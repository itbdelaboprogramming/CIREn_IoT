#pragma once

#include <Arduino.h>

#define METRIC_TYPE_SYSTEM 0
#define METRIC_TYPE_TEMPERATURE 1
#define METRIC_TYPE_VIBRATION 2

#define CMD_TURN_OFF 0
#define CMD_TURN_ON 1

#define CIREN_MAX_SLAVES 10

struct CIREnMessage {
    uint8_t id;
    uint8_t metric_type;
    union {
        uint8_t system_data;
        uint16_t sensor_data;
    } data;
} __attribute__((packed));
