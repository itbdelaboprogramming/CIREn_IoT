#pragma once

#include <Arduino.h>

#define METRIC_TYPE_SYSTEM       0
#define METRIC_TYPE_TEMPERATURE  1
#define METRIC_TYPE_VIBRATION    2
#define METRIC_TYPE_VELOCITY     3
#define METRIC_TYPE_DISPLACEMENT 4
#define METRIC_TYPE_ACCELERATION 5
#define METRIC_TYPE_FREQUENCY    6

#define CMD_TURN_OFF 0
#define CMD_TURN_ON 1

#define CIREN_MAX_SLAVES 10

struct CIREnMessage {
    uint8_t id;
    uint8_t metric_type;
    union {
        uint8_t  system_data;
        uint16_t sensor_data;
        uint32_t frequency_data;
    } data;
} __attribute__((packed));
