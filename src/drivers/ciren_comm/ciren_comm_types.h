#pragma once

#include <Arduino.h>

#define METRIC_TYPE_SYSTEM       0
#define METRIC_TYPE_TEMPERATURE  1
#define METRIC_TYPE_VIBRATION    2
#define METRIC_TYPE_VELOCITY     3
#define METRIC_TYPE_DISPLACEMENT 4
#define METRIC_TYPE_ACCELERATION 5
#define METRIC_TYPE_FREQUENCY    6

#define METRIC_TYPE_VELOCITY_X      7
#define METRIC_TYPE_VELOCITY_Y      8
#define METRIC_TYPE_VELOCITY_Z      9
#define METRIC_TYPE_DISPLACEMENT_X  10
#define METRIC_TYPE_DISPLACEMENT_Y  11
#define METRIC_TYPE_DISPLACEMENT_Z  12
#define METRIC_TYPE_ACCELERATION_X  13
#define METRIC_TYPE_ACCELERATION_Y  14
#define METRIC_TYPE_ACCELERATION_Z  15
#define METRIC_TYPE_FREQUENCY_X     16
#define METRIC_TYPE_FREQUENCY_Y     17
#define METRIC_TYPE_FREQUENCY_Z     18

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
