#pragma once
#include "hardware/pwm.h"

typedef struct RP2040DriverParams {
    int pins[6];
    uint slice[6];
    uint chan[6];
    long pwm_frequency;
    float dead_zone;
} RP2040DriverParams;
