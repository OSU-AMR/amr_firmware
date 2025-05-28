#ifndef CORE1_H
#define CORE1_H

#include "pico/stdlib.h"

void core1_init();

void core1_update_targets(const float *rps, const float *voltage_limit);

#define NUM_MOTORS 1
#define DRIVER_VOLTAGE_SUPPLY 16

#endif  // CORE1_H
