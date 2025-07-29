#ifndef CORE1_H
#define CORE1_H

#include "fixedptc.h"

#include "pico/stdlib.h"

void core1_init();

void core1_update_targets(const fixedpt *rps, const fixedpt *voltage_limit);

#define NUM_MOTORS 2
#define DRIVER_VOLTAGE_SUPPLY_FIXEDPT fixedpt_rconst(16)

#endif  // CORE1_H
