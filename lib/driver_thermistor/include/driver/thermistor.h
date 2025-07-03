#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "pico/stdlib.h"

typedef enum { THERMISTOR_PROFILE_BOARD, THERMISTOR_PROFILE_CHASSIS } thermistor_profile;

float thermistor_get_c(float voltage, thermistor_profile profile);

#endif  // THERMISTOR_H
