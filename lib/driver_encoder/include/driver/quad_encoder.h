#ifndef DRIVER__ENCODER_H_
#define DRIVER__ENCODER_H_

#include "pico/stdlib.h"

typedef struct encoder_t {
    uint PIN_A;
    uint PIN_B;
    uint cpr;

    volatile bool a_state;
    volatile bool b_state;

    volatile long pulse_counter;
    volatile long pulse_timestamp;

    volatile long prev_pulse_counter, prev_timestamp_us;

    float prev_Th, pulse_per_second;

    long inverted_mul;
} encoder;

float encoder_get_angle(encoder *enc);

float encoder_get_velocity(encoder *enc);

void encoder_init(encoder *enc, uint pin_a, uint pin_b, uint tpr, bool inverted);

#endif  // DRIVER__ENCODER_H_
