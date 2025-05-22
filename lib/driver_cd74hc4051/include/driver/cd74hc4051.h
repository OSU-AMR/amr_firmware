#ifndef CD74HC4051_H
#define CD74HC4051_H

#include "pico/stdlib.h"

float multiplexer_decode_analog(uint num);

bool multiplexer_decode_digital(uint num);

void multiplexer_init(uint read_pin, uint s0_pin, uint s1_pin, uint s2_pin);

#endif  // CD74HC4051_H
