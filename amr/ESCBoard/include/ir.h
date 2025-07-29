#ifndef IR_H
#define IR_H

#include "pico/stdlib.h"

void ir_init(uint pin);

uint16_t ir_read(uint pin);

#endif  // IR_H
