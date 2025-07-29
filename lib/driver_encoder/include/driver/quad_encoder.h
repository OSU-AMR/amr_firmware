/*
   This library is a C translation of the [QuadratureDecoder]
   (https://github.com/adamgreen/QuadratureDecoder),
   all functions are a 1:1 port of the original.
   2022 Unmanned

    Copyright 2021 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef QUAD_ENCODER_H
#define QUAD_ENCODER_H

#include <hardware/dma.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define DMA_MAX_TRANSFER_COUNT 0xFFFFFFFF
#define DMA_REFRESH_THRESHOLD 0x80000000

typedef struct encoder_t {
    uint32_t dma_channel;
    uint cpr;
    long inverted_mul;

    volatile long counter;
    float last_angle;
    float last_stamp;

} encoder;

int32_t encoder_init(encoder *enc, PIO pio, uint8_t pinbase, uint tpr, bool inverted);

float encoder_get_angle(encoder *enc);
float encoder_get_velocity(encoder *enc);

#endif  // QUAD_ENCODER_H
