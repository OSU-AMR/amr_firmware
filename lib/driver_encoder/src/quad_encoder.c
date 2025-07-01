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

#include "driver/quad_encoder.h"

#include <math.h>
#include <quadrature_decoder.pio.h>

#define QUADRATURE_SCALE 4

int32_t encoder_init(encoder *enc, PIO pio, uint8_t pinbase, uint tpr, bool inverted) {
    // Only add program when encoder_init is called for the first time
    static bool first_init = true;
    if (first_init) {
        if (!pio_can_add_program(pio, &QuadratureDecoder_program))
            return -1;

        pio_add_program(pio, &QuadratureDecoder_program);
        first_init = false;
    }

    enc->dma_channel = 0;
    enc->cpr = tpr * QUADRATURE_SCALE;
    enc->counter = 0;
    enc->last_angle = 0.0f;
    enc->last_stamp = 0.0f;
    enc->inverted_mul = inverted ? -1 : 1;

    // Find an unused state machine in the PIO to run the code for counting this encoder.
    int32_t state_machine = pio_claim_unused_sm(pio, false);
    if (state_machine < 0) {
        // No free state machines so return a failure code.
        return -1;
    }

    // The code is always loaded at offset 0 because of its jump table.
    const uint program_offset = 0;
    pio_sm_config sm_config = QuadratureDecoder_program_get_default_config(program_offset);
    // Configure the state machine to run the quadrature decoder program.
    const bool shift_left = false;
    const bool no_auto_push = false;
    const uint threshhold = 32;
    // We want the ISR to shift to right when the pin values are shifted in.
    sm_config_set_in_shift(&sm_config, shift_left, no_auto_push, threshhold);
    sm_config_set_in_pins(&sm_config, pinbase);
    // Use the TX FIFO entries for RX since we don't use the TX path. This makes for an 8 element RX FIFO.
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, state_machine, QuadratureDecoder_offset_start, &sm_config);

    int dma_channel = dma_claim_unused_channel(false);
    if (dma_channel < 0) {
        // No free DMA channels so return a failure code.
        return -1;
    }
    enc->dma_channel = dma_channel;
    // Configure DMA to just read the latest count from the state machine's RX FIFO and place it in the counter
    // element reserved for this encoder.
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(pio, state_machine, false));

    volatile long *p_counter = &enc->counter;
    dma_channel_configure(dma_channel, &dma_config,
                          p_counter,                 // Destination pointer
                          &pio->rxf[state_machine],  // Source pointer
                          DMA_MAX_TRANSFER_COUNT,    // Largest possible number of transfers
                          true                       // Start immediately
    );
    // Initialize state machine registers.
    // Initialize the X register to an initial count of 0.
    *p_counter = 0;
    pio_sm_exec(pio, state_machine, pio_encode_set(pio_x, *p_counter));
    // Initialize the Y register to the current value of the pins.
    pio_sm_exec(pio, state_machine, pio_encode_mov(pio_y, pio_pins));
    // Now start the state machine to count quadrature encoder ticks.
    pio_sm_set_enabled(pio, state_machine, true);

    return state_machine;
}

void restart_dma_before_it_stops(encoder *enc) {
    uint32_t dma_channel = enc->dma_channel;
    uint32_t dma_transfers_left = dma_channel_hw_addr(dma_channel)->transfer_count;
    if (dma_transfers_left > DMA_REFRESH_THRESHOLD) {
        // There are still a lot of transfers left in this DMA operation so just return.
        return;
    }
    // Stopping the DMA channel and starting it again will cause it to use all of the original settings,
    // including the 0xFFFFFFFF transfer count.
    dma_channel_abort(dma_channel);
    dma_channel_start(dma_channel);
}

float encoder_get_angle(encoder *enc) {
    restart_dma_before_it_stops(enc);
    return (2 * M_PI) * (enc->counter * enc->inverted_mul) / enc->cpr;
}

float encoder_get_velocity(encoder *enc) {
    float angle = encoder_get_angle(enc);
    float stamp = to_us_since_boot(get_absolute_time()) * 1e-6f;

    float vel = (angle - enc->last_angle) / (stamp - enc->last_stamp);
    enc->last_angle = angle;
    enc->last_stamp = stamp;

    return vel;
}
