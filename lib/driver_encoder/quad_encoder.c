#include "driver/quad_encoder.h"

#include "hardware/sync.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define QUADRATURE_SCALE 4

static uint enc_cnt = 0;
static encoder **encoders;

static void pulse_callback(uint gpio, uint32_t events) {
    encoder *curr_enc = NULL;

    for (uint i = 0; i < enc_cnt; i++) {
        curr_enc = encoders[i];

        if (gpio == curr_enc->PIN_A) {
            curr_enc->pulse_counter += (curr_enc->a_state == curr_enc->b_state) ? 1 : -1;
            curr_enc->a_state = events & 8;
            break;
        }
        else if (gpio == curr_enc->PIN_B) {
            curr_enc->pulse_counter += (curr_enc->a_state != curr_enc->b_state) ? 1 : -1;
            curr_enc->b_state = events & 8;
            break;
        }
    }

    // curr_enc will always be set by the loop
    curr_enc->pulse_timestamp = to_us_since_boot(get_absolute_time());
}

// Yes, this isn't very efficient, but it only happens on startup
static void insert_resizing(encoder *enc) {
    if (!enc_cnt) {
        encoders = malloc(sizeof(encoder *));
        encoders[0] = enc;
    }
    else {
        encoders = realloc(encoders, (enc_cnt + 1) * sizeof(encoder *));
        encoders[enc_cnt] = enc;
    }

    enc_cnt++;
}

float encoder_get_angle(encoder *enc) {
    return (2 * M_PI) * enc->pulse_counter / enc->cpr;
}

float encoder_get_velocity(encoder *enc) {
    // Keep this fast to not drop encoder pulses
    uint32_t flags = save_and_disable_interrupts();
    long copy_pulse_counter = enc->pulse_counter;
    long copy_pulse_timestamp = enc->pulse_timestamp;
    restore_interrupts(flags);

    // timestamp
    long timestamp_us = to_us_since_boot(get_absolute_time());
    // sampling time calculation
    float Ts = (timestamp_us - enc->prev_timestamp_us) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // time from last impulse
    float Th = (timestamp_us - copy_pulse_timestamp) * 1e-6f;
    long dN = copy_pulse_counter - enc->prev_pulse_counter;

    // Pulse per second calculation (Eq.3.)
    // dN - impulses received
    // Ts - sampling time - time in between function calls
    // Th - time from last impulse
    // Th_1 - time form last impulse of the previous call
    // only increment if some impulses received
    float dt = Ts + enc->prev_Th - Th;
    enc->pulse_per_second = (dN != 0 && dt > Ts / 2) ? dN / dt : enc->pulse_per_second;

    // if more than 0.05f passed in between impulses
    if (Th > 0.1f)
        enc->pulse_per_second = 0;

    // velocity calculation
    float velocity = enc->pulse_per_second / ((float) enc->cpr) * (2 * M_PI);

    // save variables for next pass
    enc->prev_timestamp_us = timestamp_us;
    // save velocity calculation variables
    enc->prev_Th = Th;
    enc->prev_pulse_counter = copy_pulse_counter;
    return velocity;
}

void encoder_init(encoder *enc, uint pin_a, uint pin_b, uint tpr) {
    enc->PIN_A = pin_a;
    enc->PIN_B = pin_b;
    enc->cpr = tpr * QUADRATURE_SCALE;
    enc->a_state = false;
    enc->b_state = false;
    enc->pulse_counter = 0;
    enc->pulse_timestamp = 0;
    enc->prev_pulse_counter = 0;
    enc->prev_timestamp_us = 0;
    enc->prev_Th = 0.0f;
    enc->pulse_per_second = 0.0f;

    gpio_init(pin_a);
    gpio_init(pin_b);
    gpio_set_irq_enabled_with_callback(pin_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_callback);
    gpio_set_irq_enabled_with_callback(pin_b, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pulse_callback);

    insert_resizing(enc);
}
