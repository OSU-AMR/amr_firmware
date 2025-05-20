#include "driver/quad_encoder.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/version.h"

#include <math.h>
#include <stdio.h>

#define cpr 2048 * 4

const uint LED_PIN = 25;
// const uint GPIO_WATCH_PIN = 1;
// const uint GPIO_WATCH_PIN2 = 2;
// const uint GPIO_WRITE = 0;

// const uint PIN_A = 27;
// const uint PIN_B = 28;

// volatile bool a_state = false;
// volatile bool b_state = false;

// volatile long pulse_counter = 0;
// volatile long pulse_timestamp = 0;

// volatile long prev_pulse_counter, prev_timestamp_us;

// float prev_Th, pulse_per_second;

// void gpio_callback(uint gpio, uint32_t events) {
//     // Put the GPIO event(s) that just happened into event_str
//     // so we can print it
//     // gpio_event_string(event_str, events);
//     // printf("GPIO %d %s\n", gpio, event_str);

//     if (gpio == PIN_A) {
//         pulse_counter += (a_state == b_state) ? 1 : -1;
//         a_state = events & 8;
//     }
//     else if (gpio == PIN_B) {
//         pulse_counter += (a_state != b_state) ? 1 : -1;
//         b_state = events & 8;
//     }

//     pulse_timestamp = to_us_since_boot(get_absolute_time());
// }

// float get_angle() {
//     return (2 * M_PI) * pulse_counter / cpr * 180 / M_PI;
// }

// float get_vel() {
//     // Keep this fast to not drop encoder pulses
//     uint32_t flags = save_and_disable_interrupts();
//     long copy_pulse_counter = pulse_counter;
//     long copy_pulse_timestamp = pulse_timestamp;
//     restore_interrupts(flags);

//     // timestamp
//     long timestamp_us = to_us_since_boot(get_absolute_time());
//     // sampling time calculation
//     float Ts = (timestamp_us - prev_timestamp_us) * 1e-6f;
//     // quick fix for strange cases (micros overflow)
//     if (Ts <= 0 || Ts > 0.5f)
//         Ts = 1e-3f;

//     // time from last impulse
//     float Th = (timestamp_us - copy_pulse_timestamp) * 1e-6f;
//     long dN = copy_pulse_counter - prev_pulse_counter;

//     // Pulse per second calculation (Eq.3.)
//     // dN - impulses received
//     // Ts - sampling time - time in between function calls
//     // Th - time from last impulse
//     // Th_1 - time form last impulse of the previous call
//     // only increment if some impulses received
//     float dt = Ts + prev_Th - Th;
//     pulse_per_second = (dN != 0 && dt > Ts / 2) ? dN / dt : pulse_per_second;

//     // if more than 0.05f passed in between impulses
//     if (Th > 0.1f)
//         pulse_per_second = 0;

//     // velocity calculation
//     float velocity = pulse_per_second / ((float) cpr) * (2 * M_PI);

//     // save variables for next pass
//     prev_timestamp_us = timestamp_us;
//     // save velocity calculation variables
//     prev_Th = Th;
//     prev_pulse_counter = copy_pulse_counter;
//     return velocity;
// }

int main() {
    watchdog_enable(5000, true);
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    printf("%s\n", FULL_BUILD_TAG);

    // IRQ start here
    // gpio_init(PIN_A);
    // gpio_init(PIN_B);
    // gpio_set_irq_enabled_with_callback(PIN_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    // gpio_set_irq_enabled_with_callback(PIN_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // gpio_init(GPIO_WRITE);
    // gpio_set_dir(GPIO_WRITE, GPIO_OUT);

    // bool value = true;

    encoder enc;
    // sleep_ms(2000);
    encoder_init(&enc, 27, 28, 2048);

    encoder enc2;
    encoder_init(&enc2, 30, 31, 2048);

    while (true) {
        watchdog_update();

        // printf("Hello World!\n");

        // gpio_put(GPIO_WRITE, value);
        // value = !value;

        // printf("%f\n", encoder_get_angle(&enc) * 180 / M_PI);
        printf("%f\n", encoder_get_velocity(&enc));

        sleep_ms(50);
    }
    return 0;
}
