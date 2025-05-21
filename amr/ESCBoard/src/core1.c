#include "core1.h"

#include "safety_interface.h"

#include "hardware/gpio.h"

//
// TODO: Prossible remove hardware gpio include
//

#define PICO_LED_PIN 25

static void __time_critical_func(core1_main)() {
    bool val = false;

    while (1) {
        safety_core1_checkin();

        gpio_put(PICO_LED_PIN, val);
        val = !val;

        sleep_ms(100);
    }
}

void core1_init() {
    safety_launch_core1(core1_main);

    gpio_init(PICO_LED_PIN);
    gpio_put(PICO_LED_PIN, 0);
    gpio_set_dir(PICO_LED_PIN, GPIO_OUT);
}
