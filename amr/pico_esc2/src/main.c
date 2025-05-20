#include "simplefoc/BLDCMotor.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/version.h"

#include <stdio.h>

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

int main() {
    watchdog_enable(5000, true);
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    printf("%s\n", FULL_BUILD_TAG);

    // bool value = true;

    BLDCMotor_t motor;
    make_BLDCMotor(&motor, 11);

    BLDCDRIVER3PWM_t driver;
    make_BLDCDriver3PWM(&driver, 10, 11, 12, 13);

    driver.voltage_power_supply = 16;
    driver.voltage_limit = 3;
    driver_init(&driver);
    motor.driver = &driver;  // Link driver

    motor.voltage_limit = 3;
    motor.controller = velocity_openloop;  // Doesn't actually matter (I think)

    motor_init(&motor);
    sleep_ms(1000);

    while (true) {
        watchdog_update();

        motor_move(&motor, 5);

        // printf("Hello World!\n");

        // gpio_put(LED_PIN, value);
        // value = !value;

        // sleep_ms(1000);
    }
    return 0;
}
