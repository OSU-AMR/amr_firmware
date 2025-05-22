#include "core1.h"

#include "safety_interface.h"

#include "driver/drv8313.h"
#include "hardware/gpio.h"

//
// TODO: Prossible remove hardware gpio include
//

#define PICO_LED_PIN 25

static void busy_sleep_ms(uint32_t ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < ms) {
        safety_core1_checkin();
    }
}

static void __time_critical_func(core1_main)() {
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

    // Test if we can commutate two motors at a time
    BLDCMotor_t motor2;
    make_BLDCMotor(&motor2, 11);

    BLDCDRIVER3PWM_t driver2;
    make_BLDCDriver3PWM(&driver2, 0, 1, 2, 3);

    driver2.voltage_power_supply = 16;
    driver2.voltage_limit = 3;
    driver_init(&driver2);
    motor2.driver = &driver2;  // Link driver

    motor2.voltage_limit = 3;
    motor2.controller = velocity_openloop;  // Doesn't actually matter (I think)

    motor_init(&motor2);

    // sleep_ms(1000);

    // bool val = false;

    while (1) {
        safety_core1_checkin();

        // gpio_put(PICO_LED_PIN, val);
        // val = !val;
        // sleep_ms(100);

        motor_move(&motor, 35);
        motor_move(&motor2, 10);
    }
}

void core1_init() {
    safety_launch_core1(core1_main);

    // gpio_init(PICO_LED_PIN);
    // gpio_put(PICO_LED_PIN, 0);
    // gpio_set_dir(PICO_LED_PIN, GPIO_OUT);
}

// #include "core1.h"
// #include "safety_interface.h"

// #include "hardware/gpio.h"

// //
// // TODO: Prossible remove hardware gpio include
// //

// #define PICO_LED_PIN 25

// static void __time_critical_func(core1_main)() {
//     bool val = false;

//     while (1) {
//         safety_core1_checkin();

//         gpio_put(PICO_LED_PIN, val);
//         val = !val;

//         sleep_ms(100);
//     }
// }

// void core1_init() {
//     safety_launch_core1(core1_main);
// }
