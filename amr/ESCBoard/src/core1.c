#include "core1.h"

#include "safety_interface.h"

#include "driver/drv8313.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"

//
// TODO: Prossible remove hardware gpio include
//

#define PICO_LED_PIN 25

#define NUM_POLE_PAIRS 11
#define DRIVER_VOLTAGE_SUPPLY 16
#define DRIVER_VOLTAGE_LIMIT 3
#define MOTOR_VOLTAGE_LIMIT 3

/**
 * @brief Shared memory for sending commands from core 0 to core 1.
 *
 * @attention This can only be modified or read when lock is held
 */
static volatile struct core1_cmd_shared_mem {
    /**
     * @brief Spin lock to protect command state transferring across cores
     */
    spin_lock_t *lock;

    /**
     * @brief The target RPS for the controller. Provided by ROS.
     */
    int16_t rps[NUM_MOTORS];

    /**
     * @brief The time after which the target RPS command is considered stale, and the thrusters should be disabled.
     * Prevents runaway robots.
     */
    absolute_time_t rps_expiration;
} target_req = { 0 };

// Declare these out here to not fill up the stack with them
BLDCMotor_t motors[NUM_MOTORS];
BLDCDRIVER3PWM_t drivers[NUM_MOTORS];

const float motor_inversions[NUM_MOTORS] = { 1.0f, -1.0f };

static void motor_make(uint idx, uint p0_pin, uint p1_pin, uint p2_pin, uint en_pin) {
    make_BLDCMotor(&motors[idx], NUM_POLE_PAIRS);
    make_BLDCDriver3PWM(&drivers[idx], p0_pin, p1_pin, p2_pin, en_pin);

    drivers[idx].voltage_power_supply = DRIVER_VOLTAGE_SUPPLY;
    drivers[idx].voltage_limit = DRIVER_VOLTAGE_LIMIT;
    driver_init(&drivers[idx]);
    motors[idx].driver = &drivers[idx];  // Link driver

    motors[idx].voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motors[idx].controller = velocity_openloop;

    motor_init(&motors[idx]);
}

static void __time_critical_func(core1_main)() {
    motor_make(0, 10, 11, 12, 13);
    motor_make(1, 0, 1, 2, 3);

    // sleep_ms(1000);

    // bool val = false;

    while (1) {
        safety_core1_checkin();

        // gpio_put(PICO_LED_PIN, val);
        // val = !val;
        // sleep_ms(100);

        // TODO: This caching needs to happen under lock
        int16_t rps_cached[NUM_MOTORS];

        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_move(&motors[i], rps_cached[i]);
        }
    }
}

void core1_init() {
    target_req.lock = spin_lock_init(spin_lock_claim_unused(true));
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
