#include "core1.h"

#include "fixedptc.h"
#include "safety_interface.h"

#include "driver/drv8313.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "titan/logger.h"

//
// TODO: Prossible remove hardware gpio include
//

#define NUM_POLE_PAIRS 11
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
    fixedpt rps[NUM_MOTORS];

    /**
     * @brief The voltage limit for each motor. Computed by the internal controller
     */
    fixedpt voltage_limit[NUM_MOTORS];

    /**
     * @brief The time after which the target RPS command is considered stale, and the thrusters should be disabled.
     * Prevents runaway robots.
     */
    absolute_time_t rps_expiration;
} target_req = { 0 };

// Declare these out here to not fill up the stack with them
BLDCMotor_t motors[NUM_MOTORS];
BLDCDRIVER3PWM_t drivers[NUM_MOTORS];

const fixedpt motor_inversions[NUM_MOTORS] = { fixedpt_fromint(-1), fixedpt_fromint(1) };

static void motor_make(uint idx, uint p0_pin, uint p1_pin, uint p2_pin, uint en_pin) {
    make_BLDCMotor(&motors[idx], NUM_POLE_PAIRS);
    make_BLDCDriver3PWM(&drivers[idx], p0_pin, p1_pin, p2_pin, en_pin);

    drivers[idx].voltage_power_supply = DRIVER_VOLTAGE_SUPPLY_FIXEDPT;
    drivers[idx].voltage_limit = DRIVER_VOLTAGE_SUPPLY_FIXEDPT;
    driver_init(&drivers[idx]);
    motors[idx].driver = &drivers[idx];  // Link driver

    motors[idx].voltage_limit = DRIVER_VOLTAGE_SUPPLY_FIXEDPT;
    motors[idx].controller = velocity_openloop;

    motor_init(&motors[idx]);
}

static void volatile_copy(volatile fixedpt *dest, const volatile fixedpt *target, size_t n) {
    for (size_t i = 0; i < n; i++)
        dest[i] = target[i];
}

static void __time_critical_func(core1_main)() {
    motor_make(0, ESC0_1_PIN, ESC0_2_PIN, ESC0_3_PIN, MOTOR_ENABLE_PIN);
    motor_make(1, ESC1_1_PIN, ESC1_2_PIN, ESC1_3_PIN, MOTOR_ENABLE_PIN);

    while (1) {
        safety_core1_checkin();

        // This caching has to happen under lock
        uint32_t irq = spin_lock_blocking(target_req.lock);
        fixedpt target_rps_cached[NUM_MOTORS];
        volatile_copy(target_rps_cached, target_req.rps, NUM_MOTORS);
        fixedpt voltage_limit_cached[NUM_MOTORS];
        volatile_copy(voltage_limit_cached, target_req.voltage_limit, NUM_MOTORS);
        spin_unlock(target_req.lock, irq);

        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].voltage_limit = voltage_limit_cached[i];
            motor_move(&motors[i], fixedpt_mul(target_rps_cached[i], motor_inversions[i]));
        }
    }
}

void core1_init() {
    target_req.lock = spin_lock_init(spin_lock_claim_unused(true));
    safety_launch_core1(core1_main);
}

void core1_update_targets(const fixedpt *rps, const fixedpt *voltage_limit) {
    // Compute expiration outside of spin lock to avoid unnecessary work in critical section.
    // absolute_time_t expiration = make_timeout_time_ms(DSHOT_MIN_UPDATE_RATE_MS);

    // Update shared variables under lock
    uint32_t irq = spin_lock_blocking(target_req.lock);
    // target_req.rpm_expiration = expiration;
    volatile_copy(target_req.rps, rps, NUM_MOTORS);
    volatile_copy(target_req.voltage_limit, voltage_limit, NUM_MOTORS);
    spin_unlock(target_req.lock, irq);
}
