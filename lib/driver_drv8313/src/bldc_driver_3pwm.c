#include "bldc_driver_3pwm.h"

#include "fixedptc.h"

#include "hardware/gpio.h"
#include "titan/logger.h"

void make_BLDCDriver3PWM(BLDCDRIVER3PWM_t *driver, int phA, int phB, int phC, int en1) {
    // Pin initialization
    driver->pwmA = phA;
    driver->pwmB = phB;
    driver->pwmC = phC;

    // enable_pin pin
    driver->enableA_pin = en1;

    // default power-supply value
    driver->voltage_power_supply = 12.0f;
    driver->voltage_limit = -12345.0f;
    driver->pwm_frequency = -12345.0f;

    driver->initialized = false;
    driver->params = 0;
    driver->enable_active_high = false;
}

// enable motor driver
void driver_enable(BLDCDRIVER3PWM_t *driver) {
    gpio_put(driver->enableA_pin, true);

    // set zero to PWM
    driver_setPwm(driver, 0, 0, 0);
}

// disable motor driver
void diver_disable(BLDCDRIVER3PWM_t *driver) {
    // set zero to PWM
    driver_setPwm(driver, 0, 0, 0);

    gpio_put(driver->enableA_pin, false);
}

// init hardware pins
int driver_init(BLDCDRIVER3PWM_t *driver) {
    // PWM pins
    gpio_init(driver->enableA_pin);
    gpio_set_dir(driver->enableA_pin, GPIO_OUT);

    // sanity check for the voltage limit configuration
    if (driver->voltage_limit < fixedpt_rconst(0.01f) || driver->voltage_limit > driver->voltage_power_supply)
        driver->voltage_limit = driver->voltage_power_supply;

    // Set the pwm frequency to the pins
    // hardware specific function - depending on driver and mcu
    driver->params = _configure3PWM(driver->pwm_frequency, driver->pwmA, driver->pwmB, driver->pwmC);
    driver->initialized = (driver->params != SIMPLEFOC_DRIVER_INIT_FAILED);
    return driver->params != SIMPLEFOC_DRIVER_INIT_FAILED;
}

// Set voltage to the pwm pin
void driver_setPhaseState(BLDCDRIVER3PWM_t *driver, __unused PhaseState_t sa, __unused PhaseState_t sb,
                          __unused PhaseState_t sc) {
    gpio_put(driver->enableA_pin, sa == PHASE_ON ? driver->enable_active_high : !driver->enable_active_high);
}

fixedpt _constrain(fixedpt val, fixedpt min, fixedpt max) {
    return MIN(MAX(val, min), max);
}

// Set voltage to the pwm pin
void driver_setPwm(BLDCDRIVER3PWM_t *driver, fixedpt Ua, fixedpt Ub, fixedpt Uc) {
    // limit the voltage in driver
    Ua = _constrain(Ua, 0, driver->voltage_limit);
    Ub = _constrain(Ub, 0, driver->voltage_limit);
    Uc = _constrain(Uc, 0, driver->voltage_limit);
    // calculate duty cycle
    // limited in [0,1]
    driver->dc_a = _constrain(fixedpt_div(Ua, driver->voltage_power_supply), 0, fixedpt_fromint(1));
    driver->dc_b = _constrain(fixedpt_div(Ub, driver->voltage_power_supply), 0, fixedpt_fromint(1));
    driver->dc_c = _constrain(fixedpt_div(Uc, driver->voltage_power_supply), 0, fixedpt_fromint(1));

    // hardware specific writing
    // hardware specific function - depending on driver and mcu
    _writeDutyCycle3PWM(driver->dc_a, driver->dc_b, driver->dc_c, driver->params);
}
