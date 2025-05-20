#include "BLDCDriver3PWM.h"

#include "hardware/gpio.h"
#include "titan/logger.h"

void make_BLDCDriver3PWM(BLDCDRIVER3PWM_t *driver, int phA, int phB, int phC, int en1) {
    // Pin initialization
    driver->pwmA = phA;
    driver->pwmB = phB;
    driver->pwmC = phC;

    // enable_pin pin
    driver->enableA_pin = en1;
    //   enableB_pin = en2;
    //   enableC_pin = en3;

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
    // enable_pin the driver - if enable_pin pin available
    // if (_isset(enableA_pin))
    //     digitalWrite(enableA_pin, enable_active_high);
    // if (_isset(enableB_pin))
    //     digitalWrite(enableB_pin, enable_active_high);
    // if (_isset(enableC_pin))
    //     digitalWrite(enableC_pin, enable_active_high);
    gpio_put(driver->enableA_pin, true);
    // set zero to PWM
    driver_setPwm(driver, 0, 0, 0);
}

// disable motor driver
void diver_disable(BLDCDRIVER3PWM_t *driver) {
    // set zero to PWM
    driver_setPwm(driver, 0, 0, 0);
    // disable the driver - if enable_pin pin available
    // if (_isset(enableA_pin))
    //     digitalWrite(enableA_pin, !enable_active_high);
    // if (_isset(enableB_pin))
    //     digitalWrite(enableB_pin, !enable_active_high);
    // if (_isset(enableC_pin))
    //     digitalWrite(enableC_pin, !enable_active_high);

    gpio_put(driver->enableA_pin, false);
}

// init hardware pins
int driver_init(BLDCDRIVER3PWM_t *driver) {
    // PWM pins
    // pinMode(pwmA, OUTPUT);
    // pinMode(pwmB, OUTPUT);
    // pinMode(pwmC, OUTPUT);
    gpio_init(driver->enableA_pin);
    gpio_set_dir(driver->enableA_pin, GPIO_OUT);
    // if (_isset(enableA_pin))
    //     pinMode(enableA_pin, OUTPUT);
    // if (_isset(enableB_pin))
    //     pinMode(enableB_pin, OUTPUT);
    // if (_isset(enableC_pin))
    //     pinMode(enableC_pin, OUTPUT);

    // sanity check for the voltage limit configuration
    // if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    //     voltage_limit = voltage_power_supply;
    if (driver->voltage_limit < 0.01 || driver->voltage_limit > driver->voltage_power_supply)
        driver->voltage_limit = driver->voltage_power_supply;

    // Set the pwm frequency to the pins
    // hardware specific function - depending on driver and mcu
    driver->params = _configure3PWM(driver->pwm_frequency, driver->pwmA, driver->pwmB, driver->pwmC);
    driver->initialized = (driver->params != SIMPLEFOC_DRIVER_INIT_FAILED);
    return driver->params != SIMPLEFOC_DRIVER_INIT_FAILED;
}

// Set voltage to the pwm pin
void driver_setPhaseState(BLDCDRIVER3PWM_t *driver, PhaseState_t sa, PhaseState_t sb, PhaseState_t sc) {
    // disable if needed
    // if (_isset(enableA_pin) && _isset(enableB_pin) && _isset(enableC_pin)) {
    gpio_put(driver->enableA_pin, sa == PHASE_ON ? driver->enable_active_high : !driver->enable_active_high);
    // digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
    // digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
    // }
}

float _constrain(float val, float min, float max) {
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}

// Set voltage to the pwm pin
void driver_setPwm(BLDCDRIVER3PWM_t *driver, float Ua, float Ub, float Uc) {
    // limit the voltage in driver
    Ua = _constrain(Ua, 0.0f, driver->voltage_limit);
    Ub = _constrain(Ub, 0.0f, driver->voltage_limit);
    Uc = _constrain(Uc, 0.0f, driver->voltage_limit);
    // calculate duty cycle
    // limited in [0,1]
    driver->dc_a = _constrain(Ua / driver->voltage_power_supply, 0.0f, 1.0f);
    driver->dc_b = _constrain(Ub / driver->voltage_power_supply, 0.0f, 1.0f);
    driver->dc_c = _constrain(Uc / driver->voltage_power_supply, 0.0f, 1.0f);

    // LOG_INFO("Got phase duty cycles of %f, %f, %f", driver->dc_a, driver->dc_b, driver->dc_c);

    // hardware specific writing
    // hardware specific function - depending on driver and mcu
    _writeDutyCycle3PWM(driver->dc_a, driver->dc_b, driver->dc_c, driver->params);
}
