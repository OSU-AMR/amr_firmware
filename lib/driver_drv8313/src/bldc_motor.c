#include "bldc_motor.h"

#include "bldc_driver_3pwm.h"
#include "fixedptc.h"

#include "hardware/timer.h"
#include "pico/time.h"
#include "titan/logger.h"

#include <math.h>

#define _SQRT3_2_FIXEDPT fixedpt_rconst(0.86602540378f)
#define _2PI_FIXEDPT fixedpt_rconst(6.28318530718f)
#define _PI_2_FIXEDPT fixedpt_rconst(1.57079632679f)

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - L             - motor phase inductance
void make_BLDCMotor(BLDCMotor_t *motor, int pp) {
    // save pole pairs number
    motor->pole_pairs = pp;

    // torque control type is voltage by default
    motor->torque_controller = voltage;

    motor->motor_status = motor_uninitialized;
    motor->enabled = 0;
    motor->modulation_centered = 1;
    motor->feed_forward_velocity = 0.0f;
    motor->sensor_direction = UNKNOWN;

    // maximum angular velocity to be used for positioning
    motor->velocity_limit = fixedpt_rconst(20.0f);
    // maximum voltage to be set to the motor
    motor->voltage_limit = fixedpt_rconst(12.0f);
    // not set on the begining
    motor->current_limit = fixedpt_rconst(2.0f);

    // index search velocity
    motor->velocity_index_search = fixedpt_rconst(1.0f);
    // sensor and motor align voltage
    motor->voltage_sensor_align = fixedpt_rconst(3.0f);

    // default modulation is SinePWM
    motor->foc_modulation = SinePWM;

    // default target value
    motor->target = 0;
    motor->voltage.d = 0;
    motor->voltage.q = 0;
    // current target values
    motor->current_sp = 0;
    motor->current.q = 0;
    motor->current.d = 0;

    // voltage bemf
    motor->voltage_bemf = fixedpt_rconst(0.0f);

    // Initialize phase voltages U alpha and U beta used for inverse Park and Clarke transform
    motor->Ualpha = 0;
    motor->Ubeta = 0;

    motor->shaft_angle = fixedpt_rconst(0.0f);

    // New additions for C
    motor->shaft_velocity_sp = fixedpt_rconst(0.0f);
    motor->open_loop_timestamp = 0l;
}

/**
    Link the driver which controls the motor
*/
void motor_linkDriver(BLDCMotor_t *motor, BLDCDRIVER3PWM_t *_driver) {
    motor->driver = _driver;
}

// init hardware pins
int motor_init(BLDCMotor_t *motor) {
    if (!motor->driver || !motor->driver->initialized) {
        motor->motor_status = motor_init_failed;
        // SIMPLEFOC_DEBUG("MOT: Init not possible, driver not initialized");
        return 0;
    }
    motor->motor_status = motor_initializing;
    LOG_INFO("Initializing motor");

    // sanity check for the voltage limit configuration
    if (motor->voltage_limit > motor->driver->voltage_limit)
        motor->voltage_limit = motor->driver->voltage_limit;
    // constrain voltage for sensor alignment
    if (motor->voltage_sensor_align > motor->voltage_limit)
        motor->voltage_sensor_align = motor->voltage_limit;

    // if using open loop control, set a CW as the default direction if not already set
    if ((motor->controller == angle_openloop || motor->controller == velocity_openloop) &&
        (motor->sensor_direction == UNKNOWN)) {
        motor->sensor_direction = CW;
    }

    // enable motor
    LOG_INFO("Enabling driver");
    motor_enable(motor);
    motor->motor_status = motor_uncalibrated;
    return 1;
}

// disable motor driver
void motor_disable(BLDCMotor_t *motor) {
    // set zero to PWM
    driver_setPwm(motor->driver, 0, 0, 0);
    // disable the driver
    driver_disable(motor->driver);
    // motor status update
    motor->enabled = 0;
}
// enable motor driver
void motor_enable(BLDCMotor_t *motor) {
    // enable the driver
    driver_enable(motor->driver);
    // set zero to PWM
    driver_setPwm(motor->driver, 0, 0, 0);

    // motor status update
    motor->enabled = 1;
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void motor_move(BLDCMotor_t *motor, fixedpt new_target) {
    // set internal target variable
    motor->target = new_target;

    // if disabled do nothing
    if (!motor->enabled)
        return;

    // velocity control in open loop - sensor precision: this calculation is numerically precise.
    motor->shaft_velocity_sp = motor->target;
    motor->voltage.q =
        motor_velocityOpenloop(motor, motor->shaft_velocity_sp);  // returns the voltage that is set to the motor
    motor->voltage.d = 0;
}

void _sincos(fixedpt a, fixedpt *s, fixedpt *c) {
    *s = fixedpt_sin(a);
    *c = fixedpt_cos(a);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void motor_setPhaseVoltage(BLDCMotor_t *motor, fixedpt Uq, fixedpt Ud, fixedpt angle_el) {
    fixedpt center;
    fixedpt _ca, _sa;

    // Sinusoidal PWM modulation
    // Inverse Park + Clarke transformation
    _sincos(angle_el, &_sa, &_ca);

    // Inverse park transform
    // motor->Ualpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
    // motor->Ubeta = _sa * Ud + _ca * Uq;   //  cos(angle) * Uq;

    motor->Ualpha = fixedpt_mul(_ca, Ud) - fixedpt_mul(_sa, Uq);
    motor->Ubeta = fixedpt_mul(_sa, Ud) + fixedpt_mul(_ca, Uq);

    // Clarke transform
    // motor->Ua = motor->Ualpha;
    // motor->Ub = -0.5f * motor->Ualpha + _SQRT3_2 * motor->Ubeta;
    // motor->Uc = -0.5f * motor->Ualpha - _SQRT3_2 * motor->Ubeta;

    motor->Ua = motor->Ualpha;
    motor->Ub = fixedpt_mul(fixedpt_rconst(-0.5f), motor->Ualpha) + fixedpt_mul(_SQRT3_2_FIXEDPT, motor->Ubeta);
    motor->Uc = fixedpt_mul(fixedpt_rconst(-0.5f), motor->Ualpha) - fixedpt_mul(_SQRT3_2_FIXEDPT, motor->Ubeta);

    // center = motor->driver->voltage_limit / 2;

    center = fixedpt_div(motor->driver->voltage_limit, fixedpt_fromint(2));

    // if (!motor->modulation_centered) {
    //     float Umin = min(motor->Ua, min(motor->Ub, motor->Uc));
    //     motor->Ua -= Umin;
    //     motor->Ub -= Umin;
    //     motor->Uc -= Umin;
    // }
    // else {
    motor->Ua += center;
    motor->Ub += center;
    motor->Uc += center;
    // }

    // set the voltages in driver
    driver_setPwm(motor->driver, motor->Ua, motor->Ub, motor->Uc);
}

// normalizing radian angle to [0,2PI]
fixedpt _normalizeAngle(fixedpt angle) {
    fixedpt a = angle % _2PI_FIXEDPT;
    return a >= 0 ? a : (a + _2PI_FIXEDPT);
}

fixedpt _electricalAngle(fixedpt shaft_angle, fixedpt pole_pairs) {
    return fixedpt_mul(shaft_angle, pole_pairs);
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
fixedpt motor_velocityOpenloop(BLDCMotor_t *motor, fixedpt target_velocity) {
    // get current timestamp
    unsigned long now_us = to_us_since_boot(get_absolute_time());
    // calculate the sample time from last call
    fixedpt Ts = fixedpt_mul(fixedpt_fromint(now_us - motor->open_loop_timestamp), fixedpt_rconst(1e-6f));
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if (Ts <= 0 || Ts > fixedpt_rconst(0.5f))
        Ts = fixedpt_rconst(1e-3f);

    // calculate the necessary angle to achieve target velocity
    motor->shaft_angle = _normalizeAngle(motor->shaft_angle + fixedpt_mul(target_velocity, Ts));

    // for display purposes
    motor->shaft_velocity = target_velocity;

    // use voltage limit or current limit
    fixedpt Uq = motor->voltage_limit;

    // set the maximal allowed voltage (voltage_limit) with the necessary angle
    motor_setPhaseVoltage(motor, Uq, 0, _electricalAngle(motor->shaft_angle, fixedpt_fromint(motor->pole_pairs)));

    // save timestamp for next call
    motor->open_loop_timestamp = now_us;

    return Uq;
}
