#include "bldc_motor.h"

#include "bldc_driver_3pwm.h"

#include "hardware/timer.h"
#include "pico/time.h"
#include "titan/logger.h"

#include <math.h>

#define _SQRT3_2 0.86602540378f
#define _2PI 6.28318530718f
#define _PI_2 1.57079632679f

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
    motor->velocity_limit = 20.0f;
    // maximum voltage to be set to the motor
    motor->voltage_limit = 12.0f;
    // not set on the begining
    motor->current_limit = 2.0f;

    // index search velocity
    motor->velocity_index_search = 1.0f;
    // sensor and motor align voltage
    motor->voltage_sensor_align = 3.0f;

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
    motor->voltage_bemf = 0.0f;

    // Initialize phase voltages U alpha and U beta used for inverse Park and Clarke transform
    motor->Ualpha = 0;
    motor->Ubeta = 0;

    motor->shaft_angle = 0.0f;

    // New additions for C
    motor->shaft_velocity_sp = 0.0f;
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

    sleep_ms(500);
    // enable motor
    LOG_INFO("Enabling driver");
    motor_enable(motor);
    sleep_ms(500);
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
void motor_move(BLDCMotor_t *motor, float new_target) {
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

float min(float val1, float val2) {
    return val1 < val2 ? val1 : val2;
}

float _sin(float a) {
    // 16bit integer array for sine lookup. interpolation is used for better precision
    // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
    // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
    static uint16_t sine_array[65] = { 0,     804,   1608,  2411,  3212,  4011,  4808,  5602,  6393,  7180,  7962,
                                       8740,  9512,  10279, 11039, 11793, 12540, 13279, 14010, 14733, 15447, 16151,
                                       16846, 17531, 18205, 18868, 19520, 20160, 20788, 21403, 22006, 22595, 23170,
                                       23732, 24279, 24812, 25330, 25833, 26320, 26791, 27246, 27684, 28106, 28511,
                                       28899, 29269, 29622, 29957, 30274, 30572, 30853, 31114, 31357, 31581, 31786,
                                       31972, 32138, 32286, 32413, 32522, 32610, 32679, 32729, 32758, 32768 };
    int32_t t1, t2;
    unsigned int i = (unsigned int) (a * (64 * 4 * 256.0f / _2PI));
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;
    if (i < 64) {
        t1 = (int32_t) sine_array[i];
        t2 = (int32_t) sine_array[i + 1];
    }
    else if (i < 128) {
        t1 = (int32_t) sine_array[128 - i];
        t2 = (int32_t) sine_array[127 - i];
    }
    else if (i < 192) {
        t1 = -(int32_t) sine_array[-128 + i];
        t2 = -(int32_t) sine_array[-127 + i];
    }
    else {
        t1 = -(int32_t) sine_array[256 - i];
        t2 = -(int32_t) sine_array[255 - i];
    }
    return (1.0f / 32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}

// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a) {
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}

void _sincos(float a, float *s, float *c) {
    *s = _sin(a);
    *c = _cos(a);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void motor_setPhaseVoltage(BLDCMotor_t *motor, float Uq, float Ud, float angle_el) {
    float center;
    float _ca, _sa;

    // Sinusoidal PWM modulation
    // Inverse Park + Clarke transformation
    _sincos(angle_el, &_sa, &_ca);

    // Inverse park transform
    motor->Ualpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
    motor->Ubeta = _sa * Ud + _ca * Uq;   //  cos(angle) * Uq;

    // Clarke transform
    motor->Ua = motor->Ualpha;
    motor->Ub = -0.5f * motor->Ualpha + _SQRT3_2 * motor->Ubeta;
    motor->Uc = -0.5f * motor->Ualpha - _SQRT3_2 * motor->Ubeta;

    center = motor->driver->voltage_limit / 2;

    if (!motor->modulation_centered) {
        float Umin = min(motor->Ua, min(motor->Ub, motor->Uc));
        motor->Ua -= Umin;
        motor->Ub -= Umin;
        motor->Uc -= Umin;
    }
    else {
        motor->Ua += center;
        motor->Ub += center;
        motor->Uc += center;
    }

    // set the voltages in driver
    driver_setPwm(motor->driver, motor->Ua, motor->Ub, motor->Uc);
}

// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle) {
    float a = fmod(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

float _electricalAngle(float shaft_angle, int pole_pairs) {
    return (shaft_angle * pole_pairs);
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float motor_velocityOpenloop(BLDCMotor_t *motor, float target_velocity) {
    // get current timestamp
    unsigned long now_us = to_us_since_boot(get_absolute_time());
    // calculate the sample time from last call
    float Ts = (now_us - motor->open_loop_timestamp) * 1e-6f;
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // calculate the necessary angle to achieve target velocity
    motor->shaft_angle = _normalizeAngle(motor->shaft_angle + target_velocity * Ts);
    // LOG_INFO("Got angle as %f", motor->shaft_angle);

    // for display purposes
    motor->shaft_velocity = target_velocity;

    // use voltage limit or current limit
    float Uq = motor->voltage_limit;

    // set the maximal allowed voltage (voltage_limit) with the necessary angle
    motor_setPhaseVoltage(motor, Uq, 0, _electricalAngle(motor->shaft_angle, motor->pole_pairs));

    // save timestamp for next call
    motor->open_loop_timestamp = now_us;

    return Uq;
}
