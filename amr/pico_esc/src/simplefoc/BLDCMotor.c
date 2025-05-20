#include "BLDCMotor.h"

#include "BLDCDriver3PWM.h"

#include "hardware/timer.h"
#include "pico/time.h"
#include "titan/logger.h"

#include <math.h>

#define _SQRT3_2 0.86602540378f
#define _2PI 6.28318530718f

// #include "./communication/SimpleFOCDebug.h"

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
// int trap_120_map[6][3] = { { _HIGH_IMPEDANCE, 1, -1 }, { -1, 1, _HIGH_IMPEDANCE }, { -1, _HIGH_IMPEDANCE, 1 },
//                            { _HIGH_IMPEDANCE, -1, 1 }, { 1, -1, _HIGH_IMPEDANCE }, { 1, _HIGH_IMPEDANCE, -1 } };

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
// int trap_150_map[12][3] = { { _HIGH_IMPEDANCE, 1, -1 }, { -1, 1, -1 }, { -1, 1, _HIGH_IMPEDANCE }, { -1, 1, 1 },
//                             { -1, _HIGH_IMPEDANCE, 1 }, { -1, -1, 1 }, { _HIGH_IMPEDANCE, -1, 1 }, { 1, -1, 1 },
//                             { 1, -1, _HIGH_IMPEDANCE }, { 1, -1, -1 }, { 1, _HIGH_IMPEDANCE, -1 }, { 1, 1, -1 } };

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - L             - motor phase inductance
void make_BLDCMotor(BLDCMotor_t *motor, int pp) {
    // save pole pairs number
    motor->pole_pairs = pp;
    // save phase resistance number
    // phase_resistance = _R;
    // save back emf constant KV = 1/KV
    // 1/sqrt(2) - rms value
    // KV_rating = NOT_SET;
    // if (_isset(_KV))
    //     KV_rating = _KV;
    // save phase inductance
    // phase_inductance = _inductance;

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

    // monitor_port
    // motor->monitor_port = nullptr;
    // sensor
    // motor->sensor_offset = 0.0f;
    // motor->sensor = nullptr;
    // current sensor
    // current_sense = nullptr;
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
    // SIMPLEFOC_DEBUG("MOT: Init");
    LOG_INFO("Initializing motor");

    // sanity check for the voltage limit configuration
    if (motor->voltage_limit > motor->driver->voltage_limit)
        motor->voltage_limit = motor->driver->voltage_limit;
    // constrain voltage for sensor alignment
    if (motor->voltage_sensor_align > motor->voltage_limit)
        motor->voltage_sensor_align = motor->voltage_limit;

    // update the controller limits
    // if (motor->current_sense) {
    //     // current control loop controls voltage
    //     PID_current_q.limit = voltage_limit;
    //     PID_current_d.limit = voltage_limit;
    // }
    // if (_isset(phase_resistance) || torque_controller != TorqueControlType::voltage) {
    //     // velocity control loop controls current
    //     PID_velocity.limit = current_limit;
    // }
    // else {
    //     // velocity control loop controls the voltage
    //     PID_velocity.limit = voltage_limit;
    // }
    // P_angle.limit = velocity_limit;

    // if using open loop control, set a CW as the default direction if not already set
    if ((motor->controller == angle_openloop || motor->controller == velocity_openloop) &&
        (motor->sensor_direction == UNKNOWN)) {
        motor->sensor_direction = CW;
    }

    // _delay(500);
    sleep_ms(500);
    // enable motor
    // SIMPLEFOC_DEBUG("MOT: Enable driver.");
    LOG_INFO("Enabling driver");
    motor_enable(motor);
    // _delay(500);
    sleep_ms(500);
    motor->motor_status = motor_uncalibrated;
    return 1;
}

// disable motor driver
void motor_disable(BLDCMotor_t *motor) {
    // disable the current sense
    // if (motor->current_sense)
    //     current_sense->disable();
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
    // enable the current sense
    // if (current_sense)
    //     current_sense->enable();
    // reset the pids
    // PID_velocity.reset();
    // P_angle.reset();
    // PID_current_q.reset();
    // PID_current_d.reset();
    // motor status update
    motor->enabled = 1;
}

/**
  FOC functions
*/
// FOC initialization function
int motor_initFOC(BLDCMotor_t *motor) {
    // int exit_flag = 1;

    // motor->motor_status = FOCMotorStatus::motor_calibrating;

    // // align motor if necessary
    // // alignment necessary for encoders!
    // // sensor and motor alignment - can be skipped
    // // by setting motor.sensor_direction and motor.zero_electric_angle
    // if (sensor) {
    //     exit_flag *= alignSensor();
    //     // added the shaft_angle update
    //     sensor->update();
    //     shaft_angle = shaftAngle();

    //     // aligning the current sensor - can be skipped
    //     // checks if driver phases are the same as current sense phases
    //     // and checks the direction of measuremnt.
    //     if (exit_flag) {
    //         if (current_sense) {
    //             if (!current_sense->initialized) {
    //                 motor_status = FOCMotorStatus::motor_calib_failed;
    //                 SIMPLEFOC_DEBUG("MOT: Init FOC error, current sense not initialized");
    //                 exit_flag = 0;
    //             }
    //             else {
    //                 exit_flag *= alignCurrentSense();
    //             }
    //         }
    //         else {
    //             SIMPLEFOC_DEBUG("MOT: No current sense.");
    //         }
    //     }
    // }
    // else {
    //     SIMPLEFOC_DEBUG("MOT: No sensor.");
    //     if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop))
    //     {
    //         exit_flag = 1;
    //         SIMPLEFOC_DEBUG("MOT: Openloop only!");
    //     }
    //     else {
    //         exit_flag = 0;  // no FOC without sensor
    //     }
    // }

    // if (exit_flag) {
    //     SIMPLEFOC_DEBUG("MOT: Ready.");
    //     motor_status = FOCMotorStatus::motor_ready;
    // }
    // else {
    //     SIMPLEFOC_DEBUG("MOT: Init FOC failed.");
    //     motor_status = FOCMotorStatus::motor_calib_failed;
    //     disable();
    // }

    // return exit_flag;
}

// Calibarthe the motor and current sense phases
int motor_alignCurrentSense(BLDCMotor_t *motor) {
    // int exit_flag = 1;  // success

    // SIMPLEFOC_DEBUG("MOT: Align current sense.");

    // // align current sense and the driver
    // exit_flag = current_sense->driverAlign(voltage_sensor_align, modulation_centered);
    // if (!exit_flag) {
    //     // error in current sense - phase either not measured or bad connection
    //     SIMPLEFOC_DEBUG("MOT: Align error!");
    //     exit_flag = 0;
    // }
    // else {
    //     // output the alignment status flag
    //     SIMPLEFOC_DEBUG("MOT: Success: ", exit_flag);
    // }

    // return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int motor_alignSensor(BLDCMotor_t *motor) {
    // int exit_flag = 1;  // success
    // SIMPLEFOC_DEBUG("MOT: Align sensor.");

    // // check if sensor needs zero search
    // if (sensor->needsSearch())
    //     exit_flag = absoluteZeroSearch();
    // // stop init if not found index
    // if (!exit_flag)
    //     return exit_flag;

    // // v2.3.3 fix for R_AVR_7_PCREL against symbol" bug for AVR boards
    // // TODO figure out why this works
    // float voltage_align = voltage_sensor_align;

    // // if unknown natural direction
    // if (sensor_direction == Direction::UNKNOWN) {
    //     // find natural direction
    //     // move one electrical revolution forward
    //     for (int i = 0; i <= 500; i++) {
    //         float angle = _3PI_2 + _2PI * i / 500.0f;
    //         setPhaseVoltage(voltage_align, 0, angle);
    //         sensor->update();
    //         _delay(2);
    //     }
    //     // take and angle in the middle
    //     sensor->update();
    //     float mid_angle = sensor->getAngle();
    //     // move one electrical revolution backwards
    //     for (int i = 500; i >= 0; i--) {
    //         float angle = _3PI_2 + _2PI * i / 500.0f;
    //         setPhaseVoltage(voltage_align, 0, angle);
    //         sensor->update();
    //         _delay(2);
    //     }
    //     sensor->update();
    //     float end_angle = sensor->getAngle();
    //     // setPhaseVoltage(0, 0, 0);
    //     _delay(200);
    //     // determine the direction the sensor moved
    //     float moved = fabs(mid_angle - end_angle);
    //     if (moved < MIN_ANGLE_DETECT_MOVEMENT) {  // minimum angle to detect movement
    //         SIMPLEFOC_DEBUG("MOT: Failed to notice movement");
    //         return 0;  // failed calibration
    //     }
    //     else if (mid_angle < end_angle) {
    //         SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW");
    //         sensor_direction = Direction::CCW;
    //     }
    //     else {
    //         SIMPLEFOC_DEBUG("MOT: sensor_direction==CW");
    //         sensor_direction = Direction::CW;
    //     }
    //     // check pole pair number
    //     pp_check_result =
    //         !(fabs(moved * pole_pairs - _2PI) > 0.5f);  // 0.5f is arbitrary number it can be lower or higher!
    //     if (pp_check_result == false) {
    //         SIMPLEFOC_DEBUG("MOT: PP check: fail - estimated pp: ", _2PI / moved);
    //     }
    //     else {
    //         SIMPLEFOC_DEBUG("MOT: PP check: OK!");
    //     }
    // }
    // else {
    //     SIMPLEFOC_DEBUG("MOT: Skip dir calib.");
    // }

    // // zero electric angle not known
    // if (!_isset(zero_electric_angle)) {
    //     // align the electrical phases of the motor and sensor
    //     // set angle -90(270 = 3PI/2) degrees
    //     setPhaseVoltage(voltage_align, 0, _3PI_2);
    //     _delay(700);
    //     // read the sensor
    //     sensor->update();
    //     // get the current zero electric angle
    //     zero_electric_angle = 0;
    //     zero_electric_angle = electricalAngle();
    //     // zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    //     _delay(20);
    //     if (monitor_port) {
    //         SIMPLEFOC_DEBUG("MOT: Zero elec. angle: ", zero_electric_angle);
    //     }
    //     // stop everything
    //     setPhaseVoltage(0, 0, 0);
    //     _delay(200);
    // }
    // else {
    //     SIMPLEFOC_DEBUG("MOT: Skip offset calib.");
    // }
    // return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int motor_absoluteZeroSearch(BLDCMotor_t *motor) {
    // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
    //                    of float is sufficient.
    // SIMPLEFOC_DEBUG("MOT: Index search...");
    // // search the absolute zero with small velocity
    // float limit_vel = velocity_limit;
    // float limit_volt = voltage_limit;
    // velocity_limit = velocity_index_search;
    // voltage_limit = voltage_sensor_align;
    // shaft_angle = 0;
    // while (sensor->needsSearch() && shaft_angle < _2PI) {
    //     angleOpenloop(1.5f * _2PI);
    //     // call important for some sensors not to loose count
    //     // not needed for the search
    //     sensor->update();
    // }
    // // disable motor
    // setPhaseVoltage(0, 0, 0);
    // // reinit the limits
    // velocity_limit = limit_vel;
    // voltage_limit = limit_volt;
    // // check if the zero found
    // if (monitor_port) {
    //     if (sensor->needsSearch()) {
    //         SIMPLEFOC_DEBUG("MOT: Error: Not found!");
    //     }
    //     else {
    //         SIMPLEFOC_DEBUG("MOT: Success!");
    //     }
    // }
    // return !sensor->needsSearch();
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void motor_loopFOC(BLDCMotor_t *motor) {
    // update sensor - do this even in open-loop mode, as user may be switching between modes and we could lose track
    //                 of full rotations otherwise.
    // if (sensor)
    //     sensor->update();

    // // if open-loop do nothing
    // if (controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop)
    //     return;

    // // if disabled do nothing
    // if (!enabled)
    //     return;

    // // Needs the update() to be called first
    // // This function will not have numerical issues because it uses Sensor::getMechanicalAngle()
    // // which is in range 0-2PI
    // electrical_angle = electricalAngle();
    // switch (torque_controller) {
    // case TorqueControlType::voltage:
    //     // no need to do anything really
    //     break;
    // case TorqueControlType::dc_current:
    //     if (!current_sense)
    //         return;
    //     // read overall current magnitude
    //     current.q = current_sense->getDCCurrent(electrical_angle);
    //     // filter the value values
    //     current.q = LPF_current_q(current.q);
    //     // calculate the phase voltage
    //     voltage.q = PID_current_q(current_sp - current.q);
    //     // d voltage  - lag compensation
    //     if (_isset(phase_inductance))
    //         voltage.d =
    //             _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit,
    //             voltage_limit);
    //     else
    //         voltage.d = 0;
    //     break;
    // case TorqueControlType::foc_current:
    //     if (!current_sense)
    //         return;
    //     // read dq currents
    //     current = current_sense->getFOCCurrents(electrical_angle);
    //     // filter values
    //     current.q = LPF_current_q(current.q);
    //     current.d = LPF_current_d(current.d);
    //     // calculate the phase voltages
    //     voltage.q = PID_current_q(current_sp - current.q);
    //     voltage.d = PID_current_d(-current.d);
    //     // d voltage - lag compensation - TODO verify
    //     // if(_isset(phase_inductance)) voltage.d = _constrain( voltage.d -
    //     // current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
    //     break;
    // default:
    //     // no torque control selected
    //     SIMPLEFOC_DEBUG("MOT: no torque control selected!");
    //     break;
    // }

    // // set the phase voltage - FOC heart function :)
    // setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void motor_move(BLDCMotor_t *motor, float new_target) {
    // set internal target variable
    // if (_isset(new_target))
    //     target = new_target;
    motor->target = new_target;

    // LOG_INFO("Got target of %f", motor->target);

    // downsampling (optional)
    // if (motion_cnt++ < motion_downsample)
    //     return;
    // motion_cnt = 0;

    // shaft angle/velocity need the update() to be called first
    // get shaft angle
    // TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a
    // float
    //                        For this reason it is NOT precise when the angles become large.
    //                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a
    //                        problem when switching to a 2-component representation.
    // if (controller != MotionControlType::angle_openloop && controller != MotionControlType::velocity_openloop)
    //     shaft_angle = shaftAngle();  // read value even if motor is disabled to keep the monitoring updated but not
    //     in
    // openloop mode
    // get angular velocity  TODO the velocity reading probably also shouldn't happen in open loop modes?
    // motor->shaft_velocity =
    //     motor_shaftVelocity();  // read value even if motor is disabled to keep the monitoring updated

    // if disabled do nothing
    if (!motor->enabled)
        return;

    // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
    // if (_isset(KV_rating))
    //     voltage_bemf = shaft_velocity / (KV_rating * _SQRT3) / _RPM_TO_RADS;
    // // estimate the motor current if phase reistance available and current_sense not available
    // if (!current_sense && _isset(phase_resistance))
    //     current.q = (voltage.q - voltage_bemf) / phase_resistance;

    // upgrade the current based voltage limit
    // switch (controller) {
    // case MotionControlType::torque:
    //     if (torque_controller == TorqueControlType::voltage) {  // if voltage torque control
    //         if (!_isset(phase_resistance))
    //             voltage.q = target;
    //         else
    //             voltage.q = target * phase_resistance + voltage_bemf;
    //         voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
    //         // set d-component (lag compensation if known inductance)
    //         if (!_isset(phase_inductance))
    //             voltage.d = 0;
    //         else
    //             voltage.d =
    //                 _constrain(-target * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit,
    //                 voltage_limit);
    //     }
    //     else {
    //         current_sp = target;  // if current/foc_current torque control
    //     }
    //     break;
    // case MotionControlType::angle:
    //     // TODO sensor precision: this calculation is not numerically precise. The target value cannot express
    //     precise
    //     // positions when
    //     //                        the angles are large. This results in not being able to command small changes at
    //     high
    //     //                        position values. to solve this, the delta-angle has to be calculated in a
    //     numerically
    //     //                        precise way.
    //     // angle set point
    //     shaft_angle_sp = target;
    //     // calculate velocity set point
    //     shaft_velocity_sp = feed_forward_velocity + P_angle(shaft_angle_sp - shaft_angle);
    //     shaft_velocity_sp = _constrain(shaft_velocity_sp, -velocity_limit, velocity_limit);
    //     // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from
    //     previous
    //     // calculation
    //     current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);  // if voltage torque control
    //     // if torque controlled through voltage
    //     if (torque_controller == TorqueControlType::voltage) {
    //         // use voltage if phase-resistance not provided
    //         if (!_isset(phase_resistance))
    //             voltage.q = current_sp;
    //         else
    //             voltage.q = _constrain(current_sp * phase_resistance + voltage_bemf, -voltage_limit, voltage_limit);
    //         // set d-component (lag compensation if known inductance)
    //         if (!_isset(phase_inductance))
    //             voltage.d = 0;
    //         else
    //             voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit,
    //                                    voltage_limit);
    //     }
    //     break;
    // case MotionControlType::velocity:
    //     // velocity set point - sensor precision: this calculation is numerically precise.
    //     shaft_velocity_sp = target;
    //     // calculate the torque command
    //     current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);  // if current/foc_current torque control
    //     // if torque controlled through voltage control
    //     if (torque_controller == TorqueControlType::voltage) {
    //         // use voltage if phase-resistance not provided
    //         if (!_isset(phase_resistance))
    //             voltage.q = current_sp;
    //         else
    //             voltage.q = _constrain(current_sp * phase_resistance + voltage_bemf, -voltage_limit, voltage_limit);
    //         // set d-component (lag compensation if known inductance)
    //         if (!_isset(phase_inductance))
    //             voltage.d = 0;
    //         else
    //             voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit,
    //                                    voltage_limit);
    //     }
    //     break;
    // case MotionControlType::velocity_openloop:
    // velocity control in open loop - sensor precision: this calculation is numerically precise.
    motor->shaft_velocity_sp = motor->target;
    motor->voltage.q =
        motor_velocityOpenloop(motor, motor->shaft_velocity_sp);  // returns the voltage that is set to the motor
    motor->voltage.d = 0;
    // break;
    // case MotionControlType::angle_openloop:
    //     // angle control in open loop -
    //     // TODO sensor precision: this calculation NOT numerically precise, and subject
    //     //                        to the same problems in small set-point changes at high angles
    //     //                        as the closed loop version.
    //     shaft_angle_sp = target;
    //     voltage.q = angleOpenloop(shaft_angle_sp);  // returns the voltage that is set to the motor
    //     voltage.d = 0;
    //     break;
    // }
}

float min(float val1, float val2) {
    return val1 < val2 ? val1 : val2;
}

void _sincos(float a, float *s, float *c) {
    *s = sin(a);
    *c = cos(a);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void motor_setPhaseVoltage(BLDCMotor_t *motor, float Uq, float Ud, float angle_el) {
    float center;
    int sector;
    float _ca, _sa;

    // switch (foc_modulation) {
    // case FOCModulationType::Trapezoid_120:
    //     // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
    //     // determine the sector
    //     sector = 6 * (_normalizeAngle(angle_el + _PI_6) / _2PI);  // adding PI/6 to align with other modes
    //     // centering the voltages around either
    //     // modulation_centered == true > driver.voltage_limit/2
    //     // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
    //     center = modulation_centered ? (driver->voltage_limit) / 2 : Uq;

    //     if (trap_120_map[sector][0] == _HIGH_IMPEDANCE) {
    //         Ua = center;
    //         Ub = trap_120_map[sector][1] * Uq + center;
    //         Uc = trap_120_map[sector][2] * Uq + center;
    //         driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON,
    //                               PhaseState::PHASE_ON);  // disable phase if possible
    //     }
    //     else if (trap_120_map[sector][1] == _HIGH_IMPEDANCE) {
    //         Ua = trap_120_map[sector][0] * Uq + center;
    //         Ub = center;
    //         Uc = trap_120_map[sector][2] * Uq + center;
    //         driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF,
    //                               PhaseState::PHASE_ON);  // disable phase if possible
    //     }
    //     else {
    //         Ua = trap_120_map[sector][0] * Uq + center;
    //         Ub = trap_120_map[sector][1] * Uq + center;
    //         Uc = center;
    //         driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON,
    //                               PhaseState::PHASE_OFF);  // disable phase if possible
    //     }

    //     break;

    // case FOCModulationType::Trapezoid_150:
    //     // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
    //     // determine the sector
    //     sector = 12 * (_normalizeAngle(angle_el + _PI_6) / _2PI);  // adding PI/6 to align with other modes
    //     // centering the voltages around either
    //     // modulation_centered == true > driver.voltage_limit/2
    //     // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
    //     center = modulation_centered ? (driver->voltage_limit) / 2 : Uq;

    //     if (trap_150_map[sector][0] == _HIGH_IMPEDANCE) {
    //         Ua = center;
    //         Ub = trap_150_map[sector][1] * Uq + center;
    //         Uc = trap_150_map[sector][2] * Uq + center;
    //         driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON,
    //                               PhaseState::PHASE_ON);  // disable phase if possible
    //     }
    //     else if (trap_150_map[sector][1] == _HIGH_IMPEDANCE) {
    //         Ua = trap_150_map[sector][0] * Uq + center;
    //         Ub = center;
    //         Uc = trap_150_map[sector][2] * Uq + center;
    //         driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF,
    //                               PhaseState::PHASE_ON);  // disable phase if possible
    //     }
    //     else if (trap_150_map[sector][2] == _HIGH_IMPEDANCE) {
    //         Ua = trap_150_map[sector][0] * Uq + center;
    //         Ub = trap_150_map[sector][1] * Uq + center;
    //         Uc = center;
    //         driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON,
    //                               PhaseState::PHASE_OFF);  // disable phase if possible
    //     }
    //     else {
    //         Ua = trap_150_map[sector][0] * Uq + center;
    //         Ub = trap_150_map[sector][1] * Uq + center;
    //         Uc = trap_150_map[sector][2] * Uq + center;
    //         driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON,
    //                               PhaseState::PHASE_ON);  // enable all phases
    //     }

    //     break;

    // case FOCModulationType::SinePWM:
    // case FOCModulationType::SpaceVectorPWM:
    // Sinusoidal PWM modulation
    // Inverse Park + Clarke transformation
    _sincos(angle_el, &_sa, &_ca);

    // LOG_INFO("Angle: %f, Cos: %f, Sin: %f", angle_el, _ca, _sa);

    // Inverse park transform
    motor->Ualpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
    motor->Ubeta = _sa * Ud + _ca * Uq;   //  cos(angle) * Uq;

    // Clarke transform
    motor->Ua = motor->Ualpha;
    motor->Ub = -0.5f * motor->Ualpha + _SQRT3_2 * motor->Ubeta;
    motor->Uc = -0.5f * motor->Ualpha - _SQRT3_2 * motor->Ubeta;

    // LOG_INFO("Before center: %f, %f, %f", motor->Ua, motor->Ub, motor->Uc);

    center = motor->driver->voltage_limit / 2;
    // if (foc_modulation == FOCModulationType::SpaceVectorPWM) {
    //     // discussed here:
    //     // https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1 a bit
    //     // more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best Midpoint Clamp
    //     float Umin = min(Ua, min(Ub, Uc));
    //     float Umax = max(Ua, max(Ub, Uc));
    //     center -= (Umax + Umin) / 2;
    // }

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

    // LOG_INFO("Got motor phase voltages %f, %f, %f", motor->Ua, motor->Ub, motor->Uc);

    //     break;
    // }

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
    // if (_isset(phase_resistance)) {
    //     Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    //     // recalculate the current
    //     current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
    // }
    // set the maximal allowed voltage (voltage_limit) with the necessary angle
    motor_setPhaseVoltage(motor, Uq, 0, _electricalAngle(motor->shaft_angle, motor->pole_pairs));

    // save timestamp for next call
    motor->open_loop_timestamp = now_us;

    return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float motor_angleOpenloop(BLDCMotor_t *motor, float target_angle) {
    // // get current timestamp
    // unsigned long now_us = _micros();
    // // calculate the sample time from last call
    // float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    // // quick fix for strange cases (micros overflow + timestamp not defined)
    // if (Ts <= 0 || Ts > 0.5f)
    //     Ts = 1e-3f;

    // // calculate the necessary angle to move from current position towards target angle
    // // with maximal velocity (velocity_limit)
    // // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
    // //                        where small position changes are no longer captured by the precision of floats
    // //                        when the total position is large.
    // if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts)) {
    //     shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit) * Ts;
    //     shaft_velocity = velocity_limit;
    // }
    // else {
    //     shaft_angle = target_angle;
    //     shaft_velocity = 0;
    // }

    // // use voltage limit or current limit
    // float Uq = voltage_limit;
    // if (_isset(phase_resistance)) {
    //     Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    //     // recalculate the current
    //     current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
    // }
    // // set the maximal allowed voltage (voltage_limit) with the necessary angle
    // // sensor precision: this calculation is OK due to the normalisation
    // setPhaseVoltage(Uq, 0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

    // // save timestamp for next call
    // open_loop_timestamp = now_us;

    // return Uq;
}
