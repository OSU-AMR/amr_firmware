#ifndef BLDCMotor_h
#define BLDCMotor_h

// #include "Arduino.h"
// #include "common/base_classes/BLDCDriver.h"
// #include "common/base_classes/FOCDriver.h"
// #include "common/base_classes/FOCMotor.h"
// #include "common/base_classes/Sensor.h"
// #include "common/defaults.h"
// #include "common/foc_utils.h"
// #include "common/time_utils.h"
#include "bldc_driver_3pwm.h"

// monitoring bitmap
#define _MON_TARGET 0b1000000  // monitor target value
#define _MON_VOLT_Q 0b0100000  // monitor voltage q value
#define _MON_VOLT_D 0b0010000  // monitor voltage d value
#define _MON_CURR_Q 0b0001000  // monitor current q value - if measured
#define _MON_CURR_D 0b0000100  // monitor current d value - if measured
#define _MON_VEL 0b0000010     // monitor velocity value
#define _MON_ANGLE 0b0000001   // monitor angle value

/**
 *  Motiron control type
 */
typedef enum MotionControlType {
    torque = 0x00,    //!< Torque control
    velocity = 0x01,  //!< Velocity motion control
    angle = 0x02,     //!< Position/angle motion control
    velocity_openloop = 0x03,
    angle_openloop = 0x04
} MotionControlType_t;

/**
 *  Motiron control type
 */
typedef enum TorqueControlType {
    voltage = 0x00,      //!< Torque control using voltage
    dc_current = 0x01,   //!< Torque control using DC current (one current magnitude)
    foc_current = 0x02,  //!< torque control using dq currents
} TorqueControlType_t;

/**
 *  FOC modulation type
 */
typedef enum FOCModulationType {
    SinePWM = 0x00,         //!< Sinusoidal PWM modulation
    SpaceVectorPWM = 0x01,  //!< Space vector modulation method
    Trapezoid_120 = 0x02,
    Trapezoid_150 = 0x03,
} FOCModulationType_t;

typedef enum FOCMotorStatus {
    motor_uninitialized = 0x00,  //!< Motor is not yet initialized
    motor_initializing = 0x01,   //!< Motor intiialization is in progress
    motor_uncalibrated = 0x02,   //!< Motor is initialized, but not calibrated (open loop possible)
    motor_calibrating = 0x03,    //!< Motor calibration in progress
    motor_ready = 0x04,          //!< Motor is initialized and calibrated (closed loop possible)
    motor_error = 0x08,          //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
    motor_calib_failed = 0x0E,   //!< Motor calibration failed (possibly recoverable)
    motor_init_failed = 0x0F,    //!< Motor initialization failed (not recoverable)
} FOCMotorStatus_t;

// dq current structure
typedef struct DQCurrent {
    float d;
    float q;
} DQCurrent_s;
// phase current structure
typedef struct PhaseCurrent {
    float a;
    float b;
    float c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct DQVoltage {
    float d;
    float q;
} DQVoltage_s;
// alpha beta current structure
typedef struct ABCurrent {
    float alpha;
    float beta;
} ABCurrent_s;

typedef enum Direction {
    CW = 1,      // clockwise
    CCW = -1,    // counter clockwise
    UNKNOWN = 0  // not yet known or invalid state
} Direction_t;

typedef struct BLDCMotor {
    /**
     * BLDCDriver link:
     * - 3PWM
     * - 6PWM
     */
    BLDCDRIVER3PWM_t *driver;
    // open loop variables
    long open_loop_timestamp;
    float Ua, Ub, Uc;  //!< Current phase voltages Ua,Ub and Uc set to motor

    // state variables
    float target;                 //!< current target value - depends of the controller
    float feed_forward_velocity;  //!< current feed forward velocity
    float shaft_angle;            //!< current motor angle
    float electrical_angle;       //!< current electrical angle
    float shaft_velocity;         //!< current motor velocity
    float current_sp;             //!< target current ( q current )
    float shaft_velocity_sp;      //!< current target velocity
    float shaft_angle_sp;         //!< current target angle
    DQVoltage_s voltage;          //!< current d and q voltage set to the motor
    DQCurrent_s current;          //!< current d and q current measured
    float voltage_bemf;           //!< estimated backemf voltage (if provided KV constant)
    float Ualpha, Ubeta;          //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

    // motor configuration parameters
    float voltage_sensor_align;   //!< sensor and motor align voltage parameter
    float velocity_index_search;  //!< target velocity for index search

    // motor physical parameters
    float phase_resistance;  //!< motor phase resistance
    int pole_pairs;          //!< motor pole pairs number
    float KV_rating;         //!< motor KV rating
    float phase_inductance;  //!< motor phase inductance

    // limiting variables
    float voltage_limit;   //!< Voltage limiting variable - global limit
    float current_limit;   //!< Current limiting variable - global limit
    float velocity_limit;  //!< Velocity limiting variable - global limit

    // motor status vairables
    int8_t enabled;                 //!< enabled or disabled motor flag
    FOCMotorStatus_t motor_status;  //!< motor status

    // pwm modulation related variables
    FOCModulationType_t foc_modulation;  //!<  parameter determining modulation algorithm
    int8_t modulation_centered;          //!< flag (1) centered modulation around driver limit /2  or  (0) pulled to 0

    // configuration structures
    TorqueControlType_t torque_controller;  //!< parameter determining the torque control type
    MotionControlType_t controller;         //!< parameter determining the control loop to be used

    // // controllers and low pass filters
    // PIDController PID_current_q { DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP,
    //                               DEF_POWER_SUPPLY };  //!< parameter determining the q current PID config
    // PIDController PID_current_d { DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP,
    //                               DEF_POWER_SUPPLY };  //!< parameter determining the d current PID config
    // LowPassFilter LPF_current_q {
    //     DEF_CURR_FILTER_Tf
    // };  //!<  parameter determining the current Low pass filter configuration
    // LowPassFilter LPF_current_d {
    //     DEF_CURR_FILTER_Tf
    // };  //!<  parameter determining the current Low pass filter configuration
    // PIDController PID_velocity { DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, DEF_PID_VEL_RAMP,
    //                              DEF_PID_VEL_LIMIT };  //!< parameter determining the velocity PID configuration
    // PIDController P_angle { DEF_P_ANGLE_P, 0, 0, 0,
    //                         DEF_VEL_LIM };  //!< parameter determining the position PID configuration
    // LowPassFilter LPF_velocity {
    //     DEF_VEL_FILTER_Tf
    // };                                //!<  parameter determining the velocity Low pass filter configuration
    // LowPassFilter LPF_angle { 0.0 };  //!<  parameter determining the angle low pass filter configuration
    // unsigned int motion_downsample =
    //     DEF_MOTION_DOWNSMAPLE;    //!< parameter defining the ratio of downsampling for move commad
    // unsigned int motion_cnt = 0;  //!< counting variable for downsampling for move commad

    // sensor related variabels
    // float sensor_offset;                  //!< user defined sensor zero offset
    // float zero_electric_angle = NOT_SET;  //!< absolute zero electric angle - if available
    Direction_t
        sensor_direction;  //!< default is CW. if sensor_direction == Direction::CCW then direction will be flipped
    //                          //!< compared to CW. Set to UNKNOWN to set by calibration
    // bool pp_check_result = false;  //!< the result of the PP check, if run during loopFOC
} BLDCMotor_t;

/**
 BLDCMotor class constructor
 @param pp pole pairs number
 @param R  motor phase resistance - [Ohm]
 @param KV  motor KV rating (1/K_bemf) - rpm/V
 @param L  motor phase inductance - [H]
 */
void make_BLDCMotor(BLDCMotor_t *motor, int pp);

/**  Motor hardware init function */
int motor_init(BLDCMotor_t *motor);
/** Motor disable function */
void motor_disable(BLDCMotor_t *motor);
/** Motor enable function */
void motor_enable(BLDCMotor_t *motor);

/**
 * Function initializing FOC algorithm
 * and aligning sensor's and motors' zero position
 */
int motor_initFOC(BLDCMotor_t *motor);
/**
 * Function running FOC algorithm in real-time
 * it calculates the gets motor angle and sets the appropriate voltages
 * to the phase pwm signals
 * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
 */
void motor_loopFOC(BLDCMotor_t *motor);

/**
 * Function executing the control loops set by the controller parameter of the BLDCMotor.
 *
 * @param target  Either voltage, angle or velocity based on the motor.controller
 *                If it is not set the motor will use the target set in its variable motor.target
 *
 * This function doesn't need to be run upon each loop execution - depends of the use case
 */
void motor_move(BLDCMotor_t *motor, float target);

/**
 * Method using FOC to set Uq to the motor at the optimal angle
 * Heart of the FOC algorithm
 *
 * @param Uq Current voltage in q axis to set to the motor
 * @param Ud Current voltage in d axis to set to the motor
 * @param angle_el current electrical angle of the motor
 */
void motor_setPhaseVoltage(BLDCMotor_t *motor, float Uq, float Ud, float angle_el);

/** Sensor alignment to electrical 0 angle of the motor */
int motor_alignSensor(BLDCMotor_t *motor);
/** Current sense and motor phase alignment */
int motor_alignCurrentSense(BLDCMotor_t *motor);
/** Motor and sensor alignment to the sensors absolute 0 angle  */
int motor_absoluteZeroSearch(BLDCMotor_t *motor);

// Open loop motion control
/**
 * Function (iterative) generating open loop movement for target velocity
 * it uses voltage_limit variable
 *
 * @param target_velocity - rad/s
 */
float motor_velocityOpenloop(BLDCMotor_t *motor, float target_velocity);
/**
 * Function (iterative) generating open loop movement towards the target angle
 * it uses voltage_limit and velocity_limit variables
 *
 * @param target_angle - rad
 */
float motor_angleOpenloop(BLDCMotor_t *motor, float target_angle);

#endif
