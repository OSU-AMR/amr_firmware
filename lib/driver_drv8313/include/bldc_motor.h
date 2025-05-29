#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "bldc_driver_3pwm.h"
#include "fixedptc.h"

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
    fixedpt d;
    fixedpt q;
} DQCurrent_s;
// phase current structure
typedef struct PhaseCurrent {
    fixedpt a;
    fixedpt b;
    fixedpt c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct DQVoltage {
    fixedpt d;
    fixedpt q;
} DQVoltage_s;
// alpha beta current structure
typedef struct ABCurrent {
    fixedpt alpha;
    fixedpt beta;
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
    fixedpt Ua, Ub, Uc;  //!< Current phase voltages Ua,Ub and Uc set to motor

    // state variables
    fixedpt target;                 //!< current target value - depends of the controller
    fixedpt feed_forward_velocity;  //!< current feed forward velocity
    fixedpt shaft_angle;            //!< current motor angle
    fixedpt electrical_angle;       //!< current electrical angle
    fixedpt shaft_velocity;         //!< current motor velocity
    fixedpt current_sp;             //!< target current ( q current )
    fixedpt shaft_velocity_sp;      //!< current target velocity
    fixedpt shaft_angle_sp;         //!< current target angle
    DQVoltage_s voltage;            //!< current d and q voltage set to the motor
    DQCurrent_s current;            //!< current d and q current measured
    fixedpt voltage_bemf;           //!< estimated backemf voltage (if provided KV constant)
    fixedpt Ualpha, Ubeta;          //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

    // motor configuration parameters
    fixedpt voltage_sensor_align;   //!< sensor and motor align voltage parameter
    fixedpt velocity_index_search;  //!< target velocity for index search

    // motor physical parameters
    fixedpt phase_resistance;  //!< motor phase resistance
    int pole_pairs;            //!< motor pole pairs number
    fixedpt KV_rating;         //!< motor KV rating
    fixedpt phase_inductance;  //!< motor phase inductance

    // limiting variables
    fixedpt voltage_limit;   //!< Voltage limiting variable - global limit
    fixedpt current_limit;   //!< Current limiting variable - global limit
    fixedpt velocity_limit;  //!< Velocity limiting variable - global limit

    // motor status vairables
    int8_t enabled;                 //!< enabled or disabled motor flag
    FOCMotorStatus_t motor_status;  //!< motor status

    // pwm modulation related variables
    FOCModulationType_t foc_modulation;  //!<  parameter determining modulation algorithm
    int8_t modulation_centered;          //!< flag (1) centered modulation around driver limit /2  or  (0) pulled to 0

    // configuration structures
    TorqueControlType_t torque_controller;  //!< parameter determining the torque control type
    MotionControlType_t controller;         //!< parameter determining the control loop to be used
    Direction_t sensor_direction;  //!< default is CW. if sensor_direction == Direction::CCW then direction will be
                                   //!< flipped compared to CW. Set to UNKNOWN to set by calibration
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
 * Function executing the control loops set by the controller parameter of the BLDCMotor.
 *
 * @param target  Either voltage, angle or velocity based on the motor.controller
 *                If it is not set the motor will use the target set in its variable motor.target
 *
 * This function doesn't need to be run upon each loop execution - depends of the use case
 */
void motor_move(BLDCMotor_t *motor, fixedpt target);

/**
 * Method using FOC to set Uq to the motor at the optimal angle
 * Heart of the FOC algorithm
 *
 * @param Uq Current voltage in q axis to set to the motor
 * @param Ud Current voltage in d axis to set to the motor
 * @param angle_el current electrical angle of the motor
 */
void motor_setPhaseVoltage(BLDCMotor_t *motor, fixedpt Uq, fixedpt Ud, fixedpt angle_el);

// Open loop motion control
/**
 * Function (iterative) generating open loop movement for target velocity
 * it uses voltage_limit variable
 *
 * @param target_velocity - rad/s
 */
fixedpt motor_velocityOpenloop(BLDCMotor_t *motor, fixedpt target_velocity);

#endif
