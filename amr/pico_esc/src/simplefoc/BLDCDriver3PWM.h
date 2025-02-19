#ifndef BLDCDriver3PWM_h
#define BLDCDriver3PWM_h

// #include "../common/base_classes/BLDCDriver.h"
// #include "../common/defaults.h"
// #include "../common/foc_utils.h"
// #include "../common/time_utils.h"
#include "hardware_api.h"

#include "hardware/gpio.h"

typedef struct BLDCDriver3PWM {
    // hardware variables
    int pwmA;         //!< phase A pwm pin number
    int pwmB;         //!< phase B pwm pin number
    int pwmC;         //!< phase C pwm pin number
    int enableA_pin;  //!< enable pin number
    int enableB_pin;  //!< enable pin number
    int enableC_pin;  //!< enable pin number

    long pwm_frequency;          //!< pwm frequency value in hertz
    float voltage_power_supply;  //!< power supply voltage
    float voltage_limit;         //!< limiting voltage set to the motor

    bool initialized;  //!< true if driver was successfully initialized
    void *params;      //!< pointer to hardware specific parameters of driver

    bool enable_active_high;  //!< enable pin should be set to high to enable the driver (default is HIGH)

    float dc_a;  //!< currently set duty cycle on phaseA
    float dc_b;  //!< currently set duty cycle on phaseB
    float dc_c;  //!< currently set duty cycle on phaseC

} BLDCDRIVER3PWM_t;

typedef enum PhaseState {
    PHASE_OFF = 0,  // both sides of the phase are off
    PHASE_ON = 1,   // both sides of the phase are driven with PWM, dead time is applied in 6-PWM mode
    PHASE_HI = 2,   // only the high side of the phase is driven with PWM (6-PWM mode only)
    PHASE_LO = 3,   // only the low side of the phase is driven with PWM (6-PWM mode only)
} PhaseState_t;

enum DriverType { Unknown = 0, BLDC = 1, Stepper = 2 };

/**
  BLDCDriver class constructor
  @param phA A phase pwm pin
  @param phB B phase pwm pin
  @param phC C phase pwm pin
  @param en1 enable pin (optional input)
  @param en2 enable pin (optional input)
  @param en3 enable pin (optional input)
*/
void make_BLDCDriver3PWM(BLDCDRIVER3PWM_t *driver, int phA, int phB, int phC, int en1);

/**  Motor hardware init function */
int driver_init(BLDCDRIVER3PWM_t *driver);
/** Motor disable function */
void driver_disable(BLDCDRIVER3PWM_t *driver);
/** Motor enable function */
void driver_enable(BLDCDRIVER3PWM_t *driver);

/**
 * Set phase voltages to the hardware
 *
 * @param Ua - phase A voltage
 * @param Ub - phase B voltage
 * @param Uc - phase C voltage
 */
void driver_setPwm(BLDCDRIVER3PWM_t *driver, float Ua, float Ub, float Uc);

/**
 * Set phase voltages to the hardware
 * > Only possible is the driver has separate enable pins for all phases!
 *
 * @param sc - phase A state : active / disabled ( high impedance )
 * @param sb - phase B state : active / disabled ( high impedance )
 * @param sa - phase C state : active / disabled ( high impedance )
 */
void driver_setPhaseState(BLDCDRIVER3PWM_t *driver, PhaseState_t sa, PhaseState_t sb, PhaseState_t sc);

#endif
