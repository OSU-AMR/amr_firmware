#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H

// #include "../common/base_classes/BLDCDriver.h"
// #include "../common/foc_utils.h"
// #include "../common/time_utils.h"
// #include "../communication/SimpleFOCDebug.h"

// these defines determine the polarity of the PWM output. Normally, the polarity is active-high,
// i.e. a high-level PWM output is expected to switch on the MOSFET. But should your driver design
// require inverted polarity, you can change the defines below, or set them via your build environment
// or board definition files.

// used for 1-PWM, 2-PWM, 3-PWM, and 4-PWM modes
#ifndef SIMPLEFOC_PWM_ACTIVE_HIGH
#define SIMPLEFOC_PWM_ACTIVE_HIGH true
#endif

// flag returned if driver init fails
#define SIMPLEFOC_DRIVER_INIT_FAILED ((void *) -1)

// generic implementation of the hardware specific structure
// containing all the necessary driver parameters
// will be returned as a void pointer from the _configurexPWM functions
// will be provided to the _writeDutyCyclexPWM() as a void pointer
typedef struct GenericDriverParams {
    int pins[6];
    long pwm_frequency;
    fixedpt dead_zone;
} GenericDriverParams;

/**
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 3PWM setting
 * - hardware specific
 *
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pinA pinA bldc driver
 * @param pinB pinB bldc driver
 * @param pinC pinC bldc driver
 *
 * @return -1 if failed, or pointer to internal driver parameters struct if successful
 */
void *_configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC);

/**
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 3PWM setting
 * - hardware specific
 *
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param params  the driver parameters
 */
void _writeDutyCycle3PWM(fixedpt dc_a, fixedpt dc_b, fixedpt dc_c, void *params);

#endif
