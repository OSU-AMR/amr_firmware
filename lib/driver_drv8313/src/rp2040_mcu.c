/**
 * Support for the RP2040 MCU, as found on the Raspberry Pi Pico.
 */

#include "./rp2040_mcu.h"

// #pragma message("")
// #pragma message("SimpleFOC: compiling for RP2040")
// #pragma message("")

// #if !defined(SIMPLEFOC_DEBUG_RP2040)
// #define SIMPLEFOC_DEBUG_RP2040
// #endif

#include "./hardware_api.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <stdlib.h>
// #if defined(USE_ARDUINO_PINOUT)
// #include <pinDefinitions.h>
// #endif

#define _PWM_FREQUENCY 24000
#define _PWM_FREQUENCY_MAX 66000
#define _PWM_FREQUENCY_MIN 1

// until I can figure out if this can be quickly read from some register, keep it here.
// it also serves as a marker for what slices are already used.
uint16_t wrapvalues[NUM_PWM_SLICES];

// TODO add checks which channels are already used...

void setupPWM(int pin_nr, long pwm_frequency, bool invert, RP2040DriverParams *params, uint8_t index) {
#if defined(USE_ARDUINO_PINOUT)
    uint pin = (uint) digitalPinToPinName(pin_nr);  // we could check for -DBOARD_HAS_PIN_REMAP ?
#else
    uint pin = (uint) pin_nr;
#endif
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan = pwm_gpio_to_channel(pin);
    params->pins[index] = pin;
    params->slice[index] = slice;
    params->chan[index] = chan;
    uint32_t sysclock_hz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000;
    uint32_t factor = 4096 * 2 * pwm_frequency;
    uint32_t div = sysclock_hz / factor;
    if (sysclock_hz % factor != 0)
        div += 1;
    if (div < 16)
        div = 16;
    uint32_t wrapvalue = (sysclock_hz * 8) / div / pwm_frequency - 1;

    pwm_set_clkdiv_int_frac(slice, div >> 4, div & 0xF);
    pwm_set_phase_correct(slice, true);
    pwm_set_wrap(slice, wrapvalue);
    wrapvalues[slice] = wrapvalue;
    if (invert) {
        if (chan == 0)
            hw_write_masked(&pwm_hw->slice[slice].csr, 0x1 << PWM_CH0_CSR_A_INV_LSB, PWM_CH0_CSR_A_INV_BITS);
        else
            hw_write_masked(&pwm_hw->slice[slice].csr, 0x1 << PWM_CH0_CSR_B_INV_LSB, PWM_CH0_CSR_B_INV_BITS);
    }
    pwm_set_chan_level(slice, chan, 0);  // switch off initially
}

void syncSlices() {
    for (uint i = 0; i < NUM_PWM_SLICES; i++) {
        pwm_set_enabled(i, false);
        pwm_set_counter(i, 0);
    }

    // enable all slices
    pwm_set_mask_enabled(0xFF);
}

void *_configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
    RP2040DriverParams *params = malloc(sizeof(*params));

    params->pwm_frequency = _PWM_FREQUENCY;
    pwm_frequency = _PWM_FREQUENCY;
    setupPWM(pinA, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 0);
    setupPWM(pinB, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 1);
    setupPWM(pinC, pwm_frequency, !SIMPLEFOC_PWM_ACTIVE_HIGH, params, 2);
    syncSlices();
    return params;
}

void writeDutyCycle(float val, uint slice, uint chan) {
    pwm_set_chan_level(slice, chan, (wrapvalues[slice] + 1) * val);
}

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void *params) {
    writeDutyCycle(dc_a, ((RP2040DriverParams *) params)->slice[0], ((RP2040DriverParams *) params)->chan[0]);
    writeDutyCycle(dc_b, ((RP2040DriverParams *) params)->slice[1], ((RP2040DriverParams *) params)->chan[1]);
    writeDutyCycle(dc_c, ((RP2040DriverParams *) params)->slice[2], ((RP2040DriverParams *) params)->chan[2]);
}
