#include "ir.h"

#include "hardware/adc.h"
#include "titan/logger.h"

#define ADC_MIN_PIN 26

static bool first_init = true;

void ir_init(uint pin) {
    if (pin < ADC_MIN_PIN) {
        LOG_ERROR("Attempted to init an IR sensor on a non-ADC pin: %u", pin);
        return;
    }

    if (first_init) {
        adc_init();
        first_init = false;
    }

    adc_gpio_init(pin);
}

uint16_t ir_read(uint pin) {
    if (pin < ADC_MIN_PIN) {
        LOG_ERROR("Attempted to read an IR sensor on a non-ADC pin: %u", pin);
        return 0;
    }

    adc_select_input(pin - ADC_MIN_PIN);  // ADC_MIN_PIN maps to adc 0
    return adc_read();
}
