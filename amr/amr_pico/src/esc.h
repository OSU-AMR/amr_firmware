#ifndef ESC_H_
#define ESC_H_

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "titan/logger.h"

#define PWM_FREQ_HZ 200
#define PWM_PERIOD_US 5000

#define ESC_INIT_US 1590
#define ESC_STARTUP_DELAY_MS 7000

static uint left_slice;
static uint left_channel;
static uint right_slice;
static uint right_channel;

static bool escs_active = false;

static uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d) {
    // uint32_t clock = clock_get_hz(clk_sys)
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);

    if (divider16 / 16 == 0)
        divider16 = 16;

    uint32_t wrap = clock * 16 / divider16 / f - 1;

    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);

    return wrap;
}

static void esc_set_us(uint slice, uint channel, uint us) {
    pwm_set_freq_duty(slice, channel, PWM_FREQ_HZ, (float) us / PWM_PERIOD_US * 100);
}

static int64_t mark_escs_active() {
    escs_active = true;
    LOG_INFO("Marked ESCs as ready");
    return 0;
}

void enable_escs() {
    escs_active = false;

    gpio_set_function(LEFT_ESC_PIN, GPIO_FUNC_PWM);
    left_slice = pwm_gpio_to_slice_num(LEFT_ESC_PIN);
    left_channel = pwm_gpio_to_channel(LEFT_ESC_PIN);
    // esc_set_us(left_slice, left_channel, ESC_INIT_US);
    pwm_set_freq_duty(left_slice, left_channel, PWM_FREQ_HZ, 1500.0f / 20000.0f * 100.0f);
    pwm_set_enabled(left_slice, true);

    gpio_set_function(RIGHT_ESC_PIN, GPIO_FUNC_PWM);
    right_slice = pwm_gpio_to_slice_num(RIGHT_ESC_PIN);
    right_channel = pwm_gpio_to_channel(RIGHT_ESC_PIN);
    // esc_set_us(right_slice, right_channel, ESC_INIT_US);
    pwm_set_freq_duty(right_slice, right_channel, PWM_FREQ_HZ, 1500.0f / 20000.0f * 100.0f);
    pwm_set_enabled(right_slice, true);

    LOG_INFO("Issued startup PWM");

    add_alarm_in_ms(ESC_STARTUP_DELAY_MS, mark_escs_active, NULL, false);
}

void disable_escs() {
    pwm_set_enabled(left_slice, false);
    pwm_set_enabled(right_slice, false);

    LOG_INFO("Disabled ESC PWM on ROS disconnect");
}

void set_left_esc_us(uint us) {
    if (!escs_active)
        return;

    esc_set_us(left_slice, left_channel, us);
}

void set_right_esc_us(uint us) {
    if (!escs_active)
        return;

    esc_set_us(right_slice, right_channel, us);
}

#endif  // ESC_H_
