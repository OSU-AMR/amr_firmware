#include "controller.h"

#include "core1.h"

#include "driver/quad_encoder.h"

#include <math.h>
#include <memory.h>

// Hardware parameters
#define ENCODER_TPR 2048

// Controller parameters
#define ENCODER_FILTER_GAIN .01
#define MAX_SPEED 130  // rad / sec
#define STARTUP_VOLTAGE_BOOST 5

// Hardware
encoder encoders[NUM_MOTORS];

// Internal state
float vel_avg[NUM_MOTORS];

uint32_t previous_loop_time;

float target_velocity[NUM_MOTORS];
float voltage_limit[NUM_MOTORS];

void controller_init() {
    core1_init();

    encoder_init(&encoders[0], ENC0_A_PIN, ENC0_B_PIN, ENCODER_TPR);
    encoder_init(&encoders[1], ENC1_A_PIN, ENC1_B_PIN, ENCODER_TPR);

    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
}

bool val = false;

void controller_tick() {
    // Do this math as a separate loop since the timing here is somewhat important
    float time = to_us_since_boot(get_absolute_time()) / 1000000.f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        vel_avg[i] =
            (vel_avg[i] + encoder_get_velocity(&encoders[i]) * ENCODER_FILTER_GAIN) / (1 + ENCODER_FILTER_GAIN);
    }
    previous_loop_time = time;

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Clamp max velocity
        target_velocity[i] = MAX(MIN(MAX_SPEED, target_velocity[i]), -MAX_SPEED);

        // Handle voltage regulation
        voltage_limit[i] = 0.0f;
        if (target_velocity[i] > 0.1f)
            voltage_limit[i] =
                fabs(target_velocity[i] - vel_avg[i]) * 0.5f + 1.0f * fabs(target_velocity[i] - 10.0f) / 20.0f + 3.0f;
        else if (target_velocity[i] < -0.1f) {
            voltage_limit[i] =
                fabs(vel_avg[i] - target_velocity[i]) * 0.5f + 1.0f * fabs(target_velocity[i] - 10.0f) / 20.0f + 3.0f;
        }

        // Startup voltage boost
        if (vel_avg[i] / target_velocity[i] < 0.5f && fabs(target_velocity[i]) > 0.1f) {
            voltage_limit[i] = voltage_limit[i] + STARTUP_VOLTAGE_BOOST;
        }

        // Enforce max voltage limit
        voltage_limit[i] = MIN(voltage_limit[i], fixedpt_tofloat(DRIVER_VOLTAGE_SUPPLY_FIXEDPT));

        // Send motor commands (under spin lock)
        fixedpt target_velocity_fixed = fixedpt_rconst(*target_velocity);
        fixedpt voltage_limit_fixed = fixedpt_rconst(*voltage_limit);

        core1_update_targets(&target_velocity_fixed, &voltage_limit_fixed);
    }

    gpio_put(2, val);
    val = !val;
}

void controller_set_target(const float *rps) {
    memcpy(target_velocity, rps, NUM_MOTORS);
    // fixedpt vlim = 0;
    // fixedpt rps_fixed = fixedpt_rconst(*rps);

    // core1_update_targets(&rps_fixed, &vlim);
}
