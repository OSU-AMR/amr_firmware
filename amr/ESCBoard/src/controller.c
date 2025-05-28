#include "controller.h"

#include "core1.h"

#include "driver/quad_encoder.h"

#include <math.h>

// Hardware parameters
#define ENCODER_TPR 2048

// Controller parameters
#define ENCODER_FILTER_GAIN .01
#define MAX_SPEED 130  // rad / sec
#define STARTUP_VOLTAGE_BOOST 5

// Hardware
encoder encoders[NUM_MOTORS];

// Internal state
float vel_avg[2];

uint32_t previous_loop_time;

float target_velocity[2];
float voltage_limit[2];

void controller_init() {
    core1_init();

    encoder_init(&encoders[0], ENC0_A_PIN, ENC0_B_PIN, ENCODER_TPR);
    encoder_init(&encoders[1], ENC1_A_PIN, ENC1_B_PIN, ENCODER_TPR);
}

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
        voltage_limit[i] = MIN(voltage_limit[i], DRIVER_VOLTAGE_SUPPLY);

        // Send motor commands (under spin lock)
        core1_update_targets(target_velocity, voltage_limit);
    }
}

void controller_set_target(float left_rps, float right_rps) {
    target_velocity[0] = left_rps;
    target_velocity[1] = right_rps;
}
