#include "controller.h"

#include "core1.h"

#include "driver/quad_encoder.h"
#include "titan/logger.h"

#include <math.h>
#include <memory.h>

// Hardware parameters
#define ENCODER_TPR 2048

// Voltage control parameters (new)
#define KP_FIXEDPT fixedpt_rconst(0.5f)
#define MINIMUM_DESIGN_VEL_FIXEDPT fixedpt_rconst(10.0f)
#define FF_LINEAR_FIXEDPT fixedpt_rconst(1.0f / 20.0f)
#define VOLTAGE_BASELINE_FIXEDPT fixedpt_rconst(1.5f)
#define MAX_SLEW_RATE_FIXEDPT fixedpt_rconst(100.0f)

// Controller parameters
#define ENCODER_FILTER_GAIN_FIXEDPT fixedpt_rconst(.01f)
#define MAX_SPEED_FIXEDPT fixedpt_rconst(130.0f)  // rad / sec
#define STARTUP_VOLTAGE_BOOST_FIXEDPT fixedpt_rconst(5.0f)

// Interface parameters
#define RPS_MIN_UPDATE_RATE_MS 500

// Hardware
encoder encoders[NUM_MOTORS];

// Internal state
fixedpt vel_avg[NUM_MOTORS];

fixedpt target_velocity[NUM_MOTORS];
fixedpt voltage_limit[NUM_MOTORS];
fixedpt slewed_velocity[NUM_MOTORS];

absolute_time_t rps_expiration = { 0 };

// Telemetry
float encoder_vel[NUM_MOTORS];
float encoder_angle[NUM_MOTORS];

void controller_init() {
    core1_init();

    encoder_init(&encoders[0], ENC0_A_PIN, ENC0_B_PIN, ENCODER_TPR, false);
    encoder_init(&encoders[1], ENC1_A_PIN, ENC1_B_PIN, ENCODER_TPR, true);
}

void controller_tick() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Get encoder data to store in telemetry
        encoder_vel[i] = encoder_get_velocity(&encoders[i]);
        encoder_angle[i] = encoder_get_angle(&encoders[i]);

        vel_avg[i] = fixedpt_div(vel_avg[i] + fixedpt_mul(fixedpt_rconst(encoder_vel[i]), ENCODER_FILTER_GAIN_FIXEDPT),
                                 fixedpt_fromint(1) + ENCODER_FILTER_GAIN_FIXEDPT);

        // Slew target velocity
        fixedpt slew_error = target_velocity[i] - slewed_velocity[i];
        if (fixedpt_abs(slew_error) < MAX_SLEW_RATE_FIXEDPT)
            slewed_velocity[i] = target_velocity[i];
        else
            slewed_velocity[i] += fixedpt_mul(MAX_SLEW_RATE_FIXEDPT,
                                              (fixedpt_div(slew_error, fixedpt_abs(slew_error))));  // slew_error != 0

        // Clamp max velocity
        slewed_velocity[i] = MAX(MIN(MAX_SPEED_FIXEDPT, slewed_velocity[i]), -MAX_SPEED_FIXEDPT);

        // Handle voltage regulation
        voltage_limit[i] = fixedpt_rconst(0.0f);

        if (slewed_velocity[i] > fixedpt_rconst(0.1f))
            voltage_limit[i] =
                fixedpt_mul(fixedpt_abs(slewed_velocity[i] - vel_avg[i]), KP_FIXEDPT) +
                fixedpt_mul(fixedpt_abs(slewed_velocity[i] - MINIMUM_DESIGN_VEL_FIXEDPT), FF_LINEAR_FIXEDPT) +
                VOLTAGE_BASELINE_FIXEDPT;
        else if (slewed_velocity[i] < fixedpt_rconst(-0.1f)) {
            voltage_limit[i] =
                fixedpt_mul(fixedpt_abs(vel_avg[i] - slewed_velocity[i]), KP_FIXEDPT) +
                fixedpt_mul(fixedpt_abs(slewed_velocity[i] - MINIMUM_DESIGN_VEL_FIXEDPT), FF_LINEAR_FIXEDPT) +
                VOLTAGE_BASELINE_FIXEDPT;
        }

        // Startup voltage boost
        if (fixedpt_div(vel_avg[i], slewed_velocity[i]) < fixedpt_rconst(0.5f) &&
            fixedpt_abs(slewed_velocity[i]) > fixedpt_rconst(0.1f)) {
            voltage_limit[i] = voltage_limit[i] + STARTUP_VOLTAGE_BOOST_FIXEDPT;
        }

        // Enforce max voltage limit
        voltage_limit[i] = MIN(voltage_limit[i], DRIVER_VOLTAGE_SUPPLY_FIXEDPT);
    }

    // Kill motor power immediately if we haven't gotten a new command in a while
    if (time_reached(rps_expiration)) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            slewed_velocity[i] = 0;
            voltage_limit[i] = 0;
        }
    }

    // Send motor commands (under spin lock)
    core1_update_targets(slewed_velocity, voltage_limit);
}

void controller_set_target(const float *rps) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        target_velocity[i] = fixedpt_rconst(rps[i]);
    }

    // Set new timeout for command staling
    rps_expiration = make_timeout_time_ms(RPS_MIN_UPDATE_RATE_MS);
}

const float *controller_get_encoders_vel() {
    return encoder_vel;
}

const float *controller_get_encoders_angle() {
    return encoder_angle;
}
