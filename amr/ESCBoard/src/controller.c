#include "controller.h"

#include "core1.h"

#include "driver/quad_encoder.h"
#include "titan/logger.h"

#include <math.h>
#include <memory.h>

// Hardware parameters
#define ENCODER_TPR 2048

#define KP_FIXEDPT fixedpt_rconst(0.5f)
#define MINIMUM_DESIGN_VEL_FIXEDPT fixedpt_rconst(10.0f)
#define FF_LINEAR_FIXEDPT fixedpt_rconst(1.0f / 20.0f)
#define VOLTAGE_BASELINE_FIXEDPT fixedpt_rconst(1.0f)

// Controller parameters
#define ENCODER_FILTER_GAIN_FIXEDPT fixedpt_rconst(.01)
#define MAX_SPEED_FIXEDPT fixedpt_rconst(130.0)  // rad / sec
#define STARTUP_VOLTAGE_BOOST_FIXEDPT fixedpt_rconst(5.0)

// Hardware
encoder encoders[NUM_MOTORS];

// Internal state
fixedpt vel_avg[NUM_MOTORS];

fixedpt previous_loop_time;

fixedpt target_velocity[NUM_MOTORS];
fixedpt voltage_limit[NUM_MOTORS];

void controller_init() {
    core1_init();

    encoder_init(&encoders[0], ENC0_A_PIN, ENC0_B_PIN, ENCODER_TPR, false);
    encoder_init(&encoders[1], ENC1_A_PIN, ENC1_B_PIN, ENCODER_TPR, true);
}

void controller_tick() {
    // Do this math as a separate loop since the timing here is somewhat important
    // fixedpt time = fixedpt_rconst(to_us_since_boot(get_absolute_time()) * 1e-6f);
    for (int i = 0; i < NUM_MOTORS; i++) {
        vel_avg[i] = fixedpt_div(
            vel_avg[i] + fixedpt_mul(fixedpt_rconst(encoder_get_velocity(&encoders[i])), ENCODER_FILTER_GAIN_FIXEDPT),
            fixedpt_fromint(1) + ENCODER_FILTER_GAIN_FIXEDPT);
    }
    // previous_loop_time = time;

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Clamp max velocity
        target_velocity[i] = MAX(MIN(MAX_SPEED_FIXEDPT, target_velocity[i]), -MAX_SPEED_FIXEDPT);

        // Handle voltage regulation
        voltage_limit[i] = fixedpt_rconst(0.0f);

        if (target_velocity[i] > fixedpt_rconst(0.1f))
            voltage_limit[i] =
                fixedpt_mul(fixedpt_abs(target_velocity[i] - vel_avg[i]), KP_FIXEDPT) +
                fixedpt_mul(fixedpt_abs(target_velocity[i] - MINIMUM_DESIGN_VEL_FIXEDPT), FF_LINEAR_FIXEDPT) +
                VOLTAGE_BASELINE_FIXEDPT;
        else if (target_velocity[i] < fixedpt_rconst(-0.1f)) {
            voltage_limit[i] =
                fixedpt_mul(fixedpt_abs(vel_avg[i] - target_velocity[i]), KP_FIXEDPT) +
                fixedpt_mul(fixedpt_abs(target_velocity[i] - MINIMUM_DESIGN_VEL_FIXEDPT), FF_LINEAR_FIXEDPT) +
                VOLTAGE_BASELINE_FIXEDPT;
        }

        // Startup voltage boost
        if (fixedpt_div(vel_avg[i], target_velocity[i]) < fixedpt_rconst(0.5f) &&
            fixedpt_abs(target_velocity[i]) > fixedpt_rconst(0.1f)) {
            voltage_limit[i] = voltage_limit[i] + STARTUP_VOLTAGE_BOOST_FIXEDPT;
        }

        // Enforce max voltage limit
        voltage_limit[i] = MIN(voltage_limit[i], DRIVER_VOLTAGE_SUPPLY_FIXEDPT);
    }

    // LOG_INFO("Got unclamped voltages as %f, %f", fixedpt_tofloat(voltage_limit[0]),
    // fixedpt_tofloat(voltage_limit[1])); LOG_INFO("Got encoder vels as %f, %f", fixedpt_tofloat(vel_avg[0]),
    // fixedpt_tofloat(vel_avg[1]));

    // Send motor commands (under spin lock)
    core1_update_targets(target_velocity, voltage_limit);
}

void controller_set_target(const float *rps) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        target_velocity[i] = fixedpt_rconst(rps[i]);
    }
}
