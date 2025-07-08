#ifndef CONTROLLER_H
#define CONTROLLER_H

#define CONTROLLER_PERIOD_MS 10  // This frequency will cause us to miss some timer ticks, but that's ok for now

void controller_init();

void controller_tick();

void controller_set_target(const float *rps);

void controller_set_braking_voltage(const int index, const float voltage);

void controller_set_slew_rps2(const float slew_rps2);

const float *controller_get_encoders_vel();

const float *controller_get_encoders_angle();

#endif  // CONTROLLER_H
