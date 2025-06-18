#ifndef CONTROLLER_H
#define CONTROLLER_H

void controller_init();

void controller_tick();

void controller_set_target(const float *rps);

const float *controller_get_encoders_vel();

const float *controller_get_encoders_angle();

#endif  // CONTROLLER_H
