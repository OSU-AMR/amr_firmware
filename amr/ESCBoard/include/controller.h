#ifndef CONTROLLER_H
#define CONTROLLER_H

void controller_init();

void controller_tick();

void controller_set_target(const float *rps);

#endif  // CONTROLLER_H
