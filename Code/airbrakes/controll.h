#ifndef AIRBRAKES_CONTROLL_H
#define AIRBRAKES_CONTROLL_H
#include "rocket.h"

//structs
typedef struct Parameters_t {
  float kpp;
  float kpi;
  float kpd;
}Parameters;

// reference height, velocity pair. Height is index and 
extern const float reference[3300];

float controller(Rocket* rocket, float* error, Parameters* parameters, float* riemann_sum, float dt, float height, float velocity);
float integrate(float prev_sum, float value, float step);

#endif
