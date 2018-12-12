#ifndef AIRBRAKES_CONTROLL_H
#define AIRBRAKES_CONTROLL_H
#include "rocket.h"

//structs
typedef struct Parameters_t {
  float kpp;
  float kpd;
  float kpi;
}Parameters;

//functions
float calk_ref(int altitude);
float controller(Rocket* rocket, Parameters* parameters, float* riemann_sum, float* derivative);
float integrate(float prev_sum, float value, float step);
float derivate(float prev_val, float cur_val, float step);

#endif
