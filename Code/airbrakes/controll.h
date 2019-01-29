#ifndef AIRBRAKES_CONTROLL_H
#define AIRBRAKES_CONTROLL_H
#include "rocket.h"
#include <NewPing.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

//structs
typedef struct Parameters_t {
  float kpp;
  float kpi;
  float kpd;
}Parameters;


//functions
float calk_ref(int altitude);
// the sonar parameter is just for demo testing
float controller(Rocket* rocket,float* error, Parameters* parameters, float* riemann_sum, float* derivative, NewPing* sonar, float dt);
float integrate(float prev_sum, float value, float step);
float derivate(float prev_val, float cur_val, float step);
float kalman(Rocket* rocket,float  altitude, float acceleration);

#endif
