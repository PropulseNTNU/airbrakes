#ifndef AIRBRAKES_CONTROLL_H
#define AIRBRAKES_CONTROLL_H
#include <BasicLinearAlgebra.h>

using namespace BLA;

//structs
typedef struct Parameters_t {
  float kpp;
  float kpi;
  float kpd;
}Parameters;

//functions
float getReferenceVelocity(float height);
// the sonar parameter is just for demo testing
float controller(float* error, Parameters* parameters, float* riemann_sum, float dt);
float integrate(float prev_sum, float value, float step);
//functions for testing
float test_calculate_area(float u);
float test_modifications(float ref_u, float prev_u, float dt);//Only for testing
#endif
