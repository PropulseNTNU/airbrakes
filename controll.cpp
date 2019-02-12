

#include "controll.h"



float integrate(float prev_sum, float value, float step){
  return prev_sum + (value * step);
}

float controller(float* error, Parameters* parameters, float* riemann_sum, float dt){
  //reference_v = getReferenceVelocity(height);
  //kalman(rocket, 0, 0, dt, reference_v);
  //*error = getReferenceVelocity(height) - velocity;
  // add a lower and upper bound to prevernt overflow
  *riemann_sum = integrate(*riemann_sum, *error, dt);
  return (parameters->kpp*(*error)) + (parameters->kpi*(*riemann_sum));
}
