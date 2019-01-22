#include "controll.h"
#include "Arduino.h"
#include "rocket.h"

// the planed optimal path precalculated. It will be represented as a array off height(index) -> velocity pairs
const float reference_velocity[3300] = {0,0,0};

float integrate(float prev_sum, float value, float step){
  return prev_sum + (value * step);
}

float derivate(float prev_val, float cur_val, float step){
  return (cur_val - prev_val)/step;
}

float controller(Rocket* rocket, float* error, Parameters* parameters, float* riemann_sum, float dt, unsigned int height, float velocity){
  *error = reference_velocity[height] - velocity;
  // add a lower and upper bound to prevernt overflow
  *riemann_sum = integrate(*riemann_sum, *error, dt);
  return (parameters->kpp*(*error)) + (parameters->kpi*(*riemann_sum));
}
