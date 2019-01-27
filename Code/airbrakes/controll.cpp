#include "controll.h"
#include "Arduino.h"
#include "rocket.h"

// the planed optimal path precalculated. It will be represented as a array off height(index) -> velocity pairs
const float reference_velocity[5] = {100,75,60,55,52};

float getReferenceVelocity(float height){
  unsigned int x0 = floor(height);
  unsigned int x1 = x0 + 1;
  float y0 = reference_velocity[x0];
  float y1 = reference_velocity[x1];
  return ((y0 * (x1 - height)) + (y1 * (height - x0 ))) / (x1 - x0);
}

float integrate(float prev_sum, float value, float step){
  return prev_sum + (value * step);
}

float derivate(float prev_val, float cur_val, float step){
  return (cur_val - prev_val)/step;
}

float controller(Rocket* rocket, float* error, Parameters* parameters, float* riemann_sum, float dt, float height, float velocity){
  *error = getReferenceVelocity(height) - velocity;
  // add a lower and upper bound to prevernt overflow
  *riemann_sum = integrate(*riemann_sum, *error, dt);
  return (parameters->kpp*(*error)) + (parameters->kpi*(*riemann_sum));
}
