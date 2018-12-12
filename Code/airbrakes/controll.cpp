#include "controll.h"
#include "Arduino.h"
#include "rocket.h"

float calk_ref(int altitude){
  if (altitude<2000){
    return -1;
  }
  return (-0.5*altitude+500);//random function
}

float integrate(float prev_sum, float value, float step){
  return prev_sum + (value * step);
}

float derivate(float prev_val, float cur_val, float step){
  return (cur_val - prev_val)/step;
}

float controller(Rocket* rocket, Parameters* parameters, float* riemann_sum, float* derivative){
  float error = abs(calk_ref(rocket->getAltitude()) - rocket->getVelocity());
  *riemann_sum = integrate(*riemann_sum, error, 1);
  float prev_velocity = rocket->getVelocity();
  delay(10);
  float cur_velocity = rocket->getVelocity();
  *derivative = derivate(prev_velocity, cur_velocity, 1);
  return (parameters->kpp * error) + (parameters->kpi*(*riemann_sum)) + (parameters->kpd*(*derivative));
}
