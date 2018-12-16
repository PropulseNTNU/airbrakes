#include "controll.h"
#include "Arduino.h"
#include "rocket.h"
#include <NewPing.h>

float calk_ref(int altitude){
  if (altitude < 2000){
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

float controller(Rocket* rocket, float* error, Parameters* parameters, float* riemann_sum, float* derivative, NewPing* sonar, float dt){
  
  float new_error = (float)sonar->ping_cm() - (float)11;
  *riemann_sum = integrate(*riemann_sum, new_error, dt);
  
  // a test to handle integral windup
  if(*riemann_sum > SERVO_180){
     *riemann_sum = SERVO_180;
  }
  else if(*riemann_sum < SERVO_0){
     *riemann_sum = SERVO_0;
  }
  *derivative = derivate(*error, new_error, dt);
  *error = new_error;
  Serial.print("Kpp * error: ");
  Serial.println((*error));
  Serial.print("Kpi * riemann_sum: ");
  Serial.println(parameters->kpi * (*riemann_sum));
  Serial.print("Kpd * derivative: ");
  Serial.println(parameters->kpd * (*derivative));
  Serial.print("Dt");
  Serial.println(dt);
  return (parameters->kpp*(*error)) + (parameters->kpi*(*riemann_sum)) + (parameters->kpd*(*derivative));
}
