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

float kalman(float  altitude, float acceleration){
  //Computing kalman gain------------------------------------------------------
  K_k = P_k_bar*(~C_d)*((C_d*P_k_bar*(~C_d)+R).Inverse());// litt usikker på om K_k skal være matrise eller skalar
  Seral.println(K_k);
  //Update estimate with measurement-------------------------------------------
  x_hat = x_hat_bar + K_k*((rocket->getBarometer()) -C_d*x_hat_bar);
  Serial << "x_hat: " << x_hat << '\n'
  //updatet estimates----------------------------------------------------------
  rocket->setAltitude(x_hat(0));
  rocket-> setVelocity(x_hat(1));
  //Compute  error  covariance  for  updated  estimate-------------------------
  P_k<2,2> = (I - K_k*C_d)*P_k_bar *(~(I-K_k*C_d)) + K_k*R*(~K_k);
  Serial << "P_k: " << P_k << '\n'
  //project ahead--------------------------------------------------------------
  x_hat_bar = A_d * x_hat + B_d * (rocket->getAcceleration());
  P_k_bar = A_d * P_k * (~A_d) + E_d * Q * (~E_d);
  Serial << "P_k_bar: " << P_k_bar << '\n'
  Serial << "x_hat_bar: " << x_hat_bar << '\n'
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
