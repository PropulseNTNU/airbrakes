#include "controll.h"
#include "Arduino.h"
#include "rocket.h"
#include <NewPing.h>

Matrix<2,2> A_d={0,1,0,3};
Matrix<2,1> B_d={0,1};
Matrix<1,2> C_d={1,0};
Matrix<2,2> E_d={0.00001,0.0000002,0.000000004,0.0000003};
Matrix<2,2> Q={3,0 ,0 ,0.0000001};
float R=0.0000001; //Litt usikker på om R skal være en matrise eller en skalar. trodde egentlig den skulle være en matrise.
Matrix<2,2> I={1,0,0,1};
Matrix<2,2> P_k_bar={1,0,0.1,0};
Matrix<2,1>x_hat_bar={2000,300};
Matrix<2,1> x_hat = {0,0};

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

  kalman(rocket, 0, 0);

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

float kalman(Rocket* rocket, float  altitude, float acceleration){
  //Computing kalman gain------------------------------------------------------
  Matrix<2,1> K_k = P_k_bar*(~C_d)*((C_d*P_k_bar*(~C_d)+R).Inverse());// litt usikker på om K_k skal være matrise eller skalar
  Serial << "K_k" << K_k << '\n';
  //Update estimate with measurement-------------------------------------------
  Matrix<2,1> Z = {rocket->getBarometer(), 0};
  x_hat = x_hat_bar + K_k*(rocket->getBarometer() -(x_hat_bar(0)));
  Serial << "x_hat: " << x_hat << '\n';
  //updatet estimates----------------------------------------------------------
  rocket->setAltitude(x_hat(0));
  rocket-> setVelocity(x_hat(1));
  //Compute  error  covariance  for  updated  estimate-------------------------
  Matrix<2, 2> P_k = (I - K_k*C_d)*P_k_bar *(~(I-K_k*C_d)) + K_k*R*(~K_k);
  Serial << "P_k: " << P_k << '\n';
  //project ahead--------------------------------------------------------------
  x_hat_bar = A_d * x_hat + B_d * (rocket->getAcceleration());
  P_k_bar = A_d * P_k * (~A_d) + E_d * Q * (~E_d);
  Serial << "P_k_bar: " << P_k_bar << '\n';
  Serial << "x_hat_bar: " << x_hat_bar << '\n';
}
