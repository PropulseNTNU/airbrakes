

#include "controll.h"
#include "Arduino.h"
#include "rocket.h"

float drag= 20;
float reference_v= 200;
const float mass= 30;

Matrix<2,2> A_d;
Matrix<2,1> B_d;
const Matrix<1,2> C_d={1,0};
const Matrix<2,2> E_d={0.00001,0.0000002,0.000000004,0.0000003};
const Matrix<2,2> Q={3,0 ,0 ,0.0000001};
const float R=0.0000001; //Litt usikker på om R skal være en matrise eller en skalar. trodde egentlig den skulle være en matrise.
const Matrix<2,2> I={1,0,0,1};
Matrix<2,2> P_k_bar={1,0,0.1,0};
Matrix<2,1>x_hat_bar={2000,300};
Matrix<2,1> x_hat = {0,0};
Matrix<2,1> K_k;
Matrix<2,1> Z;
Matrix<2, 2> P_k;

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

float controller(Rocket* rocket, float* error, Parameters* parameters, float* riemann_sum, float dt, float height, float velocity){
  reference_v = getReferenceVelocity(height);
  kalman(rocket, 0, 0, dt, reference_v);
  *error = getReferenceVelocity(height) - velocity;
  // add a lower and upper bound to prevernt overflow
  *riemann_sum = integrate(*riemann_sum, *error, dt);
  return (parameters->kpp*(*error)) + (parameters->kpi*(*riemann_sum));
}

float kalman(Rocket* rocket, float  altitude, float acceleration, float dt, float reference_v){
  //Updating variables
  //reference_v=rocket.getReferenceVelocity();
  A_d={0,1,0,-dt*(drag*reference_v/mass)};
  B_d={0,dt};

  //Computing kalman gain------------------------------------------------------
  K_k = P_k_bar*(~C_d)*((C_d*P_k_bar*(~C_d)+R).Inverse());// litt usikker på om K_k skal være matrise eller skalar
  Serial << "K_k" << K_k << '\n';
  //Update estimate with measurement-------------------------------------------
  Z = {rocket->getBarometer(), 0};
  x_hat = x_hat_bar + K_k*(rocket->getBarometer() -(x_hat_bar(0)));
  Serial << "x_hat: " << x_hat << '\n';
  //updatet estimates----------------------------------------------------------
  rocket->setAltitude(x_hat(0));
  rocket-> setVelocity(x_hat(1));
  //Compute  error  covariance  for  updated  estimate-------------------------
  P_k = (I - K_k*C_d)*P_k_bar *(~(I-K_k*C_d)) + K_k*R*(~K_k);
  Serial << "P_k: " << P_k << '\n';
  //project ahead--------------------------------------------------------------
  x_hat_bar = A_d * x_hat + B_d * (rocket->getAcceleration());
  P_k_bar = A_d * P_k * (~A_d) + E_d * Q * (~E_d);
  Serial << "P_k_bar: " << P_k_bar << '\n';
  Serial << "x_hat_bar: " << x_hat_bar << '\n';
}
