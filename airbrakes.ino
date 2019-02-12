#include "airbrakes_setup.h"
#include "controll.h"
#include "kalman.h"
#include "interpolation.h"
#include <Servo.h>
Servo _servo;

//initilises variables
float error = 0; //error used in controller
float riemann_sum = 0; //used in integrator, witch is used in controller
float u = 90; //sets servo to 90 degrees, this causes the air brakes to brake at 50% capasaty
float dt = 0; //time step used in integrator and kalman filter
Parameters parameters = { 1 , 1 , 1 }; //Control parameters (Kp, Ki, Kd)
unsigned long time_new, time_old = 0; // time variables for delta time

float sensor_data[2]={0,0}; //Barometer at index 0 and accelrometer (z-direction)at index 1. Utvides kanskje senere m/pitch
float estimates[2]; //Estimates from Kalman filter. [height, velocity]
float reference_v= 200; //reference_velovity

void setup() { //initiates servo and printing
  pinMode(SERVO_PIN, OUTPUT);
  _servo.attach(SERVO_PIN);
  Serial.begin(9600);
  while(!Serial) {};
}

void loop() { //Main-loop. Will be replaced with the loop in the statemachine.
  //Updats dt
  time_new = micros();
  dt = (float)(time_new - time_old);
  dt /= (float)1000000; // converted to seconds

  kalman(estimates, sensor_data[0], sensor_data[1], dt, reference_v);
  reference_v=getReferenceVelocity(estimates[0]);
  error=reference_v-estimates[1];
  u += controller(&error, &parameters, &riemann_sum, dt); //updates controll signal
  time_old = time_new;
  if(u >= 0 && u <= 180) {
     _servo.write(u); //updates servo position
   }
}
