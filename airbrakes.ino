#include "airbrakes_setup.h"
#include "controll.h"
#include "kalman.h"
#include "interpolation.h"
#include <Servo.h>
Servo _servo;


float error = 0;
float riemann_sum = 0;
float u = 1500;
float dt = 0;
Parameters parameters = { 1 , 1 , 1 };

// time variables for delta time
unsigned long time_new, time_old = 0;

float sensor_data[2]={0,0}; //Inneholder sensor data barometer på plass 0, akselrometer i z-retning på plass 1. Utvides kanskje senere m/pitch
float estimates[2]; //inneholder estimatene fra kalmanfilteret. høyde på plass 0, hastighet på plass 1.
float reference_v= 200;

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  _servo.attach(SERVO_PIN);
  Serial.begin(9600);
  while(!Serial) {};
}

void loop() {
  time_new = micros();
  dt = (float)(time_new - time_old);
  dt /= (float)1000000; // converted to seconds

  kalman(estimates, sensor_data[0], sensor_data[1], dt, reference_v);
  reference_v=getReferenceVelocity(estimates[0]);
  error=reference_v-estimates[1];
  u += controller(&error, &parameters, &riemann_sum, dt);
  time_old = time_new;
  if(u >= 0 && u <= 180) {
     _servo.write(u);
   }
}
