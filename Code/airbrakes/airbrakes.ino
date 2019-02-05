#include "airbrakes_setup.h"
#include "rocket.h"
#include "controll.h"

Rocket* rocket;

float error = 0;
float riemann_sum = 0;
float u = 1500;
float dt = 0;
Parameters parameters = { 1 , 1 , 1 };

// time variables for delta time
unsigned long time_new, time_old = 0;


void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  rocket = Rocket::Instance();
  Serial.begin(9600);
  while(!Serial) {};
}

void loop() {
  time_new = micros();
  dt = (float)(time_new - time_old);
  dt /= (float)1000000; // converted to seconds
  u += controller(rocket, &error, &parameters, &riemann_sum, dt, 0, 1);
  time_old = time_new;
  rocket->setAirbraksesPosition(u);
}
