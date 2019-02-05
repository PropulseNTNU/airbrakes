#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <Wire.h>
#include "airbrakes_calculations.h"
#include "airbrakes_sensors.h"
#include "airbrakes_setup.h"
#include "rocket.h"
#include "controll.h"
#include "MPU9250.h"
#include <NewPing.h>

// Distance sensor for a demo
NewPing sonar(TRIGGER_PIN, ECHO_PIN);

// The rocket object representing the rocket state
// we may need to clean this using delete but im not sure if nessesary or when to do it
Rocket* rocket;

// MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

float error = 0;
float riemann_sum = 0;
float derivative = 0;
float  u;

Parameters parameters = { 100, 2, 1 };

// time variables for delta time
unsigned long time_new, time_old;


void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  rocket = Rocket::Instance();
  //imuInit(IMU);
  Serial.begin(9600);
  while(!Serial) {};
}

void loop() {
  time_new = millis();
  delta_t=(float)(time_new - time_old)/1000;
  u = controller(rocket, &error, &parameters, &riemann_sum, &derivative, &sonar, delta_t);
  time_old = time_new;
  rocket->setAirbraksesPosition(u);
  Serial.print("Controll signal: ");
  Serial.println(u);
  delay(20);
}
