#include <Wire.h>
#include "airbrakes_calculations.h"
#include "airbrakes_sensors.h"
#include "airbrakes_setup.h"
#include "rocket.h"
#include "controll.h"
#include "MPU9250.h"

// the rocket object representing the rockets state
// we may need to clean this using delete but im not sure if nessesary or when to do it
Rocket* rocket;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);

const int potPin = A0;
int val = 0;
float riemann_sum = 0;
long timestep = 0;
float  u;
float prev_velocity;
float cur_velocity;
unsigned long time_start;

void setup() {
  pinMode(ACTUATOR_PIN, OUTPUT);
  rocket = Rocket::Instance();
  time_start = millis();
  cur_velocity=rocket->getVelocity();
  imuInit(IMU);
  Serial.begin(9600);
  while(!Serial) {};
}

void loop() {
  timestep = millis() - time_start;
  time_start = millis();
  prev_velocity = cur_velocity;
  cur_velocity = rocket->getVelocity();
  u = controller(calk_ref(rocket->getAltitude()), rocket->getVelocity(), timestep, &riemann_sum, cur_velocity, prev_velocity, 5, 5, 5);
  if (u>180) {
    rocket->setAirbraksesPosition(180);
  }
  else if (u < 0) {
    rocket->setAirbraksesPosition(0);
  }
  else {
    rocket->setAirbraksesPosition(u);
  }
  Serial.print(u);
}
