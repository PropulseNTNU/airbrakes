#include <Wire.h>
#include "airbrakes_calculations.h"
#include "airbrakes_sensors.h"
#include "airbrakes_setup.h"
#include "rocket.h"
#include "controll.h"
#include "MPU9250.h"

// The rocket object representing the rockets state
// we may need to clean this using delete but im not sure if nessesary or when to do it
Rocket* rocket;

// MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

float riemann_sum = 0;
float derivative = 0;
float  u;
Parameters parameters = { 5, 5, 5 };

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  rocket = Rocket::Instance();
  //imuInit(IMU);
  Serial.begin(9600);
  while(!Serial) {};
}

void loop() {  
  u = controller(rocket, &parameters, &riemann_sum, &derivative);
  rocket->setAirbraksesPosition(u);
}
