#include "airbrakes_calculations.h"
#include "airbrakes_sensors.h"
#include "airbrakes_setup.h"
#include "rocket.h"
#include "controll.h"
// SPI library is included for communication between the airbrakes and fc controllers
#include <SPI.h>

// the rocket object representing the rockets state
// we may need to clean this using delete but im not sure if nessesary or when to do it
Rocket* rocket;

const int potPin = A0;
int val = 0;

void setup() {
  rocket = Rocket::Instance();
  // initialize SPI
  SPI.begin();

}
float riemann_sum=0;
long timestep = 0;
unsigned long time_start=millis();
float cur_velocity=rocket->getVelocity();
void loop() {
  timestep=millis()-time_start;
  time_start=millis();
  float prev_velocity=cur_velocity;
  cur_velocity=rocket->getVelocity();
  //val = analogRead(potPin);
  //val =  map(val, 0, 1023, 0, 180);
  //rocket->setAirbraksesPosition(val);
  //delay(40);
  float  u= controller(calk_ref(rocket->getAltitude()), rocket->getVelocity(), timestep, &riemann_sum, cur_velocity, prev_velocity, 5,5,5);
  prev_velocity=cur_velocity;
  if (u>180) {
    rocket->setAirbraksesPosition(180);
  }
  else if (u<0) {
    rocket->setAirbraksesPosition(0);
  }
  else{
    rocket->setAirbraksesPosition(u);
  }
  Serial.print(u);
}
