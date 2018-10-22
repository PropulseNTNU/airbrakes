#include "airbrakes_calculations.h"
#include "airbrakes_sensors.h"
#include "airbrakes_setup.h"
#include "rocket.h"

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

void loop() {
  val = analogRead(potPin);
  val =  map(val, 0, 1023, 0, 180);
  rocket->setAirbraksesPosition(val);
  delay(40);

}
