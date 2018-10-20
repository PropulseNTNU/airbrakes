  
// header files that can be used for constants and functions so the main file is kept clean
#include "airbrakes_calculations.h"
#include "airbrakes_sensors.h"
#include "airbrakes_setup.h"
#include "rocket.h"

// SPI library is included for communication between the airbrakes and fc controllers
#include <SPI.h>

// the rocket object representing the rockets state
// we may need to clean this using delete but im not sure if nessesary or when to do it
Rocket* rocket;

void setup() {
  rocket = Rocket::Instance();
  // initialize SPI
  SPI.begin(); 
}

void loop() {
  rocket->setAirbraksesPosition(1);
  delay(1000);
  rocket->setAirbraksesPosition(179);
  delay(1000);
}
