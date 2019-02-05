#include "rocket.h"
#include <Servo.h>
#include "airbrakes_setup.h"
#include <stddef.h> 
Rocket* Rocket::_instance = NULL;
Servo _servo;

Rocket* Rocket::Instance(){
    if(!_instance){
        _instance = new Rocket(0,0,0,0,false,true);
       _servo.attach(SERVO_PIN);
    }
    return _instance;
}

float Rocket::getAltitude(){
    return _altitude;
}

float Rocket::getAcceleration(){
    return _acceleration;
}

float Rocket::getAttitude(){
    return _attitude;
}

float Rocket::getBarometer(){
    return _barometer;
}

float Rocket::getVelocity(){
    return _velocity;
}

bool Rocket::getAirbrakesNeutral(){
    return _airbrakes_neutral;
}

bool Rocket::getActiveState(){
    return _active_state;
}

int Rocket::getAirbrakesPosition(){
    // The angle of the servo, from 0 to 180 degrees. 
    // Here we may need to implement a way of reading the position with a sensor
    return _servo.read();
}

bool Rocket::setAltitude(float altitude){
    _altitude = altitude;
    return true;
}

bool Rocket::setAcceleration(float acceleration){
    _acceleration = acceleration;
    return true;
}

bool Rocket::setAttitude(float attitude){
    _attitude = attitude;
    return true;
}

bool Rocket::setVelocity(float velocity){
    _velocity = velocity;
    return true;
}

bool Rocket::setAirbrakesNeutral(bool airbrakes_neutral){
    _airbrakes_neutral = airbrakes_neutral;
    return _airbrakes_neutral;
}

bool Rocket::setActiveState(bool active_state){
    _active_state = active_state;
    return _active_state;
}

bool Rocket::setAirbraksesPosition(unsigned int airbrakes_position){
  if(airbrakes_position >= 0 && airbrakes_position <= 180) {
     _servo.write(airbrakes_position);
     return true;
  }
  return false;
}
