#include "rocket.h"
#include <stddef.h> 
#include <Servo.h>
#include "airbrakes_setup.h"

Rocket* Rocket::_instance = NULL;
Servo _servo;

Rocket* Rocket::Instance(){
    if(!_instance){
        _instance = new Rocket(0,0,0,0,false,true);
        _servo.attach(ACTUATOR_PIN);
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
    return _servo.read();;
}

bool Rocket::setAltitude(float altitude){
    _altitude = altitude;
    return true;
}

bool Rocket::setAcceleration(float acceleration){
    _acceleration = acceleration;
    return true;
}

bool Rocket::setAttitude(float acceleration){
    _acceleration = acceleration;
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

bool Rocket::setAirbraksesPosition(int airbrakes_position){
    if(airbrakes_position >=  0 && airbrakes_position <= 180){
        _servo.write(airbrakes_position);
        return true;
    }
    return false;
}
