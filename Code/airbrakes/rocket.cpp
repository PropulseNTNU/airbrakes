#include "rocket.h"
#include <stddef.h> 

Rocket* Rocket::m_pInstance = NULL;

Rocket* Rocket::Instance(){
    if(!m_pInstance){
        m_pInstance = new Rocket(0,0,0,0,0,false,true);
    }
    return m_pInstance;
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
    // this is maybe not necesary sice the returned valus is the value last written to _servo
    _airbrakes_position = _servo.read();
    return _airbrakes_position;
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
        _airbrakes_position = airbrakes_position;
        _servo.write(_airbrakes_position);
        return true;
    }
    return false;

}
