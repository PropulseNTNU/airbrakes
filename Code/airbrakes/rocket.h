#ifndef AIRBRAKES_ROCKET_H
#define AIRBRAKES_ROCKET_H
#include <Servo.h>
#include "airbrakes_setup.h"
class Rocket
{
    public:
        static Rocket* Instance();
        bool setAltitude(float altitude);
        bool setAcceleration(float acceleration);
        bool setAttitude(float attitude);
        bool setVelocity(float velocity);
        bool setAirbraksesPosition(int position);
        bool setActiveState(bool active_state);
        bool setAirbrakesNeutral(bool airbrakes_neutral);

        float getAltitude();
        float getAcceleration();
        float getAttitude();
        float getVelocity();
        int   getAirbrakesPosition();
        bool getActiveState();
        bool getAirbrakesNeutral();
        
    private:
        // not sure if this is best practice when using private constructors
        Rocket(float altitude, float acceleration, float attitude, float velocity, int airbrakes_position, bool active_state, bool airbrakes_neutral){
            _altitude = altitude;
            _acceleration = acceleration; 
            _attitude = attitude;
            _velocity = velocity; 
            _airbrakes_position = airbrakes_position;
            _active_state = active_state;
            _airbrakes_neutral = airbrakes_neutral;
            _servo.attach(ACTUATOR_PIN);
        };
        Rocket(Rocket const&){};
        Rocket& operator=(Rocket const&){};
        static Rocket* m_pInstance;
        Servo _servo;
        float _altitude;
        float _acceleration;
        float _attitude;
        float _velocity;
        // we may not need this variable since the servo holds this value
        int   _airbrakes_position;
        // airbrakes_neutral is a boolean that is true if the airbrakes are in the default neutral position
        bool _airbrakes_neutral;
        // active_state is a boolean that is true if the rocket is in a state where airbrakes 
        // can be deployed
        bool _active_state;
};
#endif
