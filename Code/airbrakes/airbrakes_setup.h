#ifndef _AIRBRAKES_SETUP_H  // Include guard.
#define _AIRBRAKES_SETUP_H  

#define SLAVESELECT_PIN 20
#define SERVO_PIN 9
#define SERVO_0 1050 // position 0 is set when we send a high signel in 1000 microseconds(1ms)
#define SERVO_180 1950 // position 180 is set when we send a high in 2000 microseconds(2ms)
#define SERVO_PERIOD 20 // the servo signal period in milliseconds
#define TRIGGER_PIN  13 // trigger pin for distance sensor
#define ECHO_PIN     12 // echo pin for distance sensor

#endif
