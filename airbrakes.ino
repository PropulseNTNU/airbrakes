#include <Servo.h>

#include "airbrakes_setup.h"
#include "controll.h"
#include "kalman.h"
#include "interpolation.h"

#include "serial_reader.h"


Servo _servo;

//initilises variables
float error = 0; //error used in controller
float riemann_sum = 0; //used in integrator, witch is used in controller
float u = 0; //sets servo to 45 degrees, this causes the air brakes to brake at 50% capasaty
float prev_u=0;//only used for testing
float dt = 0; //time step used in integrator and kalman filter
Parameters parameters = { 1 , 1 , 1 }; //Control parameters (Kp, Ki, Kd)
unsigned long time_new, time_old = 0; // time variables for delta time

float sensor_data[2]={0,0}; //Barometer at index 0 and accelrometer (z-direction)at index 1. Utvides kanskje senere m/pitch
float estimates[2]; //Estimates from Kalman filter. [height, velocity]
float reference_v= 0; //reference_velovity

void setup() { //initiates servo and printing
  pinMode(SERVO_PIN, OUTPUT);
  _servo.attach(SERVO_PIN);
  Serial.begin(9600);
  while(!Serial) {};
}


void loop() { //Main-loop. Will be replaced with the loop in the statemachine.
  //Updats dt
  time_new = micros();
  dt = (float)(time_new - time_old);
  dt /= (float)1000000; // converted to seconds

  updateSensorData(sensor_data);
  
  Serial.print("t_h");
  Serial.println(sensor_data[0]);
  Serial.print("t_a");
  Serial.println(sensor_data[1]);
        
  kalman(estimates, sensor_data[0], sensor_data[1], dt, reference_v);

  Serial.print("est_h");
  Serial.println(estimates[0]);

  Serial.print("est_v");
  Serial.println(estimates[1]);
  
  reference_v=getReferenceVelocity(estimates[0]);
  error=reference_v-estimates[1];
  u = controller(&error, &parameters, &riemann_sum, dt); //updates controll signal
  time_old = time_new;
  prev_u=test_modifications(u, prev_u, dt);
  u=prev_u;
  if (u>90){
    u=90;
  }
  if (u<0){
    u=0;
  }
  Serial.print("c_s");
  Serial.println(test_calculate_area(u),5);
  Serial.print("itime");
  Serial.println(dt,5);
  delay(10);
}
